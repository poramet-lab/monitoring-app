from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from typing import List, Dict, Any, Optional
import subprocess, glob, os, psutil, json, time

import re  # ใส้ไว้บนหัวไฟล์ ถ้ายังไม่มี

import time
from typing import Optional, Tuple

HOST_NETDEV = "/host/proc/net/dev"
HOST_ROUTE  = "/host/proc/net/route"

def _get_default_iface_from_route(route_path: str = HOST_ROUTE) -> Optional[str]:
    """
    คืนชื่ออินเทอร์เฟซ default route จาก /host/proc/net/route
    เลือกบรรทัดที่ Destination == '00000000'
    """
    try:
        with open(route_path, "r") as f:
            lines = f.read().strip().splitlines()
        # ข้าม header แถวแรก
        for line in lines[1:]:
            parts = line.split()
            # คอลัมน์: Iface  Destination  Gateway  Flags ...
            if len(parts) >= 2 and parts[1] == "00000000":
                return parts[0]
    except Exception:
        pass
    return None


def _read_netdev_bytes(iface: str, netdev_path: str = HOST_NETDEV) -> Optional[Tuple[int, int]]:
    """
    อ่าน rx_bytes / tx_bytes ของ iface จาก /host/proc/net/dev
    รูปแบบแถว: 'iface:  rx_bytes ... tx_bytes ...'
    คอลัมน์ rx=คอลัมน์ที่1 หลัง ':'  และ tx=คอลัมน์ที่9 หลัง ':'
    """
    try:
        with open(netdev_path, "r") as f:
            lines = f.read().strip().splitlines()
        for line in lines[2:]:  # ข้าม 2 header lines
            if ":" not in line:
                continue
            name, rest = line.split(":", 1)
            name = name.strip()
            if name == iface:
                cols = rest.split()
                rx_bytes = int(cols[0])
                tx_bytes = int(cols[8])
                return rx_bytes, tx_bytes
    except Exception:
        pass
    return None


# เก็บสถานะครั้งก่อนเพื่อคำนวณอัตรา
_net_prev = {"ts": None, "rx": None, "tx": None}

def net_rates_kbps_host() -> dict:
    """
    คำนวณอัตราเน็ตจาก host (kbps) ของ default iface
    ถ้าหา iface ไม่ได้ หรืออ่าน host ไม่ได้ จะ fallback ไป eth0 ในคอนเทนเนอร์
    คืนค่า: {"rx_kbps": float, "tx_kbps": float, "ifaces":[{"name":iface,"rx_kbps":...,"tx_kbps":...}]}
    """
    iface = _get_default_iface_from_route()
    rx, tx = None, None

    if iface:
        pair = _read_netdev_bytes(iface)
        if pair:
            rx, tx = pair

    # fallback: ใช้ psutil/net_io_counters ของคอนเทนเนอร์ (ถ้าคุณมีอยู่แล้ว) หรือให้เป็นศูนย์
    if rx is None or tx is None:
        try:
            # Fallback แบบง่าย: อ่านจาก psutil ของ container เอง
            import psutil
            c = psutil.net_io_counters()
            rx, tx = c.bytes_recv, c.bytes_sent
            iface = iface or "container"
        except Exception:
            return {"rx_kbps": 0.0, "tx_kbps": 0.0, "ifaces": []}

    now = time.time()
    prev_ts = _net_prev["ts"]
    prev_rx = _net_prev["rx"]
    prev_tx = _net_prev["tx"]

    _net_prev.update({"ts": now, "rx": rx, "tx": tx})

    if not prev_ts or prev_rx is None or prev_tx is None:
        # ครั้งแรกยังไม่มี baseline
        return {"rx_kbps": 0.0, "tx_kbps": 0.0, "ifaces": [{"name": iface, "rx_kbps": 0.0, "tx_kbps": 0.0}]}

    dt = max(1e-3, now - prev_ts)  # กันหารศูนย์
    rx_kbps = max(0.0, (rx - prev_rx) * 8.0 / 1000.0 / dt)
    tx_kbps = max(0.0, (tx - prev_tx) * 8.0 / 1000.0 / dt)

    return {"rx_kbps": rx_kbps, "tx_kbps": tx_kbps, "ifaces": [{"name": iface, "rx_kbps": rx_kbps, "tx_kbps": tx_kbps}]}


def _nvme_temp_for(ctrl: str) -> Optional[float]:
    """
    อ่านอุณหภูมิของคอนโทรลเลอร์ nvmeX ผ่าน sysfs: /sys/class/nvme/nvmeX/device/hwmon/*/temp*_input
    คืนค่าเป็น °C (float) หรือ None
    """
    base = f"/sys/class/nvme/{ctrl}/device/hwmon"
    for hw in glob.glob(os.path.join(base, "hwmon*")):
        for tf in glob.glob(os.path.join(hw, "temp*_input")):
            v = _read_file(tf)
            if v and v.isdigit():
                try:
                    return int(v) / 1000.0
                except Exception:
                    pass
    return None

def _nvme_first_ns(ctrl: str) -> Optional[str]:
    """
    คืนชื่อ namespace ตัวแรก เช่น nvme0n1 (ถ้ามี)
    """
    patt = os.path.join(f"/sys/class/nvme/{ctrl}", f"{ctrl}n*")
    nss = sorted(glob.glob(patt))
    if not nss:
        return None
    return os.path.basename(nss[0])

def _nvme_size_gb(ns: str) -> Optional[float]:
    """
    อ่านขนาด namespace ผ่าน /sys/block/<ns>/size (sector * 512) แปลงเป็น GB (ฐาน 10^9)
    """
    sz_path = f"/sys/block/{ns}/size"
    s = _read_file(sz_path)
    if s and s.isdigit():
        try:
            size_bytes = int(s) * 512
            return round(size_bytes / 1_000_000_000, 2)
        except Exception:
            return None
    return None

def nvme_sysfs_inventory(sensors: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    สรุปข้อมูล SSD ต่อคอนโทรลเลอร์ (ไม่ใช้ nvme-cli สำหรับโมเดล/ไซส์)
    และ 'ผสาน' ค่า SMART (ถ้าอ่านได้) ต่อไดรฟ์: /dev/nvme0, /dev/nvme1, ...
    fields: ctrl, namespace, model, serial, firmware, size_gb, temp_c, alias
            + life_used_percent, written_tb, read_tb, power_on_hours, unsafe_shutdowns, media_errors
    """

    # รวมอุณหภูมิ NVMe จาก sensors (ใช้เฉพาะ Composite ตามลำดับ)
    nvme_composites: List[float] = []
    for s in sensors or []:
        chip = str(s.get("chip", "")).lower()
        label = str(s.get("label", "")).lower()
        if chip.startswith("nvme") and "composite" in label:
            try:
                nvme_composites.append(float(s.get("temp_c")))
            except Exception:
                pass

    out: List[Dict[str, Any]] = []
    ctrls = sorted(glob.glob("/sys/class/nvme/nvme*"))  # nvme0, nvme1, ...

    for idx, path in enumerate(ctrls):
        ctrl = os.path.basename(path)                    # "nvme0"
        ns   = _nvme_first_ns(ctrl)                      # "nvme0n1"
        model    = _read_file(os.path.join(path, "model")) or ""
        serial   = _read_file(os.path.join(path, "serial")) or ""
        firmware = _read_file(os.path.join(path, "firmware_rev")) or _read_file(os.path.join(path, "firmware")) or ""
        size_gb  = _nvme_size_gb(ns) if ns else None

        # temp: hwmon ก่อน, ไม่ได้ค่อย fallback เป็น Composite ตามลำดับ
        temp_c = _nvme_temp_for(ctrl)
        if temp_c is None and nvme_composites:
            use_idx = idx if idx < len(nvme_composites) else len(nvme_composites) - 1
            temp_c = nvme_composites[use_idx]

        # อ่าน SMART ของไดรฟ์นี้
        smart = _nvme_smart(f"/dev/{ctrl}")

        # ตั้งชื่อเล่นง่าย ๆ จากความจุ (จะย้ายไป map ด้วย serial ทีหลังได้)
        alias = None
        if size_gb is not None:
            if 900 <= size_gb <= 1100:
                alias = "OS"
            elif 1800 <= size_gb <= 2200:
                alias = "Data"

        out.append({
            "ctrl": ctrl,
            "namespace": ns,
            "model": model.strip(),
            "serial": serial.strip(),
            "firmware": firmware.strip(),
            "size_gb": size_gb,
            "temp_c": temp_c if temp_c is not None else (smart or {}).get("temperature_c"),
            "alias": alias,
            # SMART (อาจเป็น None ถ้าอ่านไม่ได้)
            "life_used_percent": (smart or {}).get("percentage_used"),
            "written_tb": (smart or {}).get("data_written_tb"),
            "read_tb": (smart or {}).get("data_read_tb"),
            "power_on_hours": (smart or {}).get("power_on_hours"),
            "unsafe_shutdowns": (smart or {}).get("unsafe_shutdowns"),
            "media_errors": (smart or {}).get("media_errors"),
        })

    return out



app = FastAPI(title="PrivateAI Hub Metrics")

# เปิด CORS ให้หน้าเว็บเรียก API ได้ (ต่างพอร์ต)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],   # ถ้าจะล็อก: ["http://192.168.10.138:8001"]
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------- Aliases (friendly names) ----------
_ALIAS_PATH = "/app/config/sensor_aliases.json"
_alias_cache: Optional[Dict[str, Any]] = None
_alias_mtime: Optional[float] = None

def _load_aliases() -> Dict[str, Any]:
    global _alias_cache, _alias_mtime
    try:
        mtime = os.path.getmtime(_ALIAS_PATH)
    except FileNotFoundError:
        _alias_cache, _alias_mtime = {"overrides": []}, None
        return _alias_cache
    if _alias_cache is None or _alias_mtime != mtime:
        try:
            with open(_ALIAS_PATH, "r", encoding="utf-8") as f:
                _alias_cache = json.load(f)
        except Exception:
            _alias_cache = {"overrides": []}
        _alias_mtime = mtime
    return _alias_cache

def _match_rule(sensor: Dict[str, Any], rule: Dict[str, Any]) -> bool:
    chip_ok  = ("chip"  not in rule) or (str(sensor.get("chip",""))  == str(rule.get("chip")))
    label_ok = ("label" not in rule) or (str(sensor.get("label","")) == str(rule.get("label")))
    return chip_ok and label_ok

def apply_aliases(sensors: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    cfg = _load_aliases()
    rules = cfg.get("overrides", []) if isinstance(cfg, dict) else []
    out = []
    for s in sensors:
        alias = None
        for r in rules:
            if _match_rule(s, r):
                alias = r.get("name")
                if alias: break
        d = dict(s)
        if alias:
            d["alias"] = alias
        out.append(d)
    return out

# ---------- System readers ----------
def _read_file(path: str) -> Optional[str]:
    try:
        with open(path) as f:
            return f.read().strip()
    except Exception:
        return None

def hwmon_temps() -> List[Dict[str, Any]]:
    """อ่านอุณหภูมิจากทุกชิปใน /sys/class/hwmon"""
    results: List[Dict[str, Any]] = []
    for hw in glob.glob("/sys/class/hwmon/hwmon*"):
        chip = _read_file(os.path.join(hw, "name")) or "unknown"
        for tfile in glob.glob(os.path.join(hw, "temp*_input")):
            base = os.path.dirname(tfile)
            num  = os.path.basename(tfile).split("_")[0]  # e.g., temp1
            label = _read_file(os.path.join(base, f"{num}_label")) or num
            val   = _read_file(tfile)
            try:
                if val is not None:
                    c = int(val) / 1000.0
                    results.append({"chip": chip, "label": label, "temp_c": c})
            except Exception:
                continue
    return results

def nv_gpu() -> Optional[Dict[str, Any]]:
    """อ่านข้อมูล GPU ผ่าน nvidia-smi (ต้องรันคอนเทนเนอร์ด้วย gpus: all)"""
    try:
        out = subprocess.check_output([
            "nvidia-smi",
            "--query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu,power.draw",
            "--format=csv,noheader,nounits"
        ], stderr=subprocess.STDOUT, timeout=2.5)
        line = out.decode().strip().splitlines()[0]
        name, util, mu, mt, temp, power = [x.strip() for x in line.split(",")]
        return {
            "name": name,
            "usage_percent": float(util),
            "vram_used_mib": float(mu),
            "vram_total_mib": float(mt),
            "temp_c": float(temp),
            "power_w": float(power)
        }
    except Exception:
        return None

# === Helpers สำหรับ RAM DIMM max และ SSD filesystem usage ===
# === Helpers สำหรับ RAM DIMM max และ SSD filesystem usage ===
import psutil

def ram_dimm_max_c(sensors):
    """หาอุณหภูมิสูงสุดของ DIMM จากชิปตระกูล spd* (เช่น spd5118)"""
    if not sensors:
        return None
    temps = [float(s.get("temp_c")) for s in sensors
             if isinstance(s, dict)
             and "temp_c" in s
             and isinstance(s.get("temp_c"), (int, float))
             and "chip" in s
             and str(s["chip"]).lower().startswith("spd")]
    return round(max(temps), 1) if temps else None

def fs_usage_gb(path):
    """คืนค่า (used_gb, total_gb) แบบ GB (ฐาน 1000) ปัดเป็นจำนวนเต็ม"""
    try:
        du = psutil.disk_usage(path)
        used_gb  = int(round(du.used / 1000**3, 0))
        total_gb = int(round(du.total / 1000**3, 0))
        return used_gb, total_gb
    except Exception:
        return None, None

def add_fs_usage_to_ssd(ssds):
    """เติมฟิลด์ fs_used_gb / fs_total_gb ให้แต่ละ SSD ตาม alias → mount path"""
    if not isinstance(ssds, list):
        return ssds
    alias_to_path = {"OS": "/", "Data": "/mnt/data"}
    out = []
    for d in ssds:
        d = dict(d)
        mnt = alias_to_path.get(d.get("alias"))
        if mnt:
            used, total = fs_usage_gb(mnt)
            if used is not None and total is not None:
                d["fs_used_gb"]  = used
                d["fs_total_gb"] = total
        out.append(d)
    return out





# ---------- API ----------
@app.get("/")
def root():
    return {"ok": True}

# --- Network RX/TX rate (kbps) ---
_prev_net = None  # (timestamp, {iface: (rx_bytes, tx_bytes)})

def _snapshot_net():
    # อ่านตัวนับ bytes ของทุก iface (ยกเว้น loopback)
    counters = psutil.net_io_counters(pernic=True)
    snap = {}
    for name, c in counters.items():
        if name.lower().startswith("lo"):
            continue
        snap[name] = (c.bytes_recv, c.bytes_sent)
    return snap

def net_rates_kbps():
    global _prev_net
    now = time.time()
    snap = _snapshot_net()
    if _prev_net is None:
        _prev_net = (now, snap)
        return {"rx_kbps": None, "tx_kbps": None, "ifaces": []}

    prev_t, prev_snap = _prev_net
    dt = max(1e-6, now - prev_t)
    ifaces = []
    rx_total = tx_total = 0.0
    for name, (rx, tx) in snap.items():
        prx, ptx = prev_snap.get(name, (rx, tx))
        rx_kbps = (max(0, rx - prx) * 8.0) / dt / 1000.0
        tx_kbps = (max(0, tx - ptx) * 8.0) / dt / 1000.0
        ifaces.append({"name": name, "rx_kbps": rx_kbps, "tx_kbps": tx_kbps})
        rx_total += rx_kbps
        tx_total += tx_kbps

    _prev_net = (now, snap)
    return {"rx_kbps": rx_total, "tx_kbps": tx_total, "ifaces": ifaces}

def _nvme_smart(dev: str) -> Optional[Dict[str, Any]]:
    """
    อ่าน SMART ผ่าน nvme-cli; คืน dict แบบย่อสำหรับการ์ด SSD Health
    """
    try:
        out = subprocess.check_output(
            ["nvme", "smart-log", "-o", "json", dev],
            stderr=subprocess.STDOUT, timeout=2.5
        )
        j = json.loads(out.decode())
        # แปลงค่าตามสเปค: data_units_* หน่วยละ 512,000 bytes
        def units_to_tb(units: int) -> float:
            return round((units * 512000) / 1_000_000_000_000, 2)

        return {
            "percentage_used": j.get("percentage_used"),
            "data_written_tb": units_to_tb(j.get("data_units_written", 0)),
            "data_read_tb": units_to_tb(j.get("data_units_read", 0)),
            "power_on_hours": j.get("power_on_hours"),
            "unsafe_shutdowns": j.get("unsafe_shutdowns"),
            "media_errors": j.get("media_errors"),
            "temperature_c": round((j.get("temperature") or 0) / 1000, 1) if isinstance(j.get("temperature"), (int,float)) else None
        }
    except Exception:
        return None


@app.get("/metrics")
def metrics():
    mem = psutil.virtual_memory()
    sensors = hwmon_temps()

    # SSD + เติม usage เป็น GB
    ssds = nvme_sysfs_inventory(sensors)
    ssds = add_fs_usage_to_ssd(ssds)

    payload = {
        "gpu": nv_gpu(),
        "ram": {
            "used_gb": round(mem.used / 1024**3, 2),
            "total_gb": round(mem.total / 1024**3, 2),
            "percent": round(mem.percent, 0),
            "temp_dimm_max_c": ram_dimm_max_c(sensors),  # DIMM max
        },
        "sensors": sensors,
        "net": net_rates_kbps_host(),   # (rx_kbps / tx_kbps)
        "ssd": ssds,               # มี fs_used_gb / fs_total_gb เพิ่ม
    }
    return payload


@app.get("/inventory")
def inventory():
    sensors = apply_aliases(hwmon_temps())
    # กลุ่มตาม chip
    by_chip: Dict[str, Dict[str, Any]] = {}
    for s in sensors:
        chip = str(s.get("chip","unknown"))
        by_chip.setdefault(chip, {"chip": chip, "count": 0, "labels": set()})
        by_chip[chip]["count"] += 1
        by_chip[chip]["labels"].add(str(s.get("label","")))
    # แปลง set -> list
    summary = []
    for chip, d in by_chip.items():
        summary.append({
            "chip": chip,
            "count": d["count"],
            "labels": sorted(list(d["labels"]))
        })
    return {"summary": sorted(summary, key=lambda x: x["chip"]), "samples": sensors[:100]}


