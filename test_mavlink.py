from pymavlink import mavutil
import time
import math
import random

# =========================
# CONFIG
# =========================

# Python gửi vào COM này
# App Qt mở cổng COM còn lại trong cặp COM ảo
PORT = "COM7"
BAUD = 115200

# Tọa độ giả lập gần TP.HCM / Tân Sơn Nhất
BASE_LAT = 10.8231
BASE_LON = 106.6297
BASE_ALT_M = 12.0

# Noise config for chart/UI testing.
ATTITUDE_NOISE_DEG = 2.5
GPS_NOISE_DEG = 0.000025
ALT_NOISE_M = 0.7
BATTERY_NOISE_PERCENT = 1.5

# =========================
# MAVLINK CONNECTION
# =========================

master = mavutil.mavlink_connection(
    PORT,
    baud=BAUD,
    source_system=1,
    source_component=1
)

print(f"Sending MAVLink to {PORT} at {BAUD}")

t0 = time.time()
seq = 0

while True:
    t = time.time() - t0

    # =========================
    # FAKE ATTITUDE DATA
    # =========================
    roll_rad = math.sin(t * 1.5) * 0.35       # khoảng ±20 độ
    pitch_rad = math.cos(t * 1.2) * 0.25      # khoảng ±14 độ
    yaw_rad = (t * 0.25) % (2 * math.pi)

    roll_rad += math.radians(random.uniform(-ATTITUDE_NOISE_DEG, ATTITUDE_NOISE_DEG))
    pitch_rad += math.radians(random.uniform(-ATTITUDE_NOISE_DEG, ATTITUDE_NOISE_DEG))
    yaw_rad += math.radians(random.uniform(-ATTITUDE_NOISE_DEG, ATTITUDE_NOISE_DEG))

    rollspeed = math.cos(t * 1.5) * 0.05
    pitchspeed = -math.sin(t * 1.2) * 0.04
    yawspeed = 0.25

    roll_deg = math.degrees(roll_rad)
    pitch_deg = math.degrees(pitch_rad)
    yaw_deg = math.degrees(yaw_rad)

    # =========================
    # FAKE GPS DATA
    # =========================
    # Cho drone bay vòng tròn nhỏ quanh vị trí gốc
    radius_deg = 0.0008

    lat = BASE_LAT + math.sin(t * 0.08) * radius_deg + random.uniform(-GPS_NOISE_DEG, GPS_NOISE_DEG)
    lon = BASE_LON + math.cos(t * 0.08) * radius_deg + random.uniform(-GPS_NOISE_DEG, GPS_NOISE_DEG)
    alt_m = BASE_ALT_M + 5.0 + math.sin(t * 0.5) * 3.0 + random.uniform(-ALT_NOISE_M, ALT_NOISE_M)

    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    alt_mm = int(alt_m * 1000)

    # GPS velocity fake
    vx = int(math.cos(t * 0.08) * 100)  # cm/s
    vy = int(math.sin(t * 0.08) * 100)  # cm/s
    vz = int(math.sin(t * 0.3) * 20)    # cm/s

    ground_speed_cms = int(math.sqrt(vx * vx + vy * vy))
    heading_cdeg = int((yaw_deg % 360) * 100)

    satellites_visible = 12
    fix_type = 3  # 0 no gps, 2 2D fix, 3 3D fix

    # =========================
    # FAKE BATTERY DATA
    # =========================
    voltage_v = 16.0 - 0.01 * t
    if voltage_v < 14.0:
        voltage_v = 14.0

    voltage_mv = int(voltage_v * 1000)
    current_ca = 1200  # centi-ampere, 1200 = 12A
    battery_percent = max(0, min(100, int(90 - t * 0.05 + random.uniform(-BATTERY_NOISE_PERCENT, BATTERY_NOISE_PERCENT))))

    # =========================
    # TIME
    # =========================
    time_boot_ms = int(t * 1000)

    # =========================
    # 1. HEARTBEAT
    # =========================
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE
    )

    # Test-only marker: virtual COM ports can ignore baud-rate mismatches.
    # The C++ app uses this STATUSTEXT to verify that the selected baud matches
    # the simulator baud when testing with virtual serial links.
    master.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_INFO,
        f"BAUD={BAUD}".encode("ascii")
    )

    # =========================
    # 2. ATTITUDE
    # roll, pitch, yaw đơn vị radian
    # =========================
    master.mav.attitude_send(
        time_boot_ms,
        roll_rad,
        pitch_rad,
        yaw_rad,
        rollspeed,
        pitchspeed,
        yawspeed
    )

    # =========================
    # 3. GPS_RAW_INT
    # lat/lon: degE7
    # alt: mm
    # eph/epv: cm
    # vel: cm/s
    # cog: cdeg
    # =========================
    master.mav.gps_raw_int_send(
        int(time.time() * 1_000_000),  # time_usec
        fix_type,
        lat_int,
        lon_int,
        alt_mm,
        100,                           # eph
        100,                           # epv
        ground_speed_cms,
        heading_cdeg,
        satellites_visible
    )

    # =========================
    # 4. GLOBAL_POSITION_INT
    # relative_alt: mm
    # vx/vy/vz: cm/s
    # hdg: cdeg
    # =========================
    master.mav.global_position_int_send(
        time_boot_ms,
        lat_int,
        lon_int,
        alt_mm,
        int((alt_m - BASE_ALT_M) * 1000),
        vx,
        vy,
        vz,
        heading_cdeg
    )

    # =========================
    # 5. SYS_STATUS
    # voltage_battery: mV
    # current_battery: cA
    # battery_remaining: %
    # =========================
    master.mav.sys_status_send(
        0,                  # onboard_control_sensors_present
        0,                  # onboard_control_sensors_enabled
        0,                  # onboard_control_sensors_health
        500,                # load, 500 = 50.0%
        voltage_mv,
        current_ca,
        battery_percent,
        0, 0, 0, 0, 0, 0
    )

    # =========================
    # 6. BATTERY_STATUS
    # voltages: mV từng cell, 65535 nếu không dùng
    # current_battery: cA
    # battery_remaining: %
    # =========================
    cell_voltage = int(voltage_mv / 4)

    voltages = [
        cell_voltage,
        cell_voltage,
        cell_voltage,
        cell_voltage,
        65535,
        65535,
        65535,
        65535,
        65535,
        65535
    ]

    master.mav.battery_status_send(
        0,                                      # id
        mavutil.mavlink.MAV_BATTERY_FUNCTION_ALL,
        mavutil.mavlink.MAV_BATTERY_TYPE_LIPO,
        0,                                      # temperature unknown
        voltages,
        current_ca,
        -1,                                     # current_consumed mAh unknown
        -1,                                     # energy_consumed hJ unknown
        battery_percent
    )

    print(
        f"GPS=({lat:.7f}, {lon:.7f}) "
        f"ALT={alt_m:.1f}m | "
        f"ROLL={roll_deg:.1f} "
        f"PITCH={pitch_deg:.1f} "
        f"YAW={yaw_deg:.1f} | "
        f"BAT={voltage_v:.2f}V {battery_percent}%"
    )

    time.sleep(0.1)
