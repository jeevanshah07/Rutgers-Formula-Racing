# RFR VN-300 Full Data Logger — Rutgers Formula Racing
# Run: py -3.11 rfr_vn300_logger.py
# Logs GPS, IMU, INS, and attitude data to a timestamped CSV file
import argparse
import math
from vectornav import Sensor, Registers
from datetime import datetime  # gives us current date and time
import csv  # csv is python's built-in library for writing CSV files
import sys  # sys lets us exit the program if connection fails
import logging
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger(__name__)


def distance_meters(lat1, lon1, lat2, lon2):
    # Haversine formula to calculate distance between two lat/lon points in meters (Curved distance between 2GPS points)
    R = 6371000  # Earth radius in meters
    dlat = math.radians(lat2-lat1)
    dlon = math.radians(lon2-lon1)
    a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * \
        math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
    return R * 2 * math.asin(math.sqrt(a))

# gives us 2 things we need from VectorNav SDK library


# argparse lets us pass PORT and RATE from the terminal when running the script
# CONFIG Argumentparser handles reading terminal input
parser = argparse.ArgumentParser(description="VN-300 Data Logger")
parser.add_argument("--port", default="COM3",
                    help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
parser.add_argument("--rate", type=int, default=40, help="Rate Divisor")
args = parser.parse_args()
PORT = args.port
RATE = args.rate
# creates a new filename everytime the script runs
FILENAME = f"rfr_vn300_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# LAP TIMER CONFIG
START_LAT = 40.52653
START_LON = -74.46526
THRESHOLD_METERS = 5.0
COOL_DOWN_SECONDS = 20

# CSV SETUP COLS is the list that defines every coloumns in the CSV file, order = The order coloumns appear in the Excel
# Each string is a coloumns header name
COLS = [
    "timestamp",                           # Local system time
    "startup_ns", "gps_utc_ns",           # Sensor timing
    "yaw", "pitch", "roll",               # Attitude degrees
    "ax", "ay", "az",                     # Acceleration m/s² (az = downforce)
    "gx", "gy", "gz",                     # Gyro rad/s (gz = yaw rate)
    "dvx", "dvy", "dvz",                  # Delta velocity
    "dtx", "dty", "dtz", "dt",           # Delta theta + interval
    "mx", "my", "mz",                     # Magnetometer Gauss
    "temp_c", "pressure_pa",              # Temperature and pressure
    "gnss_fix", "gnss_sats",               # GNSS quality
    "gnss_lat", "gnss_lon", "gnss_alt",    # Raw GNSS position
    "gnss_vn", "gnss_ve", "gnss_vd",       # GNSS velocity
    "gnss_speed",                          # Ground speed m/s
    "ins_lat", "ins_lon", "ins_alt",    # INS fused position
    "ins_vn", "ins_ve", "ins_vd",       # INS fused velocity
]

# ── CONNECT ──────────────────────────────────────────────────────────
sensor = Sensor()  # Creates Sensor object
try:
    # autocconnect tries all standard baud rates automatically until it finds the VN-300
    sensor.autoConnect(PORT)
# contains specific error message explaining what went wrong (e.g. wrong port, no permission, etc.)
except Exception as e:
    # explaining what went wrong (e.g. wrong port, no permission, etc.)
    logger.error(f"Connection failed: {e}")
    sys.exit(1)  # stops the entire script

# confirms connection worked and prints baud rate
logger.info(f"Connected: {sensor.connectedBaudRate()} baud")
# creates an empty container for the model number data
model_register = Registers.System.Model()
# prints model number to confirm we're talking to the sensor(VN-300)
sensor.readRegister(model_register)
# confirms we are talking to the right device and prints the model number(VN-300T-CR)
logger.info(f"Sensor: {model_register.model}")

# BINARY OUTPUT 1 — IMU + ATTITUDE    1 MEANS INCLUDE IT IN THE PACKET, 0 MEANS DON'T INCLUDE IT
# Creates a configuration object for Binary Output 1
imu_output = Registers.System.BinaryOutput1()
# Tells the sensor to stream data out on the serial port 1(FTDI cable)
imu_output.asyncMode.serial1 = 1
# RATE is already defined as 40, which divides the internal 400Hz rate to get 10Hz output
imu_output.rateDivisor = RATE
imu_output.common.timeStartup = 1  # Time since boot in nanoseconds
imu_output.common.timeGps = 1      # GPS UTC time
# Yaw Pitch Roll in degrees: Orientation of the car in degrees
imu_output.common.ypr = 1
imu_output.common.accel = 1        # Accelerometer forces X/Y/Z m/s²
imu_output.common.angularRate = 1  # Gyroscope(rotation rates) X/Y/Z in rad/s
imu_output.common.imu = 1          # Full IMU packet
# Magnetometer (heading) and barometric pressure
imu_output.common.magPres = 1
# Delta velocity and delta theta (Change in velocities and angles between samples)
imu_output.common.deltas = 1
# Sends the completed config form to the physical sensor
sensor.writeRegister(imu_output)
logger.info("Binary Output 1 configured (IMU + Attitude)")

# BINARY OUTPUT 2 — GNSS
# Creates configuration object for the raw GPS data. The data straight from the satellite receiver
gnss_output = Registers.System.BinaryOutput2()
gnss_output.asyncMode.serial1 = 1  # Stream on serial port 1
gnss_output.rateDivisor = RATE  # same 10Hz rate
# Fix type: 3 = 3D fix(fix type 3 means full 3D fix, need this before driving)
gnss_output.gnss.gnss1Fix = 1
# Satellite count (Number of satellites in view, more is better for accuracy)
gnss_output.gnss.gnss1NumSats = 1
# Raw GPS position lat/lon/alt (Latitude, longitude, and altitude from the GPS)
gnss_output.gnss.gnss1PosLla = 1
# GPS velocity North/East/Down (Velocity in north/east/down coordinates from the GPS)
gnss_output.gnss.gnss1VelNed = 1
# Position accuracy estimate (how accurate the GPS position is in meters)
gnss_output.gnss.gnss1PosUncertainty = 1
# Sends the completed config form to the physical sensor
sensor.writeRegister(gnss_output)
logger.info("Binary Output 2 configured (GNSS)")

# ── BINARY OUTPUT 3 — INS FUSED (Inertial Navigation system) GPS + IMU COMBINED, more accurate than raw GPS
# Creates configuration object for the INS fused solution
ins_output = Registers.System.BinaryOutput3()
ins_output.asyncMode.serial1 = 1             # Stream on serial port 1
ins_output.rateDivisor = RATE                # same 10Hz rate
ins_output.ins.posLla = 1          # Fused lat/lon/altitude position — most accurate
# Fused velocity North/East/Down - smoother than raw GPS
ins_output.ins.velNed = 1
# Position uncertainty- How confident the INS is in meters
ins_output.ins.posU = 1
# Velocity uncertainty- How confident the INS is in m/s
ins_output.ins.velU = 1
# Sends the completed INS config form to the physical sensor
sensor.writeRegister(ins_output)
logger.info("Binary Output 3 configured (INS)")

# Opens CSV file for writing "w"= Write mode(creates news file), newline=prevents extra blank lines in Windows
f = open(FILENAME, "w", newline="")
# Dictwriter lets us write rows as dictionaires, fieldnames=COLS tells it the order of coloumns, ignore keys that are not in COLS
w = csv.DictWriter(f, fieldnames=COLS, extrasaction="ignore")
w.writeheader()  # Coloumns names as the first row in the CSV
# Confirms to the terminal that logging has started
logger.info(f"Logging → {FILENAME}")
logger.info("Ctrl+C to stop")

# MAIN LOGGING LOOP
last_lap_time = None
lap_count = 0
row_count = 0  # n is a row counter
try:
    while True:
        # Get next data packet (asks the sensor for the next avaiable data packet)
        measurement = sensor.getNextMeasurement()
        if not measurement:
            continue                # Skip if nothing ready

        # start a new row dictionary with current local time isoformat gives a clean readable timestamp
        r = {"timestamp": datetime.now().isoformat()}

        # Timing
        # check if the packet contains startup time before trying to read it
        if measurement.time.timeStartup is not None:
            # nanoseconds converts the time object to a plain number
            r["startup_ns"] = measurement.time.timeStartup.nanoseconds()
        # more accurate than system clock time probably useful for syncing with other car sensors
        if measurement.time.timeGps is not None:
            r["gps_utc_ns"] = measurement.time.timeGps.nanoseconds()

        # Attitude — yaw/pitch/roll
        if measurement.attitude.ypr is not None:  # check if the packet contains attitude data before trying to read it
            # Yaw= car's heading angle, 0-360 degrees where 0/360 is north, 90 is east, 180 is south, 270 is west
            r["yaw"] = measurement.attitude.ypr.yaw
            # pitch = nose up/down angle, positive is nose up, negative is nose down
            r["pitch"] = measurement.attitude.ypr.pitch
            # roll = lean left/right angle, positive is leaning right, negative is leaning left
            r["roll"] = measurement.attitude.ypr.roll

        # Accelerometer — az is vertical g-force for aero downforce analysis
        # accel is subscriptable - [o] is x, [1] is y, [2] is z
        # ax = forward/backward force, ay = left/right cornering force, az = vertical force (KEY for aero team, increases with downforce)
        if measurement.imu.accel is not None:
            r["ax"], r["ay"], r["az"] = measurement.imu.accel[0], measurement.imu.accel[1], measurement.imu.accel[2]

        # Gyroscope — gz is yaw rate, how fast car is turning
        # rotation rates ins rad/s on each axis, gz = yaw rate how fast the car is turning, higher means faster turn
        if measurement.imu.angularRate is not None:
            r["gx"], r["gy"], r["gz"] = measurement.imu.angularRate[0], measurement.imu.angularRate[1], measurement.imu.angularRate[2]

        # Delta theta — integrated rotation between samples
        if measurement.imu.deltaTheta is not None:
            # Change in rotation angle, used to reconstruct trajectory
            r["dtx"] = measurement.imu.deltaTheta.deltaTheta[0]
            r["dty"] = measurement.imu.deltaTheta.deltaTheta[1]
            r["dtz"] = measurement.imu.deltaTheta.deltaTheta[2]
            # dt = time interval between this sample and last in seconds
            r["dt"] = measurement.imu.deltaTheta.deltaTime

        # Delta velocity — integrated acceleration between samples
        if measurement.imu.deltaVel is not None:
            # change in velocity since last sample in m/s, integrated from accelrometer between samples
            r["dvx"], r["dvy"], r["dvz"] = measurement.imu.deltaVel[0], measurement.imu.deltaVel[1], measurement.imu.deltaVel[2]

        # Magnetometer — used for heading calibration
        # Magnetic feild strength in guass on each axis, used internally by VN-300 for heading
        if measurement.imu.mag is not None:
            r["mx"], r["my"], r["mz"] = measurement.imu.mag[0], measurement.imu.mag[1], measurement.imu.mag[2]

        # Temperature and pressure
        if measurement.imu.temperature is not None:  # Internal sensor temperature in Celciius
            r["temp_c"] = measurement.imu.temperature
        if measurement.imu.pressure is not None:  # air pressure in pascals - changes with altitude and weather
            r["pressure_pa"] = measurement.imu.pressure

        # Raw GNSS — parse position and velocity from string
        # fix type 3 = full  3D lock - car cannot move until this is 3, int() converts the object to a plain number
        if measurement.gnss.gnss1Fix is not None:
            r["gnss_fix"] = int(measurement.gnss.gnss1Fix)
        if measurement.gnss.gnss1NumSats is not None:  # number of satellites in view
            r["gnss_sats"] = measurement.gnss.gnss1NumSats
        # posLla has named attributes lat/lon/alt raw GNSS position from satellites
        if measurement.gnss.gnss1PosLla is not None:
            r["gnss_lat"] = measurement.gnss.gnss1PosLla.lat
            r["gnss_lon"] = measurement.gnss.gnss1PosLla.lon
            r["gnss_alt"] = measurement.gnss.gnss1PosLla.alt
        if measurement.gnss.gnss1VelNed is not None:  # raw GNSS velocity in North East Down in m/s
            r["gnss_vn"] = measurement.gnss.gnss1VelNed[0]  # northward
            r["gnss_ve"] = measurement.gnss.gnss1VelNed[1]  # eastward
            r["gnss_vd"] = measurement.gnss.gnss1VelNed[2]  # downward
            # ground speed = 3D magnitude of velocity vector using pythagoras
            # sqrt(vn² + ve² + vd²) gives total speed in m/s
            r["gnss_speed"] = (r["gnss_vn"]**2 + r["gnss_ve"] **
                               2 + r["gnss_vd"]**2)**0.5

        # INS fused solution — GPS + IMU combined, most accurate
        if measurement.ins.posLla is not None:
            # posLla has named attributes .lat .lon .alt, this is GPS + IMU combined, more accurate and smoother than raw gnss_lat/lon above
            r["ins_lat"] = measurement.ins.posLla.lat
            r["ins_lon"] = measurement.ins.posLla.lon
            r["ins_alt"] = measurement.ins.posLla.alt
        if measurement.ins.velNed is not None:
            # fused velocity North East Down in m/s, smoother than raw GNSS velocity because IMU fills the gaps
            r["ins_vn"] = measurement.ins.velNed[0]
            r["ins_ve"] = measurement.ins.velNed[1]
            r["ins_vd"] = measurement.ins.velNed[2]
            # LAP TIMER
        # runs every loop — checks if car is within 5m of start/finish line
        if r.get("ins_lat") is not None and r.get("ins_lon") is not None:
            dist = distance_meters(
                r["ins_lat"], r["ins_lon"], START_LAT, START_LON)
            now = datetime.now()
            if dist < THRESHOLD_METERS:
                if last_lap_time is None or (now - last_lap_time).total_seconds() > COOL_DOWN_SECONDS:
                    if last_lap_time is not None:
                        lap_time = (now - last_lap_time).total_seconds()
                        logger.info(
                            f"Lap {lap_count} time: {lap_time:.3f} seconds")
                    last_lap_time = now
                    lap_count += 1

        w.writerow(r)   # Write row to CSV
        if row_count % 50 == 0:
            f.flush()  # Flush every 50 rows to ensure data is written to disk regularly
        row_count += 1
except KeyboardInterrupt:
    logger.info(f"Stopped. {row_count} rows saved → {FILENAME}")
finally:
    f.close()
    sensor.disconnect()
    logger.info("Done.")
