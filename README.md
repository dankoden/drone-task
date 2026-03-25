# Drone Task: STABILIZE + RC Override (A -> B)

This repository contains a custom ArduPilot SITL mission script for a test task:

- Takeoff in `STABILIZE`
- Fly from point **A** to point **B** using **RC Override**
- Hold target altitude during transit
- Land as close as possible to point **B**

## Integration With ArduPilot

This task is implemented directly inside an ArduPilot workspace and uses ArduPilot SITL + MAVProxy for simulation and visualization.

- ArduPilot official repository: [https://github.com/ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot)
- ArduPilot project website: [https://ardupilot.org](https://ardupilot.org)

## Mission Points

- Point A (start): `50.450739, 30.461242`
- Point B (target): `50.443326, 30.448078`
- Target altitude: `100 m`

## Main Files

- Mission script: `Tools/autotest/stabilize_rc_override_mission.py`
- Runner script (with `tee` log): `./run_drone.sh`
- Flight log output: `./run_drone.log`

## What The Script Does

- Connects to SITL (`udp:127.0.0.1:14550` by default, with endpoint fallback)
- Sets and verifies wind parameters:
  - `SIM_WIND_SPD = 3`
  - `SIM_WIND_DIR = 30`
  - `SIM_WIND_TURB = 2`
  - `SIM_WIND_TURB_FREQ = 0.2` (or fallback via `SIM_WIND_TC`)
- Arms in `STABILIZE`
- Performs RC-override takeoff
- Navigates A -> B with detailed telemetry logs
- Executes RC-override landing near B
- Prints touchdown coordinates and distance to B
- Supports emergency abort by `Ctrl+C` (override clear + disarm attempt)

## Environment Setup

### macOS

```bash
cd /path/to/ardupilot
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install dronekit pymavlink MAVProxy matplotlib opencv-python
```

### Ubuntu Linux

```bash
sudo apt update
sudo apt install -y git python3 python3-venv python3-pip

cd /path/to/ardupilot
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install dronekit pymavlink MAVProxy matplotlib opencv-python
```

## Start SITL

Run in terminal #1:

```bash
cd /path/to/ardupilot
source venv/bin/activate
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --map --console -l 50.450739,30.461242,584,0
```

Optional marker for final point B in MAVProxy console:

```text
map icon 50.443326 30.448078 redflag
```

## Run Mission (with tee log)

Run in terminal #2:

```bash
cd /path/to/ardupilot
source venv/bin/activate
./run_drone.sh
```

This writes runtime output to:

- `./run_drone.log`

## Manual Run

```bash
python Tools/autotest/stabilize_rc_override_mission.py \
  --connect udp:127.0.0.1:14550 \
  --lat-a 50.450739 --lon-a 30.461242 \
  --lat-b 50.443326 --lon-b 30.448078 \
  --alt 100 --arrival-radius 3 \
  --speed-scale 4.0 \
  2>&1 | tee ./run_drone.log
```

## Useful Parameters

- `--speed-scale` flight aggressiveness (`1.0`..`4.0`)
- `--arrival-radius` final approach radius in meters
- `--log-interval` telemetry print period in seconds
- `--connect` MAVLink endpoint

## Stop / Safety

- Press `Ctrl+C` to abort mission immediately.
