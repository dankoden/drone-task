# Drone Task: STABILIZE + RC Override (A -> B)

This project runs an ArduPilot SITL mission script that:

- takes off in `STABILIZE`
- flies from point A to point B using **RC Override**
- holds target altitude during transit
- lands near point B and prints touchdown error

## Mission Target

- Point A: `50.450739, 30.461242`
- Point B: `50.443326, 30.448078`
- Target altitude: `100 m`

## Flight Accuracy

Landing accuracy is reported by the mission script as:

- `dist_to_B` in meters (distance from touchdown point to target point B)
- printed at the end of mission:
  - `Touchdown lat=... lon=... dist_to_B=...m`

Current practical target for this setup:

- desired: `<= 1.0 m`
- strong result: `<= 0.5 m`

Note: final error depends on wind model, SITL physics, and selected speed profile.

---

## Quick Start (macOS / Ubuntu)

### 1. Clone repository

HTTPS:

```bash
git clone https://github.com/dankoden/drone-task.git
cd drone-task
```

SSH:

```bash
git clone git@github.com:dankoden/drone-task.git
cd drone-task
```

### 2. Create virtual environment

```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Python dependencies

```bash
pip install --upgrade pip
pip install dronekit pymavlink MAVProxy matplotlib opencv-python
```

### 4. Start SITL (Terminal #1)

```bash
source venv/bin/activate
Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --map --console -l 50.450739,30.461242,584,0
```

Optional marker for final point B in MAVProxy console:

```text
map icon 50.443326 30.448078 redflag
```

### 5. Run mission script (Terminal #2)

```bash
cd drone-task
source venv/bin/activate
./run_drone.sh
```

Mission output is streamed and saved to:

- `./run_drone.log`

### 6. Stop mission

- Press `Ctrl+C` in mission terminal for emergency abort.

---

## What `run_drone.sh` Does

`run_drone.sh` starts:

- `Tools/autotest/stabilize_rc_override_mission.py`
- with default connect endpoint `udp:127.0.0.1:14550`
- and writes output via `tee` to `./run_drone.log`

You can override values:

```bash
CONNECT=udp:127.0.0.1:14550 \
ALT=100 \
ARRIVAL_RADIUS=3 \
SPEED_SCALE=4.0 \
./run_drone.sh
```

---

## Manual Run (without wrapper)

```bash
python Tools/autotest/stabilize_rc_override_mission.py \
  --connect udp:127.0.0.1:14550 \
  --lat-a 50.450739 --lon-a 30.461242 \
  --lat-b 50.443326 --lon-b 30.448078 \
  --alt 100 --arrival-radius 3 \
  --speed-scale 4.0 \
  2>&1 | tee ./run_drone.log
```

---

## Wind Parameters Applied by Script

The script sets and verifies:

- `SIM_WIND_SPD = 3`
- `SIM_WIND_DIR = 30`
- `SIM_WIND_TURB = 2`
- `SIM_WIND_TURB_FREQ = 0.2` (or fallback via `SIM_WIND_TC`)

---

## Main Files

- Mission script: `Tools/autotest/stabilize_rc_override_mission.py`
- Runner script: `run_drone.sh`
- Runtime log: `run_drone.log`

---

## Integration Note

This task is implemented inside an ArduPilot SITL workflow.

- ArduPilot repo: [https://github.com/ArduPilot/ardupilot](https://github.com/ArduPilot/ardupilot)
- ArduPilot site: [https://ardupilot.org](https://ardupilot.org)
