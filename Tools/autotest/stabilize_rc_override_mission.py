#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

"""
ArduCopter SITL demo:
- STABILIZE mode only
- RC override for takeoff, transit, and landing
- Wind parameters configured from script
"""

from __future__ import annotations

import argparse
import collections
import collections.abc
import math
import signal
import sys
import time
from dataclasses import dataclass

# DroneKit (2.9.x) still references collections.MutableMapping on Python 3.10+.
if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import VehicleMode, connect


EARTH_RADIUS_M = 6378137.0


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(value, hi))


def wrap_180(angle_deg: float) -> float:
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    return wrapped


def distance_and_bearing_m(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    mean_lat = math.radians((lat1 + lat2) * 0.5)
    north = dlat * EARTH_RADIUS_M
    east = dlon * EARTH_RADIUS_M * math.cos(mean_lat)
    distance = math.hypot(north, east)
    bearing = (math.degrees(math.atan2(east, north)) + 360.0) % 360.0
    return distance, bearing


def safe_rel_alt(vehicle) -> float:
    loc = vehicle.location.global_relative_frame
    if loc is None or loc.alt is None:
        return 0.0
    return float(loc.alt)


def safe_heading(vehicle) -> float:
    if vehicle.heading is None:
        return 0.0
    return float(vehicle.heading)


def safe_velocity_down(vehicle) -> float:
    vel = vehicle.velocity
    if vel is None or len(vel) < 3 or vel[2] is None:
        return 0.0
    return float(vel[2])


def safe_velocity_ne(vehicle) -> tuple[float, float]:
    vel = vehicle.velocity
    if vel is None or len(vel) < 2:
        return 0.0, 0.0
    vn = float(vel[0] or 0.0)
    ve = float(vel[1] or 0.0)
    return vn, ve


def earth_to_body(north: float, east: float, heading_deg: float) -> tuple[float, float]:
    yaw = math.radians(heading_deg)
    forward = north * math.cos(yaw) + east * math.sin(yaw)
    right = -north * math.sin(yaw) + east * math.cos(yaw)
    return forward, right


def offset_ne_m(lat_from: float, lon_from: float, lat_to: float, lon_to: float) -> tuple[float, float]:
    dlat = math.radians(lat_to - lat_from)
    dlon = math.radians(lon_to - lon_from)
    mean_lat = math.radians((lat_to + lat_from) * 0.5)
    north = dlat * EARTH_RADIUS_M
    east = dlon * EARTH_RADIUS_M * math.cos(mean_lat)
    return north, east


def set_mode(vehicle, mode_name: str, timeout_s: float = 10.0) -> None:
    vehicle.mode = VehicleMode(mode_name)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if vehicle.mode is not None and vehicle.mode.name == mode_name:
            return
        time.sleep(0.1)
    raise TimeoutError(f"Failed to switch mode to {mode_name}")


def wait_armable(vehicle, timeout_s: float = 60.0) -> None:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if vehicle.is_armable:
            return
        time.sleep(0.25)
    raise TimeoutError("Vehicle did not become armable")


def arm(vehicle, timeout_s: float = 20.0) -> None:
    vehicle.armed = True
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if vehicle.armed:
            return
        time.sleep(0.1)
    raise TimeoutError("Arming failed")


def emergency_abort(vehicle, hz: float = 8.0) -> None:
    """Best-effort emergency stop on Ctrl+C."""
    print("\n[ABORT] Ctrl+C received: stopping mission now...")
    dt = 1.0 / max(2.0, hz)
    try:
        rc = RCOverride(roll=1500, pitch=1500, throttle=1000, yaw=1500)
        for _ in range(3):
            rc.push(vehicle)
            time.sleep(dt)
    except Exception:
        pass

    try:
        RCOverride.clear(vehicle)
    except Exception:
        pass

    try:
        vehicle.armed = False
        t0 = time.time()
        while time.time() - t0 < 2.0 and vehicle.armed:
            time.sleep(0.1)
    except Exception:
        pass


@dataclass
class RCOverride:
    roll: int = 1500
    pitch: int = 1500
    throttle: int = 1000
    yaw: int = 1500

    def push(self, vehicle) -> None:
        vehicle.channels.overrides = {
            "1": int(clamp(self.roll, 1000, 2000)),
            "2": int(clamp(self.pitch, 1000, 2000)),
            "3": int(clamp(self.throttle, 1000, 2000)),
            "4": int(clamp(self.yaw, 1000, 2000)),
        }

    @staticmethod
    def clear(vehicle) -> None:
        vehicle.channels.overrides = {}


def throttle_from_alt_hold(
    target_alt: float,
    current_alt: float,
    vel_down: float,
    state: dict[str, float],
    dt: float,
    hover_pwm: float = 1470.0,
) -> int:
    # STABILIZE has no altitude hold; emulate it with PI + climb-rate damping.
    alt_error = target_alt - current_alt
    alt_i = state.get("alt_i", 0.0) + alt_error * dt
    alt_i = clamp(alt_i, -60.0, 60.0)
    state["alt_i"] = alt_i

    climb_rate_current = -vel_down
    climb_rate_target = clamp(0.45 * alt_error + 0.04 * alt_i, -1.4, 1.8)
    climb_rate_error = climb_rate_target - climb_rate_current

    pwm = hover_pwm + (6.5 * alt_error) + (42.0 * climb_rate_error) + (1.2 * alt_i)
    return int(clamp(pwm, 1200, 1720))


def _set_param_and_verify(
    vehicle,
    name: str,
    requested_value: float,
    tolerance: float = 1e-3,
    verify_timeout_s: float = 6.0,
) -> tuple[bool, str]:
    try:
        vehicle.parameters[name] = requested_value
    except Exception as ex:
        return False, f"write_error={ex}"

    t0 = time.time()
    last_value = None
    while time.time() - t0 < verify_timeout_s:
        try:
            current = vehicle.parameters.get(name)
        except Exception:
            current = None
        if current is not None:
            last_value = float(current)
            if abs(last_value - float(requested_value)) <= tolerance:
                return True, f"requested={requested_value} applied={last_value}"
        time.sleep(0.2)

    if last_value is None:
        return False, f"requested={requested_value} applied=<none>"
    return False, f"requested={requested_value} applied={last_value}"


def set_wind(vehicle) -> None:
    wind_params = {
        "SIM_WIND_SPD": 3,
        "SIM_WIND_DIR": 30,
        "SIM_WIND_TURB": 2,
    }
    available = set(vehicle.parameters)
    print("Wind setup verification:")

    setup_ok = True
    for name, value in wind_params.items():
        if name not in available:
            setup_ok = False
            print(f"  {name}: FAIL (parameter not available)")
            continue
        ok, details = _set_param_and_verify(vehicle, name, value)
        setup_ok = setup_ok and ok
        print(f"  {name}: {'OK' if ok else 'FAIL'} ({details})")

    # Some firmware trees don't have SIM_WIND_TURB_FREQ; use SIM_WIND_TC fallback.
    if "SIM_WIND_TURB_FREQ" in available:
        ok, details = _set_param_and_verify(vehicle, "SIM_WIND_TURB_FREQ", 0.2)
        setup_ok = setup_ok and ok
        print(f"  SIM_WIND_TURB_FREQ: {'OK' if ok else 'FAIL'} ({details})")
    elif "SIM_WIND_TC" in available:
        # 0.2Hz => 5s period; SIM_WIND_TC is a practical approximation here.
        ok, details = _set_param_and_verify(vehicle, "SIM_WIND_TC", 5.0)
        setup_ok = setup_ok and ok
        print("  SIM_WIND_TURB_FREQ: EMULATED via SIM_WIND_TC=5.0")
        print(f"  SIM_WIND_TC: {'OK' if ok else 'FAIL'} ({details})")
    else:
        setup_ok = False
        print("  SIM_WIND_TURB_FREQ: FAIL (SIM_WIND_TURB_FREQ and SIM_WIND_TC are unavailable)")

    if setup_ok:
        print("Wind setup result: SUCCESS")
    else:
        print("Wind setup result: PARTIAL/FAILED")
    time.sleep(1.0)


def _sample_body_velocity(
    vehicle,
    rc: RCOverride,
    target_alt_m: float,
    hz: float,
    duration_s: float,
    alt_state: dict[str, float],
    hover_pwm: float,
) -> tuple[float, float]:
    dt = 1.0 / hz
    end_t = time.time() + duration_s
    sum_fwd = 0.0
    sum_right = 0.0
    count = 0
    while time.time() < end_t:
        alt = safe_rel_alt(vehicle)
        vel_down = safe_velocity_down(vehicle)
        rc.throttle = throttle_from_alt_hold(target_alt_m, alt, vel_down, alt_state, dt, hover_pwm=hover_pwm)
        rc.push(vehicle)
        vn, ve = safe_velocity_ne(vehicle)
        fwd, right = earth_to_body(vn, ve, safe_heading(vehicle))
        sum_fwd += fwd
        sum_right += right
        count += 1
        time.sleep(dt)
    if count == 0:
        return 0.0, 0.0
    return sum_fwd / count, sum_right / count


def calibrate_rc_axis_signs(
    vehicle,
    rc: RCOverride,
    target_alt_m: float,
    hz: float,
    default_pitch_sign: float,
    alt_state: dict[str, float],
    hover_pwm: float,
) -> tuple[float, float]:
    print("Calibrating RC axis signs (pitch/roll)...")
    rc.roll = 1500
    rc.pitch = 1500
    rc.yaw = 1500
    base_fwd, base_right = _sample_body_velocity(vehicle, rc, target_alt_m, hz, 1.5, alt_state, hover_pwm)

    pulse = 110

    rc.pitch = 1500 + pulse
    pulse_fwd, _ = _sample_body_velocity(vehicle, rc, target_alt_m, hz, 1.8, alt_state, hover_pwm)
    rc.pitch = 1500
    _sample_body_velocity(vehicle, rc, target_alt_m, hz, 0.9, alt_state, hover_pwm)
    delta_fwd = pulse_fwd - base_fwd

    rc.roll = 1500 + pulse
    _, pulse_right = _sample_body_velocity(vehicle, rc, target_alt_m, hz, 1.8, alt_state, hover_pwm)
    rc.roll = 1500
    _sample_body_velocity(vehicle, rc, target_alt_m, hz, 0.9, alt_state, hover_pwm)
    delta_right = pulse_right - base_right

    pitch_sign = 1.0 if delta_fwd > 0.0 else -1.0
    roll_sign = 1.0 if delta_right > 0.0 else -1.0

    # If response is too small/noisy, fallback to expected defaults.
    if abs(delta_fwd) < 0.15:
        pitch_sign = default_pitch_sign
    if abs(delta_right) < 0.15:
        roll_sign = 1.0

    print(
        f"Axis calibration: dFwd={delta_fwd:+.2f} dRight={delta_right:+.2f} "
        f"=> pitch_sign={'+' if pitch_sign > 0 else '-'} roll_sign={'+' if roll_sign > 0 else '-'}"
    )
    return pitch_sign, roll_sign


def compute_iaf(
    target_lat: float,
    target_lon: float,
    wind_from_deg: float,
    iaf_dist_m: float,
) -> tuple[float, float]:
    """Розрахувати точку IAF (Initial Approach Fix) навпроти вітру від цілі.

    IAF розміщується у напрямку звідки дує вітер (wind_from_deg) на відстані
    iaf_dist_m від цілі. Фінальний захід виконується проти вітру, що дозволяє
    дрону природньо гальмувати під час підльоту до точки посадки.
    """
    bearing_rad = math.radians(wind_from_deg)
    dlat = (iaf_dist_m * math.cos(bearing_rad)) / EARTH_RADIUS_M
    dlon = (iaf_dist_m * math.sin(bearing_rad)) / (
        EARTH_RADIUS_M * math.cos(math.radians(target_lat))
    )
    return target_lat + math.degrees(dlat), target_lon + math.degrees(dlon)


def fly_stabilize_rc(
    vehicle,
    start_lat: float,
    start_lon: float,
    target_lat: float,
    target_lon: float,
    target_alt_m: float,
    arrival_radius_m: float,
    pitch_forward_pwm: int,
    hz: float,
    log_interval_s: float,
    auto_pitch_flip: bool,
    a_hold_max_sec: float,
    wind_from_deg: float = 30.0,
    iaf_dist_m: float = 100.0,
    speed_scale: float = 1.0,
) -> None:
    rc = RCOverride()
    dt = 1.0 / hz
    speed_scale = clamp(speed_scale, 0.8, 4.0)
    nav_speed_boost = 8.0 if speed_scale >= 3.0 else (3.0 if speed_scale >= 2.0 else 1.0)
    tkof_speed_boost = 3.0 if speed_scale >= 2.0 else 1.0
    landing_rate_boost = clamp(speed_scale, 1.0, 3.0)
    land_speed_scale = clamp(1.0 + 0.65 * (speed_scale - 1.0), 1.0, 2.4)
    descent_rate_scale = clamp(1.0 + 0.55 * (speed_scale - 1.0), 1.0, 2.3)
    nav_pitch_delta = int(
        clamp(180.0 + 80.0 * (speed_scale - 1.0) + 90.0 * (nav_speed_boost - 1.0), 180.0, 700.0)
    )
    land_pitch_delta = int(clamp(170.0 + 45.0 * (speed_scale - 1.0), 170.0, 280.0))
    nav_pitch_min = int(clamp(1500 - nav_pitch_delta, 1000, 1500))
    nav_pitch_max = int(clamp(1500 + nav_pitch_delta, 1500, 2000))
    land_pitch_min = int(clamp(1500 - land_pitch_delta, 1200, 1500))
    land_pitch_max = int(clamp(1500 + land_pitch_delta, 1500, 1800))
    takeoff_min_thr = int(clamp(1600.0 + 200.0 * (speed_scale - 1.0) * tkof_speed_boost, 1600.0, 2000.0))
    control_alt_m = target_alt_m
    fast_tkof_cutover_alt = control_alt_m * (0.98 if speed_scale >= 2.0 else 1.0)
    nav_hover_pwm = 1470.0
    alt_state = {"alt_i": 0.0}

    print("Waiting for vehicle to become armable...")
    wait_armable(vehicle)

    print("Setting wind parameters...")
    set_wind(vehicle)

    print("Switching to STABILIZE and arming...")
    set_mode(vehicle, "STABILIZE")
    rc.throttle = 1000
    rc.push(vehicle)
    time.sleep(0.5)
    arm(vehicle)

    # Розрахувати IAF — точка заходу проти вітру
    iaf_lat, iaf_lon = compute_iaf(target_lat, target_lon, wind_from_deg, iaf_dist_m)
    iaf_dist_from_b, _ = distance_and_bearing_m(iaf_lat, iaf_lon, target_lat, target_lon)
    print(
        f"Start A:  lat={start_lat:.6f} lon={start_lon:.6f} | "
        f"IAF:      lat={iaf_lat:.6f} lon={iaf_lon:.6f} "
        f"(dist_to_B={iaf_dist_from_b:.0f}m, wind_from={wind_from_deg:.0f}deg) | "
        f"Target B: lat={target_lat:.6f} lon={target_lon:.6f} alt={target_alt_m:.1f}m "
        f"(control_alt={control_alt_m:.1f}m)"
    )
    print(
        f"Speed profile: scale={speed_scale:.2f} "
        f"(nav_boost={nav_speed_boost:.1f}x, tkof_boost={tkof_speed_boost:.1f}x, "
        f"takeoff_min_thr={takeoff_min_thr}, nav_pitch_limit={nav_pitch_delta}, "
        f"land_pitch_limit={land_pitch_delta}, land_rate_boost={landing_rate_boost:.1f}x)"
    )
    if speed_scale >= 3.0:
        print(f"Fast TKOF enabled: phase cutover at {fast_tkof_cutover_alt:.1f}m, climb-to-100 continues in NAV.")

    if speed_scale >= 3.0:
        if "ANGLE_MAX" in set(vehicle.parameters):
            ok, details = _set_param_and_verify(vehicle, "ANGLE_MAX", 12000)
            print(f"High-speed setup ANGLE_MAX: {'OK' if ok else 'FAIL'} ({details})")
        else:
            print("High-speed setup ANGLE_MAX: SKIP (parameter not available)")

    print(f"Takeoff to {control_alt_m:.1f} m (relative control target)...")
    t0 = time.time()
    next_tkof_log = t0
    while True:
        alt = safe_rel_alt(vehicle)
        if alt >= fast_tkof_cutover_alt:
            break
        if time.time() - t0 > 180:
            raise TimeoutError("Takeoff timeout")

        vel_down = safe_velocity_down(vehicle)
        rc.roll = 1500
        rc.pitch = 1500
        rc.yaw = 1500
        alt_err = max(0.0, control_alt_m - alt)
        if speed_scale >= 2.0:
            # Aggressive takeoff profile for very fast climb to target altitude.
            vz_up = max(0.0, -vel_down)
            if alt < (control_alt_m - 20.0):
                tkof_floor = 2000
            elif alt < (control_alt_m - 8.0):
                tkof_floor = 1920
            elif alt < (control_alt_m - 3.0):
                tkof_floor = 1840
            else:
                tkof_floor = 1720
            vz_target = 24.0 if speed_scale >= 3.0 else 18.0
            vz_boost = int(clamp((vz_target - vz_up) * 18.0, -80.0, 120.0))
            tkof_floor = int(clamp(tkof_floor + vz_boost, 1650.0, 2000.0))
        else:
            tkof_floor = takeoff_min_thr
            if alt_err < 15.0:
                tkof_floor = int(clamp(1500.0 + 26.0 * alt_err, 1500.0, takeoff_min_thr))
            if alt_err < 5.0:
                tkof_floor = int(clamp(1460.0 + 34.0 * alt_err, 1460.0, takeoff_min_thr))
        rc.throttle = max(
            tkof_floor,
            throttle_from_alt_hold(control_alt_m, alt, vel_down, alt_state, dt, hover_pwm=nav_hover_pwm),
        )
        rc.push(vehicle)
        now = time.time()
        if now >= next_tkof_log:
            print(f"[TKOF] t={now - t0:6.1f}s alt={alt:6.1f}m vz={-vel_down:5.2f}m/s thr={rc.throttle:4d}")
            next_tkof_log = now + log_interval_s
        time.sleep(dt)
    print()

    pitch_default_sign = -1.0 if pitch_forward_pwm < 1500 else 1.0
    pitch_axis_sign, roll_axis_sign = calibrate_rc_axis_signs(
        vehicle, rc, control_alt_m, hz, pitch_default_sign, alt_state, nav_hover_pwm
    )

    if a_hold_max_sec > 0:
        print(f"Quick re-center over A (max {a_hold_max_sec:.1f}s) ...")
        t_center = time.time()
        next_center_log = t_center
        settle_counter = 0
        while True:
            if time.time() - t_center > a_hold_max_sec:
                print("A-HOLD max time reached; starting A->B transit now.")
                break

            loc = vehicle.location.global_relative_frame
            if loc is None or loc.lat is None or loc.lon is None:
                time.sleep(dt)
                continue

            cur_lat = float(loc.lat)
            cur_lon = float(loc.lon)
            dist_a, bearing_a = distance_and_bearing_m(cur_lat, cur_lon, start_lat, start_lon)
            hdg = safe_heading(vehicle)
            yaw_err = wrap_180(bearing_a - hdg)

            vn, ve = safe_velocity_ne(vehicle)
            gs = math.hypot(vn, ve)
            err_n, err_e = offset_ne_m(cur_lat, cur_lon, start_lat, start_lon)
            err_fwd, err_right = earth_to_body(err_n, err_e, hdg)
            vel_fwd, vel_right = earth_to_body(vn, ve, hdg)

            hold_scale = min(speed_scale, 2.4)
            k_pos, vmax, k_vel = 0.08, 3.2 * hold_scale, 42.0 * (0.85 + 0.35 * hold_scale)
            cmd_fwd = clamp((clamp(k_pos * err_fwd, -vmax, vmax) - vel_fwd) * k_vel, -120.0, 120.0)
            cmd_right = clamp((clamp(k_pos * err_right, -vmax, vmax) - vel_right) * k_vel, -120.0, 120.0)

            rc.pitch = int(clamp(1500 + pitch_axis_sign * cmd_fwd, 1320, 1680))
            rc.roll = int(clamp(1500 + roll_axis_sign * cmd_right, 1320, 1680))
            rc.yaw = int(clamp(1500 + yaw_err * 4.5, 1260, 1740))
            alt = safe_rel_alt(vehicle)
            vel_down = safe_velocity_down(vehicle)
            rc.throttle = throttle_from_alt_hold(control_alt_m, alt, vel_down, alt_state, dt, hover_pwm=nav_hover_pwm)
            rc.push(vehicle)

            if dist_a < 6.0 and gs < 1.5:
                settle_counter += 1
            else:
                settle_counter = max(0, settle_counter - 1)
            if settle_counter > int(2.0 * hz):
                print(f"Centered over A: dist={dist_a:.2f}m gs={gs:.2f}m/s")
                break

            now = time.time()
            if now >= next_center_log:
                print(
                    f"[A-HOLD] t={now - t_center:6.1f}s distA={dist_a:6.2f}m alt={alt:6.2f}m gs={gs:4.2f} "
                    f"lat={cur_lat:.6f} lon={cur_lon:.6f}"
                )
                next_center_log = now + log_interval_s
            time.sleep(dt)
        print()
    else:
        print("A-HOLD skipped (fast mode). Starting A->B transit immediately.")
        print()

    # Маршрут: A -> IAF -> B
    # IAF дозволяє підлетіти до точки Б проти вітру, що забезпечує природнє гальмування
    waypoints = [(iaf_lat, iaf_lon), (target_lat, target_lon)]
    wp_names = ["IAF", "B"]
    final_leg_radius = max(arrival_radius_m, 20.0 if speed_scale >= 3.0 else 80.0)
    wp_radii = [max(arrival_radius_m, 30.0), final_leg_radius]

    hover_thr_samples = []
    path_distance_m = 0.0
    nav_min_los_speed_mps = 30.0 if speed_scale >= 3.0 else (20.0 if speed_scale >= 2.0 else 0.0)
    start_to_final_m, _ = distance_and_bearing_m(start_lat, start_lon, target_lat, target_lon)

    for wp_idx, ((wp_lat, wp_lon), wp_name, landing_start_radius_m) in enumerate(
        zip(waypoints, wp_names, wp_radii)
    ):
        is_final = wp_idx == len(waypoints) - 1
        print(f"\nLeg {wp_idx+1}/{len(waypoints)}: -> {wp_name} "
              f"lat={wp_lat:.6f} lon={wp_lon:.6f} radius={landing_start_radius_m:.0f}m")
        t_nav = time.time()
        next_nav_log = t_nav
        start_lat_leg = None
        start_lon_leg = None
        prev_lat = None
        prev_lon = None
        start_to_wp_m = None
        best_dist_m = 1.0e9
        drift_counter = 0
        flip_count = 0
        last_flip_time = t_nav
        prev_dist_m = None
        moving_away_counter = 0
        stall_counter = 0
        stall_recovery_count = 0

        while True:
            loc = vehicle.location.global_relative_frame
            if loc is None or loc.lat is None or loc.lon is None:
                time.sleep(dt)
                continue
    
            cur_lat = float(loc.lat)
            cur_lon = float(loc.lon)
    
            if start_lat_leg is None:
                start_lat_leg = cur_lat
                start_lon_leg = cur_lon
                start_to_wp_m, _ = distance_and_bearing_m(start_lat_leg, start_lon_leg, wp_lat, wp_lon)
                prev_lat = cur_lat
                prev_lon = cur_lon
    
            segment_m, _ = distance_and_bearing_m(prev_lat, prev_lon, cur_lat, cur_lon)
            path_distance_m += segment_m
            prev_lat = cur_lat
            prev_lon = cur_lon
    
            dist_m, bearing_deg = distance_and_bearing_m(cur_lat, cur_lon, wp_lat, wp_lon)
            if dist_m <= landing_start_radius_m:
                print(f"\nReached waypoint {wp_name} radius: {dist_m:.2f} m")
                break
            if time.time() - t_nav > 900:
                raise TimeoutError("Navigation timeout")
    
            alt = safe_rel_alt(vehicle)
            vel_down = safe_velocity_down(vehicle)
            hdg = safe_heading(vehicle)
            yaw_err = wrap_180(bearing_deg - hdg)
            vx, vy = safe_velocity_ne(vehicle)
            ground_speed = math.hypot(vx, vy)
            los_n = math.cos(math.radians(bearing_deg))
            los_e = math.sin(math.radians(bearing_deg))
            los_speed = vx * los_n + vy * los_e
            progress_ratio = 0.0
            if start_to_wp_m and start_to_wp_m > 0.5:
                progress_ratio = clamp(1.0 - dist_m / start_to_wp_m, 0.0, 1.0)
            dist_to_final_m, _ = distance_and_bearing_m(cur_lat, cur_lon, target_lat, target_lon)
            progress_to_final = 0.0
            if start_to_final_m > 0.5:
                progress_to_final = clamp(1.0 - dist_to_final_m / start_to_final_m, 0.0, 1.0)
    
            if speed_scale >= 2.0:
                if dist_m > 300:
                    desired_los_speed = 34.0
                elif dist_m > 120:
                    desired_los_speed = 30.0
                elif dist_m > 60:
                    desired_los_speed = 24.0
                elif dist_m > 20:
                    desired_los_speed = 16.0
                else:
                    desired_los_speed = 8.0
            else:
                if dist_m > 300:
                    desired_los_speed = 10.0 * speed_scale * nav_speed_boost
                elif dist_m > 120:
                    desired_los_speed = 7.5 * speed_scale * nav_speed_boost
                elif dist_m > 60:
                    desired_los_speed = 5.0 * speed_scale * nav_speed_boost
                elif dist_m > 20:
                    desired_los_speed = 3.0 * speed_scale * nav_speed_boost
                else:
                    desired_los_speed = 1.6 * speed_scale * nav_speed_boost
            if abs(yaw_err) > 75.0:
                desired_los_speed = 0.0
            if dist_m > 120.0 and abs(yaw_err) < 20.0 and nav_min_los_speed_mps > 0.0:
                desired_los_speed = max(desired_los_speed, nav_min_los_speed_mps)
            # Fast route profile: first 75% fly max speed, then decelerate for landing prep.
            if abs(yaw_err) < 25.0:
                if progress_ratio < 0.75:
                    desired_los_speed = max(desired_los_speed, 35.0 if speed_scale >= 2.0 else 30.0)
                elif progress_ratio < 0.90:
                    desired_los_speed = max(desired_los_speed, 22.0 if speed_scale >= 2.0 else 18.0)
                else:
                    desired_los_speed = max(desired_los_speed, 10.0)
            # Global turbo cruise: keep very high speed until 75% of full A->B route.
            if speed_scale >= 2.0 and progress_to_final < 0.75 and abs(yaw_err) < 30.0:
                desired_los_speed = max(desired_los_speed, 55.0 if speed_scale >= 3.0 else 40.0)

            pitch_gain = 26.0 * (0.85 + 0.55 * speed_scale) * (1.0 + 0.45 * (nav_speed_boost - 1.0))
            pitch_effort = clamp(pitch_gain * (desired_los_speed - los_speed), -900.0, 900.0)
            if dist_m < 25.0:
                pitch_effort = clamp(pitch_effort, -170.0, 170.0)
    
            yaw_gain = 4.5 if dist_m > 80.0 else 6.0
            yaw_pwm = 1500 + int(clamp(yaw_err * yaw_gain, -260, 260))
            pitch_pwm = 1500 + int(clamp(pitch_axis_sign * pitch_effort, -nav_pitch_delta, nav_pitch_delta))
            if speed_scale >= 3.0 and progress_to_final < 0.75 and abs(yaw_err) < 30.0:
                # Turbo cruise: hard pitch stick for max translational speed.
                pitch_pwm = 1000 if pitch_axis_sign < 0 else 2000
            elif speed_scale >= 2.0 and progress_to_final < 0.75 and abs(yaw_err) < 30.0:
                # Semi-turbo cruise for x2 profile.
                pitch_pwm = 1060 if pitch_axis_sign < 0 else 1940
    
            rc.roll = 1500
            rc.pitch = int(clamp(pitch_pwm, nav_pitch_min, nav_pitch_max))
            rc.yaw = int(clamp(yaw_pwm, 1240, 1760))
            nav_thr_base = throttle_from_alt_hold(control_alt_m, alt, vel_down, alt_state, dt, hover_pwm=nav_hover_pwm)
            # Keep altitude independent from speed command.
            # If altitude departs too much, apply explicit correction.
            alt_err_nav = control_alt_m - alt
            nav_alt_corr = clamp(32.0 * alt_err_nav, -280.0, 360.0)
            nav_thr_max = 2000 if (speed_scale >= 2.0 and progress_to_final < 0.75) else (1900 if speed_scale >= 2.0 else 1780)
            nav_thr_min = 1160 if speed_scale >= 2.0 else 1200
            nav_thr_cmd = nav_thr_base + nav_alt_corr
            # Fast climb continuation after early TKOF cutover.
            if speed_scale >= 2.0 and alt < (control_alt_m - 1.5):
                climb_floor = clamp(1720.0 + 8.0 * (control_alt_m - alt), 1720.0, 1900.0)
                nav_thr_cmd = max(nav_thr_cmd, climb_floor)
            rc.throttle = int(clamp(nav_thr_cmd, nav_thr_min, nav_thr_max))
            hover_thr_samples.append(rc.throttle)
            if len(hover_thr_samples) > int(12 * hz):
                hover_thr_samples.pop(0)
            rc.push(vehicle)
    
            # Auto-detect wrong pitch direction:
            # if we are generally pointed at B but distance keeps increasing, flip pitch sign.
            if dist_m < best_dist_m:
                best_dist_m = dist_m
            if (
                auto_pitch_flip
                and dist_m > best_dist_m + 4.0
                and abs(yaw_err) < 20.0
                and ground_speed > 1.0
                and abs(pitch_effort) > 45.0
            ):
                drift_counter += 1
            else:
                drift_counter = max(0, drift_counter - 1)
    
            if prev_dist_m is not None:
                if dist_m > prev_dist_m + 0.08 and abs(yaw_err) < 25.0 and dist_m < 220.0:
                    moving_away_counter += 1
                else:
                    moving_away_counter = max(0, moving_away_counter - 1)
    
                # Stall detector: very low convergence rate while still far from B.
                closing_rate = (prev_dist_m - dist_m) / dt
                # Stall: дуже повільний рух АБО відстань не зменшується швидко
                if dist_m < 150.0 and abs(yaw_err) < 25.0 and (
                    closing_rate < 0.1 or (dist_m > 15.0 and ground_speed < 0.5)
                ):
                    stall_counter += 1
                else:
                    stall_counter = max(0, stall_counter - 1)
            prev_dist_m = dist_m
    
            if (
                auto_pitch_flip
                and (drift_counter > int(7 * hz) or moving_away_counter > int(2.5 * hz))
                and flip_count < 4
                and (time.time() - last_flip_time) > 8.0
            ):
                pitch_axis_sign *= -1.0
                flip_count += 1
                drift_counter = 0
                moving_away_counter = 0
                last_flip_time = time.time()
                best_dist_m = dist_m
                print(
                    f"\n[NAV] Auto pitch direction flip #{flip_count}. "
                    f"Now pitch_sign={'+' if pitch_axis_sign > 0 else '-'} "
                    f"(trend-based correction)"
                )
    
            # If we are stuck in NAV, force recovery:
            # 1) try pitch sign flip once/twice
            # 2) if still stuck, switch to landing phase
            if stall_counter > int(5 * hz):
                should_flip = (
                    auto_pitch_flip
                    and stall_recovery_count < 2
                    and (time.time() - last_flip_time) > 6.0
                    and (los_speed < -0.15 or moving_away_counter > int(1.5 * hz))
                )
                if should_flip:
                    pitch_axis_sign *= -1.0
                    stall_recovery_count += 1
                    stall_counter = 0
                    last_flip_time = time.time()
                    best_dist_m = dist_m
                    print(
                        f"\n[NAV] Stall recovery flip #{stall_recovery_count}: "
                        f"pitch_sign={'+' if pitch_axis_sign > 0 else '-'} at remain={dist_m:.1f}m"
                    )
                else:
                    print(
                        f"\n[NAV] Stall detected at {dist_m:.1f}m "
                        f"(los={los_speed:.2f}m/s, yaw_err={yaw_err:.1f}deg); forcing landing phase"
                    )
                    break
    
            now = time.time()
            if now >= next_nav_log:
                elapsed = now - t_nav
                direct_flown_m = 0.0
                if start_lat_leg is not None and start_lon_leg is not None:
                    direct_flown_m, _ = distance_and_bearing_m(start_lat_leg, start_lon_leg, cur_lat, cur_lon)
                progress_pct = 0.0
                if start_to_wp_m and start_to_wp_m > 0.5:
                    progress_pct = clamp((1.0 - dist_m / start_to_wp_m) * 100.0, 0.0, 100.0)
    
                print(
                    f"[NAV] t={elapsed:6.1f}s flown_path={path_distance_m:7.1f}m "
                    f"flown_direct={direct_flown_m:7.1f}m alt={alt:6.1f}m gs={ground_speed:5.2f}m/s "
                    f"remain={dist_m:7.1f}m progress={progress_pct:5.1f}% "
                    f"yaw_err={yaw_err:6.1f} los={los_speed:5.2f}/{desired_los_speed:4.1f} "
                    f"roll={rc.roll:4d} pitch={rc.pitch:4d} "
                    f"thr={rc.throttle:4d} lat={cur_lat:.6f} lon={cur_lon:.6f} "
                    f"psgn={'+' if pitch_axis_sign > 0 else '-'}"
                )
                next_nav_log = now + log_interval_s
            time.sleep(dt)
        print()
    # кінець leg

    hover_thr_est = int(sum(hover_thr_samples) / len(hover_thr_samples)) if hover_thr_samples else 1470
    print("Landing at B in STABILIZE with RC override...")
    print(f"Landing controller: hover_thr_est={hover_thr_est}")
    t_land = time.time()
    next_land_log = t_land
    land_alt_target = min(control_alt_m, safe_rel_alt(vehicle))
    landing_phase = "APPROACH"
    pitch_resp_bad = 0
    sign_flip_count = 0
    last_sign_flip_t = t_land
    touchdown_counter = 0
    zero_alt_counter = 0
    ground_contact_counter = 0
    deep_ground_counter = 0
    center_lock_counter = 0
    near_center_counter = 0
    descent_entry_alt_m = 15.0
    approach_descent_rate_mps = 3.0 * landing_rate_boost
    while True:
        alt = safe_rel_alt(vehicle)
        vx, vy = safe_velocity_ne(vehicle)
        ground_speed = math.hypot(vx, vy)
        hold_for_center = False

        if time.time() - t_land > 240:
            raise TimeoutError("Landing timeout")

        loc = vehicle.location.global_relative_frame
        if loc is None or loc.lat is None or loc.lon is None:
            time.sleep(dt)
            continue

        dist_m, bearing_deg = distance_and_bearing_m(float(loc.lat), float(loc.lon), target_lat, target_lon)
        hdg = safe_heading(vehicle)
        yaw_err = wrap_180(bearing_deg - hdg)
        los_n = math.cos(math.radians(bearing_deg))
        los_e = math.sin(math.radians(bearing_deg))
        los_speed = vx * los_n + vy * los_e
        err_n, err_e = offset_ne_m(float(loc.lat), float(loc.lon), target_lat, target_lon)
        err_fwd, err_right = earth_to_body(err_n, err_e, hdg)
        vel_fwd, vel_right = earth_to_body(vx, vy, hdg)
        vel_down = safe_velocity_down(vehicle)

        center_ready = dist_m <= 0.2 and ground_speed <= 0.18
        if center_ready:
            center_lock_counter += 1
        else:
            center_lock_counter = max(0, center_lock_counter - 1)

        # Touchdown only when we are low, slow and truly centered over B.
        if alt <= 0.12 and center_lock_counter >= int(0.6 * hz):
            touchdown_counter += 1
        else:
            touchdown_counter = 0
        if touchdown_counter >= int(2.0 * hz):
            break

        # Hard touchdown criterion: once relative altitude crosses zero near B,
        # finish immediately (prevents hovering/rebound around ground level).
        if alt <= 0.0 and dist_m <= 1.5:
            zero_alt_counter += 1
        else:
            zero_alt_counter = 0
        if zero_alt_counter >= int(0.25 * hz):
            print("[LAND] Touchdown threshold reached (alt<=0 near B).")
            break

        # Fallback #1: likely touchdown even if EKF relative altitude drifts.
        likely_ground_contact = (
            alt <= 0.35
            and dist_m <= 1.20
            and ground_speed <= 0.25
            and abs(vel_down) <= 0.55
        )
        if likely_ground_contact:
            ground_contact_counter += 1
        else:
            ground_contact_counter = 0
        if ground_contact_counter >= int(0.8 * hz):
            print("[LAND] Ground-contact fallback triggered (low/slow/near B).")
            break

        # Fallback #2: clearly below terrain and stationary -> force finish.
        deep_below_ground = (
            alt <= -1.5
            and dist_m <= 2.0
            and ground_speed <= 0.10
            and abs(vel_down) <= 0.10
        )
        if deep_below_ground:
            deep_ground_counter += 1
        else:
            deep_ground_counter = 0
        if deep_ground_counter >= int(0.6 * hz):
            print("[LAND] Ground-contact fallback triggered (deep below terrain, stationary).")
            break

        if alt <= -0.8 and ground_speed < 0.16 and dist_m <= 1.2:
            print("[LAND] Ground-contact fallback triggered (below terrain level).")
            break

        if dist_m <= 6.0 and ground_speed <= 1.8 and abs(yaw_err) < 25.0:
            near_center_counter += 1
        else:
            near_center_counter = max(0, near_center_counter - 1)

        if landing_phase == "APPROACH":
            if dist_m > 80.0:
                desired_los_speed = 5.0 * land_speed_scale
                approach_alt_cmd = control_alt_m
            elif dist_m > 45.0:
                desired_los_speed = 4.0 * land_speed_scale
                approach_alt_cmd = min(control_alt_m, 65.0)
            elif dist_m > 25.0:
                desired_los_speed = 3.0 * land_speed_scale
                approach_alt_cmd = 40.0
            elif dist_m > 12.0:
                desired_los_speed = 2.0 * land_speed_scale
                approach_alt_cmd = 22.0
            elif dist_m > 6.0:
                desired_los_speed = 1.4 * land_speed_scale
                approach_alt_cmd = descent_entry_alt_m
            else:
                desired_los_speed = 1.0 * land_speed_scale
                approach_alt_cmd = 8.0

            if land_alt_target > approach_alt_cmd:
                land_alt_target = max(approach_alt_cmd, land_alt_target - approach_descent_rate_mps * dt)
            else:
                land_alt_target = min(approach_alt_cmd, land_alt_target + 0.25 * dt)

            if (
                dist_m <= 6.0
                and ground_speed <= 1.6
                and abs(yaw_err) < 25.0
                and (alt <= (descent_entry_alt_m + 1.5) or near_center_counter > int(6.0 * hz))
            ):
                landing_phase = "DESCENT"
            elif dist_m <= 12.0 and alt <= (descent_entry_alt_m - 1.0) and ground_speed <= 0.9:
                # Якщо зависли недалеко від B на низькій висоті — починаємо DESCENT без зайвих маневрів.
                landing_phase = "DESCENT"
        else:
            if dist_m > 10.0:
                landing_phase = "APPROACH"
            if dist_m > 3.0:
                desired_los_speed = 0.9 * land_speed_scale
            else:
                desired_los_speed = 0.35

        if abs(yaw_err) > 35.0:
            desired_los_speed = 0.0

        land_pitch_gain = 34.0 * (0.90 + 0.35 * land_speed_scale)
        pitch_effort = clamp(land_pitch_gain * (desired_los_speed - los_speed), -220.0, 220.0)
        precision_zone = dist_m <= 20.0
        if precision_zone:
            if dist_m > 8.0:
                pos_vmax = 3.8
            elif dist_m > 3.0:
                pos_vmax = 2.2
            elif dist_m > 1.2:
                pos_vmax = 1.6
            elif dist_m > 0.5:
                pos_vmax = 1.0
            elif dist_m > 0.25:
                pos_vmax = 0.55
            else:
                pos_vmax = 0.25
            if dist_m > 3.0:
                pos_gain = 0.85
            elif dist_m > 0.5:
                pos_gain = 1.20
            elif dist_m > 0.25:
                pos_gain = 1.50
            else:
                pos_gain = 1.80
            fwd_v_tgt = clamp(pos_gain * err_fwd, -pos_vmax, pos_vmax)
            right_v_tgt = clamp(pos_gain * err_right, -pos_vmax, pos_vmax)
            if dist_m > 3.0:
                effort_gain = 150.0
            elif dist_m > 0.5:
                effort_gain = 220.0
            elif dist_m > 0.25:
                effort_gain = 280.0
            else:
                effort_gain = 340.0
            pitch_effort = clamp((fwd_v_tgt - vel_fwd) * effort_gain, -320.0, 320.0)
            roll_effort = clamp((right_v_tgt - vel_right) * effort_gain, -280.0, 280.0)
        else:
            roll_effort = clamp((0.24 * err_right - vel_right) * 34.0, -120.0, 120.0)
        if landing_phase == "APPROACH" and dist_m > 10.0 and (desired_los_speed - los_speed) > 1.5:
            pitch_effort = clamp(pitch_effort + 18.0, -190.0, 190.0)
        if landing_phase == "DESCENT" and dist_m < 2.5:
            pitch_effort = clamp(pitch_effort, -180.0, 180.0)

        need_forward = landing_phase == "APPROACH" and dist_m > 20.0 and desired_los_speed > 1.5
        if need_forward and pitch_effort > 30.0 and los_speed < -0.25 and abs(yaw_err) < 20.0:
            pitch_resp_bad += 1
        else:
            pitch_resp_bad = max(0, pitch_resp_bad - 1)

        # Flip тільки якщо не падаємо швидко і не в DESCENT
        if (
            pitch_resp_bad > int(4 * hz)
            and (time.time() - last_sign_flip_t) > 8.0
            and sign_flip_count < 1
            and landing_phase == "APPROACH"
            and dist_m > 20.0
            and vel_down < 1.0
            and ground_speed > 0.8
        ):
            pitch_axis_sign *= -1.0
            print(f"\n[LAND] Auto pitch sign flip -> {'+' if pitch_axis_sign > 0 else '-'}")
            pitch_resp_bad = 0
            sign_flip_count += 1
            last_sign_flip_t = time.time()

        if landing_phase == "DESCENT":
            roll_effort = clamp(roll_effort, -160.0, 160.0)

        pitch_pwm = 1500 + int(clamp(pitch_axis_sign * pitch_effort, -land_pitch_delta, land_pitch_delta))
        roll_pwm = 1500 + int(clamp(roll_axis_sign * roll_effort, -170, 170))
        yaw_pwm = 1500 + int(clamp(yaw_err * 6.0, -230, 230))

        if landing_phase == "DESCENT":
            if alt > 20:
                descent_rate = 1.60
            elif alt > 10:
                descent_rate = 1.10
            elif alt > 3:
                descent_rate = 0.75
            else:
                descent_rate = 0.40
            descent_rate *= descent_rate_scale * landing_rate_boost
            # Pinpoint landing:
            # keep fast descent almost to ground, then brief hold to center.
            if 2.5 < alt <= 4.0 and (dist_m > 0.45 or ground_speed > 0.45):
                hold_for_center = True
            if 1.2 < alt <= 2.5 and (dist_m > 0.25 or ground_speed > 0.20):
                hold_for_center = True
            # Below ~1.2m never hold altitude: commit to touchdown.
            if alt <= 1.2:
                hold_for_center = False
            if hold_for_center:
                descent_rate = 0.0
            land_alt_target = max(0.0, land_alt_target - descent_rate * dt)
            # Не даємо alt target "обганяти" поточну висоту зверху,
            # щоб уникнути підпору тягою.
            if hold_for_center:
                # True altitude hold while re-centering over B.
                land_alt_target = clamp(land_alt_target, alt - 0.03, alt + 0.03)
            else:
                margin = 0.8 if alt > 3.0 else 0.25
                land_alt_target = min(land_alt_target, max(0.0, alt - margin))
            descent_active = 0 if hold_for_center else 1
        else:
            descent_active = 0

        thr = throttle_from_alt_hold(land_alt_target, alt, vel_down, alt_state, dt, hover_pwm=float(hover_thr_est))
        if descent_active:
            descent_cut = 110 + (35 if vel_down < -0.30 else 0)
            if alt < 2.0:
                descent_cut = max(descent_cut, 70)
            thr -= descent_cut
        min_thr = 1120 if alt > 2.0 else 1080
        if descent_active:
            if alt > 10.0:
                max_thr = hover_thr_est - 20
            elif alt > 3.0:
                max_thr = hover_thr_est - 40
            else:
                max_thr = hover_thr_est - 60
        elif hold_for_center:
            max_thr = hover_thr_est - 20
        else:
            max_thr = hover_thr_est + (95 if dist_m > 8.0 else 35)
        thr = int(clamp(thr, min_thr, max_thr))

        rc.roll = int(clamp(roll_pwm, 1320, 1680))
        rc.pitch = int(clamp(pitch_pwm, land_pitch_min, land_pitch_max))
        rc.yaw = int(clamp(yaw_pwm, 1240, 1760))
        rc.throttle = thr
        rc.push(vehicle)

        now = time.time()
        if now >= next_land_log:
            print(
                f"[LAND] t={now - t_land:6.1f}s remain={dist_m:6.2f}m alt={alt:5.2f}m "
                f"gs={ground_speed:4.2f} vdown={vel_down:4.2f} altT={land_alt_target:5.2f} "
                f"los={los_speed:5.2f}/{desired_los_speed:4.1f} "
                f"phase={landing_phase} desc={descent_active} thr={thr:4d} "
                f"roll={rc.roll:4d} pitch={rc.pitch:4d} "
                f"lat={float(loc.lat):.6f} lon={float(loc.lon):.6f} "
                f"psgn={'+' if pitch_axis_sign > 0 else '-'}"
            )
            next_land_log = now + log_interval_s
        time.sleep(dt)
    print()
    land_duration_s = time.time() - t_land
    print(f"LAND duration: {land_duration_s:.2f}s")

    touchdown_loc = vehicle.location.global_relative_frame
    if touchdown_loc is not None and touchdown_loc.lat is not None and touchdown_loc.lon is not None:
        td_lat = float(touchdown_loc.lat)
        td_lon = float(touchdown_loc.lon)
        td_dist_m, _ = distance_and_bearing_m(td_lat, td_lon, target_lat, target_lon)
        print(f"Touchdown lat={td_lat:.6f} lon={td_lon:.6f} dist_to_B={td_dist_m:.2f}m")
    print("Touchdown detected. Cutting throttle and disarming...")
    for _ in range(int(2 * hz)):
        rc.roll = 1500
        rc.pitch = 1500
        rc.yaw = 1500
        rc.throttle = 1000
        rc.push(vehicle)
        time.sleep(dt)

    vehicle.armed = False
    t0 = time.time()
    while time.time() - t0 < 10:
        if not vehicle.armed:
            break
        time.sleep(0.2)

    RCOverride.clear(vehicle)
    print("Mission complete.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Fly ArduCopter SITL A->B in STABILIZE using RC override.")
    parser.add_argument("--connect", default="udp:127.0.0.1:14550", help="DroneKit connection string")
    parser.add_argument("--lat-a", type=float, default=50.450739, help="Start latitude (point A)")
    parser.add_argument("--lon-a", type=float, default=30.461242, help="Start longitude (point A)")
    parser.add_argument("--lat-b", type=float, default=50.443326, help="Target latitude (point B)")
    parser.add_argument("--lon-b", type=float, default=30.448078, help="Target longitude (point B)")
    parser.add_argument("--alt", type=float, default=100.0, help="Target relative altitude in meters")
    parser.add_argument("--arrival-radius", type=float, default=3.0, help="Arrival radius in meters")
    parser.add_argument(
        "--pitch-forward-pwm",
        type=int,
        default=1420,
        help="Pitch PWM for forward movement (flip to 1580 if direction is reversed)",
    )
    parser.add_argument("--hz", type=float, default=8.0, help="Control loop rate")
    parser.add_argument("--log-interval", type=float, default=1.0, help="Telemetry log period in seconds")
    parser.add_argument(
        "--a-hold-max-sec",
        type=float,
        default=0.0,
        help="Max seconds for optional re-center over A before transit (0=skip for fastest start)",
    )
    parser.add_argument(
        "--no-auto-pitch-flip",
        action="store_true",
        help="Disable automatic pitch direction flip when distance trend is wrong",
    )
    parser.add_argument(
        "--wind-from-deg",
        type=float,
        default=30.0,
        help="Напрямок звідки дує вітер (градуси, 0=північ). Визначає розташування IAF.",
    )
    parser.add_argument(
        "--iaf-dist-m",
        type=float,
        default=100.0,
        help="Відстань IAF від точки Б (метри). Дрон підлетить до Б проти вітру.",
    )
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=3.0,
        help=(
            "Множник швидкості польоту/зльоту "
            "(1.0=базово, 3.0=швидко, максимум 4.0)."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    preferred_connect = args.connect
    connect_candidates = [preferred_connect]
    if preferred_connect in ("tcp:127.0.0.1:5760", "tcp:localhost:5760"):
        connect_candidates.extend(["udp:127.0.0.1:14550", "udp:127.0.0.1:14551"])
    elif preferred_connect in ("udp:127.0.0.1:14550", "udp:localhost:14550"):
        connect_candidates.extend(["udp:127.0.0.1:14551", "tcp:127.0.0.1:5760"])
    elif preferred_connect in ("udp:127.0.0.1:14551", "udp:localhost:14551"):
        connect_candidates.extend(["udp:127.0.0.1:14550", "tcp:127.0.0.1:5760"])
    else:
        connect_candidates.extend(["udp:127.0.0.1:14550", "tcp:127.0.0.1:5760"])
    # Keep order but remove duplicates.
    connect_candidates = list(dict.fromkeys(connect_candidates))

    vehicle = None
    connected_to = None
    last_error = None
    for idx, endpoint in enumerate(connect_candidates):
        if idx == 0:
            print(f"Connecting to {endpoint} ...")
        else:
            print(f"Retrying connect via {endpoint} ...")
        hb_timeout = 10 if endpoint.startswith("tcp:") else 18
        try:
            vehicle = connect(endpoint, wait_ready=True, heartbeat_timeout=hb_timeout)
            connected_to = endpoint
            break
        except Exception as ex:
            last_error = ex
            print(f"Connect failed on {endpoint} (hb_timeout={hb_timeout}s): {ex}")
    if vehicle is None:
        raise last_error
    if connected_to != preferred_connect:
        print(f"Connected using fallback endpoint: {connected_to}")

    def on_sigint(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_sigint)

    print(
        f"MAVProxy marker command (final B, red flag): "
        f"map icon {args.lat_b:.6f} {args.lon_b:.6f} redflag"
    )
    print("Run it in MAVProxy console once per session to show final arrival point B.")

    try:
        fly_stabilize_rc(
            vehicle=vehicle,
            start_lat=args.lat_a,
            start_lon=args.lon_a,
            target_lat=args.lat_b,
            target_lon=args.lon_b,
            target_alt_m=args.alt,
            arrival_radius_m=args.arrival_radius,
            pitch_forward_pwm=args.pitch_forward_pwm,
            hz=max(2.0, args.hz),
            log_interval_s=max(0.2, args.log_interval),
            auto_pitch_flip=not args.no_auto_pitch_flip,
            a_hold_max_sec=max(0.0, args.a_hold_max_sec),
            wind_from_deg=args.wind_from_deg,
            iaf_dist_m=max(50.0, args.iaf_dist_m),
            speed_scale=args.speed_scale,
        )
        return 0
    except KeyboardInterrupt:
        emergency_abort(vehicle, hz=max(2.0, args.hz))
        return 130
    finally:
        if vehicle is not None:
            try:
                RCOverride.clear(vehicle)
            except Exception:
                pass
            vehicle.close()


if __name__ == "__main__":
    sys.exit(main())
