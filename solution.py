import time
import math
from enum import Enum
import collections
from collections import abc
# Fix for compatibility with certain dronekit versions:
collections.MutableMapping = abc.MutableMapping
from dronekit import connect, VehicleMode, Vehicle


NEUTRAL = 1500

class Channel(str, Enum):
    ROLL: str     = "1"
    PITCH: str    = "2"
    THROTTLE: str = "3"
    YAW: str      = "4"


def override_channels(
    v: Vehicle,
    r: int = NEUTRAL,
    p: int = NEUTRAL,
    t: int = NEUTRAL,
    y: int = NEUTRAL
) -> None:
    """
    Helper function used to override channels using function interface instead of dict mapping.
    RC channel values are set to 1500 by default, meaning middle joystick position

    :param v: Vehicle object
    :param r: Roll RC param
    :param p: Pitch RC param
    :param t: Throttle RC param
    :param y: Yaw RC param
    :return: None
    """
    v.channels.overrides = {
        Channel.ROLL.value: r,
        Channel.PITCH.value: p,
        Channel.THROTTLE.value: t,
        Channel.YAW.value: y
    }


def arm_and_takeoff(v: Vehicle, target_alt: int) -> None:
    """
    Wait drone to be armable, arm, then takeoff and reach target altitude.

    :param v: Vehicle object
    :param target_alt: target altitude in meters
    :return: None
    """

    while not v.is_armable:
        print("[INFO]: Waiting for vehicle to initialise...")
        time.sleep(1)

    print("[INFO]: Arming motors")
    v.armed = True

    while not v.armed:
        print("[INFO]: Waiting for arming...")
        time.sleep(1)

    print("[INFO]: Taking off!")

    while v.location.global_relative_frame.alt <= target_alt:
        override_channels(
            v=v,
            t=1900
        )


def calculate_yaw_to_target(v: Vehicle, target_coord: tuple[float, float]) -> float:
    """
    Function used to calculate bearing to target coordinate

    :param v: Vehicle object
    :param target_coord: Coordinate of the desired target
    :return: Yaw to desired target
    """
    lat1 = math.radians(v.location.global_relative_frame.lat)
    lon1 = math.radians(v.location.global_relative_frame.lon)
    lat2 = math.radians(target_coord[0])
    lon2 = math.radians(target_coord[1])

    d_lon = lon2 - lon1

    y = math.sin(d_lon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)

    yaw = math.degrees(math.atan2(y, x))
    return (yaw + 360) % 360  # Normalize to 0-360


def haversine(home: tuple[float, float], destination: tuple[float, float]) -> float:
    """
    Compute the Haversine distance (in meters) between two GPS coordinates.

    :param home: Home(start) coordinates
    :param destination: Destination coordinates
    :return: Distance in meters
    """
    r = 6371000
    lat1, lon1 = home
    lat2, lon2 = destination
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = math.sin(d_lat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return r * c


def compute_yaw_pwm(
    current_bearing: float,
    target_bearing: float,
    min_yaw_rate: float = 25,
    max_yaw_rate: float = 200
) -> tuple[int, float]:
    """
    Compute the PWM signal for yaw correction based on the minimal angular difference.

    :param current_bearing: Current yaw(vehicle.heading)
    :param target_bearing: Where you want to turn to
    :param min_yaw_rate: Min value used for yaw signal clipping
    :param max_yaw_rate: Max values used for yaw signal clipping
    :return: Tuple of (yaw_pwm, angular_difference).
    """
    diff = ((target_bearing - current_bearing + 180) % 360) - 180
    yaw_gain = max_yaw_rate / 180.0
    yaw_cmd = diff * yaw_gain
    if 0 < abs(yaw_cmd) < min_yaw_rate:
        yaw_cmd = min_yaw_rate * math.copysign(1, yaw_cmd)
    yaw_cmd = max(-max_yaw_rate, min(yaw_cmd, max_yaw_rate))
    return NEUTRAL + int(yaw_cmd), diff


def compute_forward_pitch(
    distance: float,
    forward_range: float = 400,
    max_distance_for_pitch: float = 50.0
) -> int:
    """
    Compute the pitch override based on the distance to the destination.
    Returns a PWM value for pitch with 1500 as neutral.

    :param distance: Distance to the desired target in meters.
    :param forward_range: Maximum PWM deviation for forward tilt.
    :param max_distance_for_pitch: Distance in meters at which the maximum forward_range is reached.
    :return: An integer PWM value for pitch control.
    """
    forward_gain = forward_range / max_distance_for_pitch
    forward_offset = forward_gain * distance
    return int(NEUTRAL - min(forward_offset, forward_range))


def turn_to(
    v: Vehicle,
    target_bearing: float,
) -> None:
    """
    Turn the drone to the specified target bearing using yaw control.

    :param v: Vehicle object
    :param target_bearing: Bearing where you would want drone face
    :return: None
    """
    while True:
        current_bearing = v.heading
        yaw_pwm, diff = compute_yaw_pwm(
            current_bearing=current_bearing,
            target_bearing=target_bearing
        )
        if abs(diff) < 0.5:
            override_channels(v=v, y=NEUTRAL)
            break
        override_channels(v=v, y=yaw_pwm)


def fly(
    v: Vehicle
) -> None:
    """
    Fly the drone to the destination coordinates while adjusting pitch and yaw in-flight.

    :param v: Vehicle object
    :return: None
    """
    while True:
        current_lat = v.location.global_relative_frame.lat
        current_lon = v.location.global_relative_frame.lon
        distance = haversine(
            home=(current_lat, current_lon),
            destination=(DEST_LAT, DEST_LON)
        )
        if distance < 2.0:
            override_channels(v=v, p=NEUTRAL, y=NEUTRAL)
            print("[INFO]: Destination reached.")
            break
        target_bearing = calculate_yaw_to_target(v=v, target_coord=(DEST_LAT, DEST_LON))
        yaw_pwm, diff = compute_yaw_pwm(
            current_bearing=vehicle.heading,
            target_bearing=target_bearing
        )
        new_pitch = compute_forward_pitch(distance=distance)
        override_channels(v=v, p=new_pitch, y=yaw_pwm)
        print(f"[INFO]: Distance: {distance:.1f} m, Bearing diff: {diff:.1f}°, Pitch: {new_pitch}, Yaw: {yaw_pwm}")
        time.sleep(0.1)


if __name__ == "__main__":

    HOME_LAT        = 50.450739
    HOME_LON        = 30.461242
    DEST_LAT        = 50.443326
    DEST_LON        = 30.448078
    TARGET_ALTITUDE = 100  # meters
    FINAL_YAW       = 350  # degrees

    vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True)
    vehicle.mode = VehicleMode("ALT_HOLD")

    arm_and_takeoff(
        v=vehicle,
        target_alt=TARGET_ALTITUDE
    )
    bearing = calculate_yaw_to_target(vehicle, target_coord=(DEST_LAT, DEST_LON))
    turn_to(v=vehicle, target_bearing=bearing)
    fly(v=vehicle)
    turn_to(v=vehicle, target_bearing=FINAL_YAW)


