"""Metre-based goal geometry. No ROS, CAN, or implicit scale correction."""
import math


def finite_vector(values, size, name):
    if len(values) != size or not all(math.isfinite(float(v)) for v in values):
        raise ValueError(f'{name} requires {size} finite numbers')
    return [float(v) for v in values]


def relative_goal(start, offset):
    x, y, yaw = finite_vector(start, 3, 'Start pose')
    forward, left = finite_vector(offset, 2, 'Forward/left offset')
    return [x + math.cos(yaw)*forward - math.sin(yaw)*left,
            y + math.sin(yaw)*forward + math.cos(yaw)*left]


def distance_metrics(start, goal, points):
    start = finite_vector(start[:2], 2, 'Start')
    goal = finite_vector(goal, 2, 'Goal')
    points = [finite_vector(p, 2, 'Path point') for p in points]
    straight = math.dist(start, goal)
    route = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    return {'straight_distance_m': straight, 'planned_length_m': route,
            'detour_ratio': route/straight if straight > .01 else None,
            'units': 'm', 'pose_reference': 'chassis_center'}


def validate_anchor(current, anchor, position_limit=.15, yaw_limit=math.radians(5)):
    current = finite_vector(current, 3, 'Current pose')
    anchor = finite_vector(anchor, 3, 'Goal anchor')
    angle = math.atan2(math.sin(current[2]-anchor[2]), math.cos(current[2]-anchor[2]))
    if math.dist(current[:2], anchor[:2]) > position_limit or abs(angle) > yaw_limit:
        raise ValueError('Vehicle moved after relative target was frozen; re-enter the target')


def motion_metrics(trace):
    """Integrate only paired fresh samples; wheel distance is an estimate, not a tape measure."""
    wheel=0.;lio=0.;seconds=0.
    for a,b in zip(trace,trace[1:]):
        dt=b['elapsed_s']-a['elapsed_s']
        if not 0<dt<.5 or not a.get('wheel'):continue
        wheel+=abs(a['wheel'][0])*dt
        lio+=math.dist(a['pose'][:2],b['pose'][:2]);seconds+=dt
    return {'wheel_travelled_m':wheel,'lio_travelled_m':lio,'paired_sample_seconds':seconds,
            'lio_to_wheel_ratio':lio/wheel if wheel>.03 else None}
