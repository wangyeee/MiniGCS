from math import nan
from enum import Enum
from pymavlink.dialects.v10 import common as mavlink

WP_TYPE_NAMES = {
    mavlink.MAV_CMD_NAV_WAYPOINT : 'Waypoint',
    mavlink.MAV_CMD_NAV_LOITER_UNLIM : 'Loiter Unlimited',
    mavlink.MAV_CMD_NAV_LOITER_TURNS : 'Loiter Turns',
    mavlink.MAV_CMD_NAV_LOITER_TIME : 'Loiter Time',
    # mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH : 'Return to Launch',
    mavlink.MAV_CMD_NAV_LAND : 'Land',
    mavlink.MAV_CMD_NAV_TAKEOFF : 'Takeoff',
    mavlink.MAV_CMD_NAV_LAND_LOCAL : 'Land Local',
    mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL : 'Takeoff Local',
    mavlink.MAV_CMD_NAV_FOLLOW : 'Follow',
    mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT : 'Change Altitude',
    mavlink.MAV_CMD_NAV_LOITER_TO_ALT : 'Loiter to Altitude'
}

class MAVWaypointParameter(Enum):
    PARAM1 = 0
    PARAM2 = 1
    PARAM3 = 2
    PARAM4 = 3
    PARAM5 = 4
    PARAM6 = 5
    PARAM7 = 6

WP_NAV_WAYPOINT_DEFAULTS = {
    MAVWaypointParameter.PARAM1 : 0.0,
    MAVWaypointParameter.PARAM2 : 0.0,
    MAVWaypointParameter.PARAM3 : 0.0,
    MAVWaypointParameter.PARAM4 : nan,
    MAVWaypointParameter.PARAM5 : 0.0,
    MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_NAV_LOITER_UNLIM_DEFAULTS = {
    #MAVWaypointParameter.PARAM1 : 0.0,
    #MAVWaypointParameter.PARAM2 : 0.0,
    MAVWaypointParameter.PARAM3 : 0.0,
    MAVWaypointParameter.PARAM4 : nan,
    MAVWaypointParameter.PARAM5 : 0.0,
    MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_NAV_LOITER_TURNS_DEFAULTS = {
    MAVWaypointParameter.PARAM1 : 0.0,
    #MAVWaypointParameter.PARAM2 : 0.0,
    MAVWaypointParameter.PARAM3 : 0.0,
    MAVWaypointParameter.PARAM4 : nan,
    MAVWaypointParameter.PARAM5 : 0.0,
    MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_NAV_LOITER_TIME_DEFAULTS = {
    MAVWaypointParameter.PARAM1 : 0.0,
    #MAVWaypointParameter.PARAM2 : 0.0,
    MAVWaypointParameter.PARAM3 : 0.0,
    MAVWaypointParameter.PARAM4 : nan,
    MAVWaypointParameter.PARAM5 : 0.0,
    MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_NAV_LAND_DEFAULTS = {
    MAVWaypointParameter.PARAM1 : 0.0,
    MAVWaypointParameter.PARAM2 : mavlink.PRECISION_LAND_MODE_DISABLED,
    #MAVWaypointParameter.PARAM3 : 0.0,
    MAVWaypointParameter.PARAM4 : nan,
    MAVWaypointParameter.PARAM5 : 0.0,
    MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_NAV_TAKEOFF_DEFAULTS = {
    MAVWaypointParameter.PARAM1 : 0.0,
    #MAVWaypointParameter.PARAM2 : 0.0,
    #MAVWaypointParameter.PARAM3 : 0.0,
    MAVWaypointParameter.PARAM4 : nan,
    MAVWaypointParameter.PARAM5 : 0.0,
    MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_NAV_LAND_LOCAL_DEFAULTS = {
    MAVWaypointParameter.PARAM1 : 0.0,
    MAVWaypointParameter.PARAM2 : 0.0,
    MAVWaypointParameter.PARAM3 : 0.0,
    MAVWaypointParameter.PARAM4 : 0.0,
    MAVWaypointParameter.PARAM5 : 0.0,
    MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_NAV_TAKEOFF_LOCAL_DEFAULTS = {
    MAVWaypointParameter.PARAM1 : 0.0,
    #MAVWaypointParameter.PARAM2 : 0.0,
    MAVWaypointParameter.PARAM3 : 0.0,
    MAVWaypointParameter.PARAM4 : 0.0,
    MAVWaypointParameter.PARAM5 : 0.0,
    MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_NAV_FOLLOW_DEFAULTS = {
    MAVWaypointParameter.PARAM1 : 0.0,
    MAVWaypointParameter.PARAM2 : 0.0,
    MAVWaypointParameter.PARAM3 : 0.0,
    MAVWaypointParameter.PARAM4 : 0.0,
    MAVWaypointParameter.PARAM5 : 0.0,
    MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_NAV_CONTINUE_AND_CHANGE_ALT_DEFAULTS = {
    MAVWaypointParameter.PARAM1 : 0.0,
    #MAVWaypointParameter.PARAM2 : 0.0,
    #MAVWaypointParameter.PARAM3 : 0.0,
    #MAVWaypointParameter.PARAM4 : 0.0,
    #MAVWaypointParameter.PARAM5 : 0.0,
    #MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_NAV_LOITER_TO_ALT_DEFAULTS = {
    MAVWaypointParameter.PARAM1 : 0.0,
    MAVWaypointParameter.PARAM2 : 0.0,
    #MAVWaypointParameter.PARAM3 : 0.0,
    MAVWaypointParameter.PARAM4 : 0.0,
    MAVWaypointParameter.PARAM5 : 0.0,
    MAVWaypointParameter.PARAM6 : 0.0,
    MAVWaypointParameter.PARAM7 : 0.0
}

WP_DEFAULTS = {
    mavlink.MAV_CMD_NAV_WAYPOINT : WP_NAV_WAYPOINT_DEFAULTS,
    mavlink.MAV_CMD_NAV_LOITER_UNLIM : WP_NAV_LOITER_UNLIM_DEFAULTS,
    mavlink.MAV_CMD_NAV_LOITER_TURNS : WP_NAV_LOITER_TURNS_DEFAULTS,
    mavlink.MAV_CMD_NAV_LOITER_TIME : WP_NAV_LOITER_TIME_DEFAULTS,
    mavlink.MAV_CMD_NAV_LAND : WP_NAV_LAND_DEFAULTS,
    mavlink.MAV_CMD_NAV_TAKEOFF : WP_NAV_TAKEOFF_DEFAULTS,
    mavlink.MAV_CMD_NAV_LAND_LOCAL : WP_NAV_LAND_LOCAL_DEFAULTS,
    mavlink.MAV_CMD_NAV_TAKEOFF_LOCAL : WP_NAV_TAKEOFF_LOCAL_DEFAULTS,
    mavlink.MAV_CMD_NAV_FOLLOW : WP_NAV_FOLLOW_DEFAULTS,
    mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT : WP_NAV_CONTINUE_AND_CHANGE_ALT_DEFAULTS,
    mavlink.MAV_CMD_NAV_LOITER_TO_ALT : WP_NAV_LOITER_TO_ALT_DEFAULTS
}
