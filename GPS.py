"""
GPS.py
donkeycar part class for interfacing with GPS.

@authors:
"""

# adding parts to donkeycar
#http://docs.donkeycar.com/parts/about/

from numpy import pi, cos, sin, arctan2, sqrt, square, radians
import pynmea2


class GPS():
    def __init__(self, baudRate, pollRate, port, goalLocation):
        """
        important GPS parameters
        baud rate, port#, polling interval?
        TODO: other important stuff for GPS?
        """
        self.baudRate = baudRate
        self.pollRate = pollRate  # Hz
        self.port = port #ttys0

        # GPS coordinates used for controller
        self.currLocation = []
        self.goalLocation = goalLocation


    def poll(self):
        """
        Sidney's GPS polling function
        Method to poll gps data and parse using GPStoRad
        """
        # gpsdata = poll from gps
        # TODO: add polling function HERE
        identifier = "$GPGGA"  # identifier for relevant GPS NMEA data
        if identifier in gpsdata:
            # update the current location of car
            currLocation = self.GPStoRad(gpsdata)
            self.currLocation = currLocation
        else:
            # do nothing; don't need the other serial data
            continue
        return currLocation


    def GPStoRad(self, gpsData):
        """
        GPStoRad(gpsData)

        Method to parse GPS data and determine angular positon using the pynmea2 library.
        @params: gpsData in GPGGA format
        @return: gpsFloat tuple (latitude, longditude) in radians
        """
        nmeaObj = pynmea2.parse(gpsData)  # create nmea object
        lat = radians(nmeaObj.latitude)  # degrees->radians
        lon = radians(nmeaObj.longitude)  # degrees->radians
        gpsFloat = (float(lat), float(lon))

        # print for debugging
        # print("GPS Coordinates in Radians: (%f, %f)" % (gpsFloat[0], gpsFloat[1]))
        return gpsFloat


class Planner():
    def __init__(self, controller_gain):
        self.controller_gain = controller_gain
        self.bearingSetpoint = []  # car gps bearing setpoint
        self.currentLocation = []
        # TODO: other important planner parameters


    def steering_controller(self):
        """
        steering_controller()

        Method to implement a steering proportional controller for donkeycar
        @params: bearing_setpoint, bearing_current
        @return: steering command
        """
        # TODO: add controller Method

        # Proportional controller
        bearingError = self.bearingSetpoint - self.currentLocation
        steer_cmd = self.controller_gain * bearingError

        return


    def calcBearingSetpoint(self):
        """
        calcBearingSetpoint()

        calculate the bearing setpoint (radians) from the start position to the goal.
        """

        # TODO: determine a way to calculate the bearing setpoint
        # maybe average the GPS readings (seem to be noisy) for a period of time
        # and then calculate the bearing to the goal from the average reading.


# Methods useful for implementing planning algorithim
def haversine(pointA, pointB):
    """
    haversine()

    Method to calculate the (great circle) distance over a sphere with the haversine formula.
    Best for distances >1km. Otherwise computationally excessive.
    @params: two gps coordinates pointA and pointB (radians) defined by lat and long coordinates
    @return: distance between the two points in meters
    """
    if (type(pointA) != list) or (type(pointB) != list):
        raise TypeError("Only lists are supported as arguments")

    # radius of earth (m)
    r_earth = 6371e3

    # extract lat and long coordinates
    lat1 = pointA[0]
    lon1 = pointA[1]
    lat2 = pointB[0]
    lon2 = pointB[1]

    dlat = float(lat2) - float(lat1)  # change in latitude
    dlon = float(lon2) - float(lon1)  # change in longitude

    # distance over a sphere using the Haversine formula
    # https://www.movable-type.co.uk/scripts/latlong.html
    a = square(sin(dlat/2)) + cos(lat1)*cos(lat2)*square(sin(dlon/2))
    c = 2*arctan2(sqrt(a), sqrt(1-a))
    d = r_earth * c

    return d


def distBetweenGPSPoints(pointA, pointB):
    """
    distBetweenGPSPoints()

    Method to calculate the straight-line approximation between two gps coordinates.
    Used for distances on the 10-1000m scale.
    @params: two gps points A & B (radians) defined by lat and long coordinates
    @return: distance between the two points in meters
    """
    if (type(pointA) != list) or (type(pointB) != list):
        raise TypeError("Only lists are supported as arguments")

    # radius of earth (m)
    r_earth = 6371e3

    # extract lat and long coordinates
    lat1 = pointA[0]
    lon1 = pointA[1]
    lat2 = pointB[0]
    lon2 = pointB[1]

    dlat = lat2 - lat1  # change in latitude
    dlon = lon2 - lon1  # change in longitude

    dx = r_earth*dlon*cos((lat1+lat2)/2)
    dy = r_earth*dlat

    dist = sqrt(square(dx)+square(dy))  # straight line approximation

    return dist


def calcHeading(pointA, pointB):
    """
    calcHeading(prevLocation, currLocation, goalLocation)

    Method to calculate the heading between two points A and B w.r.t. North
    @params: two gps points A and B (lat, long) (radians)
    @return: heading from current location to goal (radians)
    """
    if (type(pointA) != list) or (type(pointB) != list):
        raise TypeError("Only lists are supported as arguments")

    # extract lat and long coordinates
    lat1 = pointA[0]
    lon1 = pointA[1]
    lat2 = pointB[0]
    lon2 = pointB[1]

    diffLon = lon2 - lon1
    x = sin(diffLon) * cos(lat2)
    y = cos(lat1) * sin(lat2) - (sin(lat1)*cos(lat2)*cos(diffLon))

    initialHeading = arctan2(x, y)

    # remap from [-pi,pi] to [0, 2*pi] for compass heading
    compassHeadingRad = (initialHeading + 2*pi) % (2*pi)

    return compassHeadingRad
