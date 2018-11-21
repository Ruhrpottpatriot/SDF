# A sensor gives the user a white noised measurement based on 
# its actual (carthesian) position in the world at time t

from enum import Enum
from numpy import array, ndarray, subtract, add, degrees, arctan2, arccos
from numpy.random import standard_normal
from numpy.linalg import norm

class SensorType(Enum):
    Polar = 1
    Carthesian = 2

class Sensor(object):
    def __init__(self, position: array, sensor_type: SensorType) -> None:
        self.position: array = position
        self.type: SensorType = sensor_type

    def __random_scan(self, coordinates: array) -> array:
        return add(coordinates, standard_normal(size=coordinates.size()))

    def __scan(self, coordinates: array) -> array:
        # Add some white noise to the coordinates
        scan: array = self.__random_scan(coordinates)

        # calculate egocentrical (carthesian) coordinates
        pos: array = subtract(scan, self.position)

        if self.type == SensorType.Polar:
            # polar coors are given as: [r, theta, phi]
            # r := |vector|
            # theta := azimuth
            # phi := polar angle (N/A in 2D)

            r: float = norm(pos)
            theta: float = degrees(arctan2(pos[1], pos[0]))

            if coordinates.size() == 2:
                return array([r, theta])

            if coordinates.size() == 3:
                phi = degrees(arccos(pos[2]/r))
                return array([r, theta, phi])

            raise IndexError()

        if self.type == SensorType.Carthesian:
            return pos

    def full_scan(self, coordinatesList : ndarray) -> ndarray:
        for coordinate in coordinatesList:
            yield self.__scan(coordinate)
