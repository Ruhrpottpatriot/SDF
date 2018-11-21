# A sensor gives the user a white noised measurement based on
# its actual (carthesian) position in the world at time t

from enum import Enum
from numpy import array, ndarray, subtract, add, degrees, arctan2, arccos, dtype, empty_like, int32, mod
from numpy.random import standard_normal
from numpy.linalg import norm


class SensorType(Enum):
    Polar = 1
    Carthesian = 2


class Sensor(object):
    def __init__(self, position: array, sensor_type: SensorType, objectsCoordinates: ndarray) -> None:
        self.position: array = position
        self.type: SensorType = sensor_type
        self.objectsCoordinates: ndarray = objectsCoordinates
        self.measured_coordinates: ndarray = empty_like(self.objectsCoordinates)

    def __add_white_noise(self, coordinates: array) -> array:
        return add(coordinates, standard_normal(size=coordinates.size)*1000)

    def scan(self, time: int32) -> array:
        # Simulate that a sensor only updates at a certain frequency
        if mod(time, 10) != 0:
            return

        # Empty array so we work on fresh data
        self.measured_coordinates: ndarray = empty_like(self.objectsCoordinates)

        # Measure all objects in the world
        for i, coordinate in enumerate(self.objectsCoordinates):
            # Add some white noise to the coordinates and then
            # calculate egocentrical (carthesian) coordinates
            pos: array = subtract(self.__add_white_noise(coordinate), self.position)

            if self.type == SensorType.Polar:
                # polar coords are given as: [r, theta, phi]
                # r := |vector|
                # theta := azimuth
                # phi := polar angle (N/A in 2D)

                r: float = norm(pos)
                theta: float = degrees(arctan2(pos[1], pos[0]))

                if coordinate.len() == 2:
                    self.measured_coordinates[i] = array([r, theta])

                if coordinate.len() == 3:
                    phi = degrees(arccos(pos[2]/r))
                    self.measured_coordinates[i] = array([r, theta, phi])

                raise IndexError()

            if self.type == SensorType.Carthesian:
                self.measured_coordinates[i] = pos
        
        return self.measured_coordinates
