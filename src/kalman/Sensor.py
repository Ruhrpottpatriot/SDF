# A sensor gives the user a white noised measurement based on
# its actual (carthesian) position in the world at time t

from enum import Enum
from numpy import array, ndarray, subtract, add, degrees, arctan2, arccos, dtype, empty, empty_like, int32, mod
from numpy.random import standard_normal
from numpy.linalg import norm

vector = array

class SensorType(Enum):
    Polar = 1
    Carthesian = 2


class Sensor(object):
    def __init__(self, position: array, sensor_type: SensorType, trajectories: ndarray) -> None:
        self.dimSize: int32 = len(position)        
        self.position: array = position
        self.type: SensorType = sensor_type
        self.trajectories: ndarray = trajectories

        if len(trajectories[0][0]) != self.dimSize:
            raise IndexError("Passed trajectory coordinates have wrong dimension.")

        self.measured_coordinates: ndarray = empty(shape=(len(self.trajectories), self.dimSize))

    def __add_white_noise(self, position: vector) -> array:
        return add(position, standard_normal(size=len(position))*1000)

    def scan(self, time: int32) -> (ndarray, bool):
        # Simulate that a sensor only updates at a certain frequency
        if mod(time, 10) != 0: # 0.1Hz
            return (self.measured_coordinates, False)

        # Empty array so we work on fresh data
        self.measured_coordinates: ndarray = empty_like(self.measured_coordinates)

        # Measure all objects in the world
        for i, trajectory in enumerate(self.trajectories):
            # Get position at time t
            # add some white noise to the coordinates and then
            # calculate egocentrical (carthesian) coordinates
            objectPosition: vector = subtract(self.__add_white_noise(trajectory[time]), self.position)

            if self.type == SensorType.Polar:
                # polar coords are given as: [r, theta, phi]
                # r := |objectPosition|
                # theta := azimuth
                # phi := polar angle (N/A in 2D)

                r: float = norm(objectPosition)
                theta: float = degrees(arctan2(objectPosition[1], objectPosition[0]))

                if self.dimSize == 2:
                    self.measured_coordinates[i] = array([r, theta])

                if self.dimSize == 3:
                    phi = degrees(arccos(objectPosition[2]/r))
                    self.measured_coordinates[i] = array([r, theta, phi])

            if self.type == SensorType.Carthesian:
                self.measured_coordinates[i] = objectPosition
        
        return (self.measured_coordinates, True)
