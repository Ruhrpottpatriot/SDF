# A sensor gives the user a white noised measurement based on
# its actual (carthesian) position in the world at time t

from enum import Enum
from numpy import array, reshape, shape, ndarray, subtract, add, degrees, arctan2, arccos, dtype, empty, empty_like, int32, mod
from numpy.random import standard_normal
from numpy.linalg import norm

vector = array

class SensorType(Enum):
    Polar = 1
    Carthesian = 2


class Sensor(object):
    def __init__(self, position: array, sensor_type: SensorType, trajectories: ndarray) -> None:
        trajsShape = shape(trajectories) 
        self.dimSize: int32 = trajsShape[2]

        # Make sure the position is a column vector
        if shape(position)[0] == 1:
            self.position = position
        else:
            self.position = reshape(position, (self.dimSize, 1))

        # Check dimensions
        if shape(position)[0] != self.dimSize:
            raise IndexError("Coordinate dimensions do not match!")

        self.type: SensorType = sensor_type
        self.trajectories: ndarray = trajectories          
        
        self.measured_coordinates: ndarray = empty(shape=(trajsShape[0], trajsShape[2], trajsShape[3]))

    def __add_white_noise(self, position: vector) -> array:
        return add(position, standard_normal((shape(position)[0], 1)) * 100)

    def scan(self, time: int32) -> (ndarray, bool):
        # Simulate that a sensor only updates at a certain frequency
        if mod(time, 5) != 0: # 0.3Hz
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
