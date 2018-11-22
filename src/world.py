from numpy import array, ndarray, empty, empty_like, arange, pi, power, sin, mod
from kalman.sensor import Sensor, SensorType
from kalman.filter import KalmanFilter

import matplotlib.pyplot as plt


class World(object):
    def __init__(self) -> None:
        # Create time steps array
        self.timeSteps = arange(0, (400/3) * pi, 1)

        # Create an optimal trajectory and plot it
        self.trajectories = [self.__calculate_trajectory()]
        for trajectory in self.trajectories:
            plt.scatter(*zip(*trajectory))

        # Create a single sensor
        self.sensors: array = empty(1, dtype=object)
        self.sensors[0] = Sensor(
            [0, 0], SensorType.Carthesian, self.trajectories)

        # Create a new KalmanFilter
        self.filter: KalmanFilter = KalmanFilter(self.sensors)

        # Run filter for each timestep
        for _, timestep in enumerate(self.timeSteps):
            predCoord = self.filter.predict(timestep)
            filterCoord = self.filter.filter(timestep)
            retrodictCoord = self.filter.retrodict(timestep)

        # Make measurement
        for i, sensor in enumerate(self.sensors):
            for trajectory in self.trajectories:
                for j, position in enumerate(trajectory):
                    if mod(j, 10) != 0:
                        continue

                    scan = sensor.full_scan([position])
                    plt.scatter(*zip(*scan))

        plt.show()

    def get_real_trajectory(self):
        return self.trajectories

    def __calculate_trajectory(self) -> ndarray:
        amplitude = power(300, 2) / 9
        w = 9 / (2*300)

        trajectories: ndarray = empty((self.timeSteps.size, 2))
        for i, t in enumerate(self.timeSteps):
            x = amplitude * sin(w * t)
            y = amplitude * sin(2 * w * t)
            trajectories[i] = array([x, y])

        return trajectories
