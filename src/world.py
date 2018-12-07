from numpy import array, ndarray,shape, ones, empty, empty_like, arange, pi, power, sin, mod, vstack, hstack,identity, zeros, dot, eye, int32
from kalman.sensor import Sensor, SensorType
from kalman.filter import KalmanFilter
from numpy.random import standard_normal
from kalman.Error_Ellipse import calculate_ellipse

import matplotlib.pyplot as plt


class World(object):
    def __init__(self) -> None:
        # Create time steps array
        self.timeSteps = [int32(x) for x in arange(0, (400/3) * pi, 1)]

        # Create an optimal trajectory and plot it
        self.trajectories = [self.__calculate_trajectory()]
        for trajectory in self.trajectories:
            plt.scatter(*zip(*trajectory))

        # Set the dimension we're in, for now 2D
        dim:int32 = 2

        # Create a single sensor
        self.sensors: array = empty(1, dtype=object)
        self.sensors[0] = Sensor(zeros((dim, 1)), SensorType.Carthesian, self.trajectories)

        # Create a new KalmanFilter
        Id: ndarray = identity(dim)
        Null: ndarray = zeros((dim, dim))

        covariance = vstack((
            hstack((50**2 * Id, Null, Null)),
            hstack((Null, (300**2) * Id, Null)),
            hstack((Null, Null, (9**2) * Id))
        ))   

        state = vstack((
            dot(hstack((Id, Null, Null)), sum(KalmanFilter.get_state_at_time(0), 50 * standard_normal((dim*3, 1)))),
            ones((dim, 1)),
            ones((dim, 1))
        ))
        self.filter: KalmanFilter = KalmanFilter(state, covariance)

        # Run filter for each timestep
        for _, timestep in enumerate(self.timeSteps):
            self.filter.predict()

            color = ''
            (scan, isNew) = self.sensors[0].scan(timestep)    
            if isNew:
                self.filter.filter(scan)
                color = 'g'
            else:
                color = 'r'
                
            lastState = self.filter.States[-1]
            plt.plot(lastState[0,0], lastState[1,0], color=color, marker='+')
            # Plot ellipse
            a = calculate_ellipse(self.filter.States[-1], self.filter.Covariances[-1])
            plt.plot(a[0], a[1])

        plt.show()


    def get_real_trajectory(self):
        return self.trajectories

    def __calculate_trajectory(self) -> ndarray:
        amplitude = power(300, 2) / 9
        w = 9 / (2*300)

        trajectory: ndarray = empty((shape(self.timeSteps)[0], 2, 1))
        for i, t in enumerate(self.timeSteps):
            x = amplitude * sin(w * t)
            y = amplitude * sin(2 * w * t)
            trajectory[i] =  array([[x],[y]])

        return trajectory
