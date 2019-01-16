from numpy import array, mod, square, exp, ndarray, shape, ones, empty, empty_like, arange, pi, power, sin, mod, vstack, hstack, identity, zeros, dot, eye, int32
from kalman.sensor import Sensor, SensorType
from kalman.filter import KalmanFilter
from numpy.random import standard_normal
from kalman.ellipse import error_ellipse

import matplotlib.pyplot as plt


class World(object):
    def __init__(self) -> None:
        showActual = False
        showPositions = False
        showEllipses = False
        showRetroctict = False
        showAll = False

        # 1. Show actual
        # 2. Show positions and every n-th block
        # 3. Show retrodiction in comparison to measurements
        # 4. Show retrodiction compared to actual (all)
        showStepSize = 5
        currentStep = 0
        if currentStep == 0:
            showActual = True
            showPositions = True
            showEllipses = True
            showRetroctict = True
            showAll = True
        elif currentStep == 1:
            showActual = True
            showPositions = False
            showEllipses = False
            showRetroctict = False
            showAll = False
        elif currentStep == 2:
            showActual = False
            showPositions = True
            showEllipses = True
            showRetroctict = False
            showAll = False
        elif currentStep == 3:
            showActual = False
            showPositions = True
            showEllipses = True
            showRetroctict = True
            showAll = False
        elif currentStep == 4:
            showActual = True
            showPositions = False
            showEllipses = False
            showRetroctict = True
            showAll = True
        else:
            raise Exception("Need valid step!")

        # Create time steps array
        self.timeSteps = [int32(x) for x in arange(0, (400/3) * pi, 1)]

        # Create an optimal trajectory and plot it
        self.trajectories = [self.__calculate_trajectory()]
        if showActual:
            for trajectory in self.trajectories:
                plt.scatter(*zip(*trajectory))

        # Set the dimension we're in, for now 2D
        dim: int32 = 2

        # Create a single sensor
        self.sensors: array = empty(1, dtype=object)
        self.sensors[0] = Sensor(
            zeros((dim, 1)), SensorType.Carthesian, self.trajectories)

        # Create a new KalmanFilter
        Id: ndarray = identity(dim)
        Null: ndarray = zeros((dim, dim))

        covariance = vstack((
            hstack((10**2 * Id, Null, Null)),   # was 50 before
            hstack((Null, (20**2) * Id, Null)),  # was 300 before
            hstack((Null, Null, (9**2) * Id))
        ))

        state = vstack((
            dot(hstack((Id, Null, Null)), sum(
                KalmanFilter.get_state_at_time(0), 50 * standard_normal((dim*3, 1)))),
            zeros((dim, 1)),
            zeros((dim, 1))
        ))

        dim: int32 = int32(state.shape[0] / 3)
        q_max: float = 9.1367
        d_t: float = 0.2        # was 1 before
        theta: float = 30.0     # was 60 before
        sigma: float = 25.0     # was 50 before

        Id: ndarray = identity(dim)
        Null: ndarray = zeros((dim, dim))

        # state transition
        F = vstack((
            hstack((Id, Id*d_t, Id*(square(d_t))*0.5)),
            hstack((Null, Id, Id*d_t)),
            hstack((Null, Null, Id*exp((-d_t)/theta)))
        ))

        # transition noise
        D = square(q_max) * (1.0 - exp(-2.0 * d_t / theta)) * vstack((
            hstack((Null, Null, Null)),
            hstack((Null, Null, Null)),
            hstack((Null, Null, Id))
        ))

        # Measurement matrix
        H = hstack((Id, Null, Null))

        # measurement noise
        R = square(sigma) * Id

        self.filter: KalmanFilter = KalmanFilter(state, covariance, F, D, H, R)

        drawBlock = False
        blockCounter = 0
        for i, timestep in enumerate(self.timeSteps):
            self.filter.predict()

            color = ''
            linestyle = ''
            (scan, isNew) = self.sensors[0].scan(timestep)
            if isNew:
                self.filter.filter(scan)
                color = 'g'
                linestyle = 'solid'
            else:
                color = 'r'
                linestyle = 'dotted'

            # Plot error elipse
            # Calculate block starts
            if mod(i, 5 * showStepSize) == 0:  # Step size * 4
                drawBlock = True

            if showEllipses and (drawBlock or showAll):
                a = error_ellipse.calculate_ellipse(
                    self.filter.States[-1], self.filter.Covariances[-1])
                plt.plot(a[0], a[1], color=color, linestyle=linestyle)

            # Plot position
            if showPositions:
                lastState = self.filter.States[-1]
                plt.plot(lastState[0, 0], lastState[1, 0],
                         color=color, marker='+')

            if isNew:
                self.filter.retrodict()

            if drawBlock:
                blockCounter += 1
                if blockCounter > 5:
                    blockCounter = 0
                    drawBlock = False

        # Finished, print retrodiction
        drawBlock = False
        blockCounter = 0
        if showRetroctict:
            for i in range(self.filter.States.shape[0]):
                if mod(i, 5 * showStepSize) == 0:  # Step size * 4
                    drawBlock = True

                if drawBlock or showAll:
                    state = self.filter.States[i, :, :]
                    cov = self.filter.Covariances[i, :, :]
                    plt.plot(state[0, 0], state[1, 0], color='k', marker='+')

                    e = error_ellipse.calculate_ellipse(state, cov)
                    plt.plot(e[0], e[1], color='k', linestyle='--')

                if drawBlock:
                    blockCounter += 1
                if blockCounter > 5:
                    blockCounter = 0
                    drawBlock = False

        plt.show()

    def get_real_trajectory(self):
        return self.trajectories

    def __calculate_trajectory(self) -> ndarray:
        amplitude = power(300, 2) / 9
        w = 9 / (2 * 300)

        trajectory: ndarray = empty((shape(self.timeSteps)[0], 2, 1))
        for i, t in enumerate(self.timeSteps):
            x = amplitude * sin(w * t)
            y = amplitude * sin(2 * w * t)
            trajectory[i] = array([[x], [y]])

        return trajectory
