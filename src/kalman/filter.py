from numpy import ndarray, array, identity, zeros, power, exp, dot, sum, transpose, append, subtract
from numpy.linalg import inv

class KalmanFilter(object):
    def __init__(self, initialMeasurement: ndarray, initialCovariance: ndarray):        
        self.States: ndarray = [initialMeasurement]
        self.Covariances: ndarray = [initialCovariance]

        q_max: float = 9.1367
        d_t:float = 5.0
        theta:float = 60.0
        Id:ndarray = identity(2)
        Null: ndarray = zeros(2,2)
        sigma:float = 50.0

        # state transition 
        self.F = ndarray(
            [
                [Id, Id*5, Id*(power(d_t, 2))*0.5],
                [Null, Id, Id*d_t], 
                [Null, Null, Id*exp((-d_t)/theta)]
            ])
        
        # transition noise
        self.D = power(q_max, 2) * (1.0 - exp(-2.0 * d_t / theta)) * ndarray(
            [
                [Null, Null, Null], 
                [Null, Null, Null],
                [Null, Null, Id]
            ])

        # Mmeasurement matrix
        self.H = ndarray([Id, Null, Null])

        # measurement noise
        self.R = Id * power(sigma, 2)

    def predict(self) -> None:
        predictedState: ndarray = dot(self.F, self.States[-1])
        self.States.append(predictedState)
        
        predictedCovariance = sum(dot(self.F, dot(self.Covariances[-1], transpose(self.F))), self.D)
        self.Covariances.append(predictedCovariance)

    def filter(self, measurement: array) -> None:
        v = subtract(measurement, dot(self.H, self.States[-1]))
        S = sum(dot(self.H, dot(self.Covariances[-1], transpose(self.H))), self.R)
        W = dot(self.Covariances[-1], dot(transpose(self.H), inv(S)))

        self.States[-1] = sum(self.States[-1], dot(W, v))
        self.Covariances[-1] = subtract(self.Covariances[-1], dot(W, dot(S, transpose(W))))

    def retrodict(self) -> None:
        rangeCount = 10 if len(self.States) > 10 else len(self.States) - 1
        for i in range(rangeCount):
            pred_P: ndarray = sum(dot(self.F, dot(self.Covariances[-(i+2)], transpose(self.F))), self.D)
            pred_X: ndarray = dot(self.F, self.States[-(i+2)])

            W: ndarray = dot(self.Covariances[-(i+2)], dot(self.F, inv(pred_P)))

            self.States[-(i+2)] =  sum(self.States[-(i+2)], dot(W, subtract(self.States[-(i+1)], pred_X)))
            self.Covariances[-(i+2)] = sum(self.Covariances[-(i+2)], dot(W, dot(subtract(self.Covariances[-(i+1)], pred_P)), transpose(W)))