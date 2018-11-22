from numpy import ndarray, array, int32, matrix, exp, transpose, power, invert

class KalmanFilter(object):
    def __init__(self):
        d_t = 5
        theta = 60
        Id:matrix = matrix([[1,0], [0,1]])
        Null: matrix = matrix([[0,0],[0,0]])
        q_max = 1 - exp(-2*d_t/theta)
        sigma = 50

        self.F = matrix([[Id, Id*5, Id*(power(d_t, 2))*0.5],[Null, Id, Id*d_t], [Null, Null, Id*exp((-d_t)/theta)]] )
        self.D = power(q_max, 2) * matrix([[Null, Null, Null], [Null, Null, Null], [Null, Null]])
        self.H = matrix([Id, Null, Null])
        self.R = Id * power(sigma, 2)

    def predict(self, probabilityMatrix: matrix, state: array) -> (matrix, matrix):
        x_k1_k: matrix = self.F  * state
        P_k1_k: matrix = self.F * probabilityMatrix * transpose(self.F) + self.D        
        return (x_k1_k, P_k1_k)

    def filter(self, currentMeasurement: array, probabilityMatrix: matrix, state: array) -> (matrix, matrix):
        v = currentMeasurement - (self.H * state)
        S = self.H * probabilityMatrix * transpose(self.H) + self.R
        W = probabilityMatrix * (transpose(self.H) * invert(S))

        x_k1_k1 = state + W*v
        P_k1_k1 = probabilityMatrix - (W * S * transpose(W))
        return (x_k1_k1, P_k1_k1)

    def retrodict(self, probabilityMatrix: matrix, state: array) -> (matrix, matrix):
        pass # Apply measurements to all past positions