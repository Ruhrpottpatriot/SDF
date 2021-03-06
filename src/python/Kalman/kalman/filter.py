from numpy import ndarray, int32, shape, array, identity, zeros, square, exp, matmul, eye, add, transpose, append, subtract, sin, cos, asarray, reshape, multiply, vstack, hstack
from numpy.linalg import inv

class KalmanFilter(object):
    def __init__(self, initialMeasurement: ndarray, initialCovariance: ndarray):
        dim: int32 = int32(initialMeasurement.shape[0] / 3)
        
        self.States: ndarray = array([initialMeasurement])
        self.Covariances: ndarray = array([initialCovariance])

        q_max: float = 9.1367
        d_t: float = 0.2        # was 1 before 
        theta: float = 30.0     # was 60 before
        sigma: float = 25.0     # was 50 before

        Id: ndarray = identity(dim)
        Null: ndarray = zeros((dim, dim))  

        # state transition
        self.F = vstack((
            hstack((Id, Id*d_t, Id*(square(d_t))*0.5)),
            hstack((Null, Id, Id*d_t)),
            hstack((Null, Null, Id*exp((-d_t)/theta)))
        ))
    
        # transition noise
        self.D = square(q_max) * (1.0 - exp(-2.0 * d_t / theta)) * vstack((
            hstack((Null, Null, Null)),
            hstack((Null, Null, Null)),
            hstack((Null, Null, Id))
        ))

        # Measurement matrix
        self.H = hstack((Id, Null, Null))

        # measurement noise
        self.R = square(sigma) * Id

    def predict(self) -> None:
        # Predict next object state
        predictedState: ndarray = matmul(self.F, self.States[-1])    
        self.States = append(self.States, [predictedState], 0)

        # Pedict how accurate the next state is going to be
        predictedCovariance = add(matmul(self.F, matmul(self.Covariances[-1], transpose(self.F))), self.D)
        self.Covariances = append(self.Covariances, [predictedCovariance], 0)

    def filter(self, measurement: ndarray) -> None:
        v = subtract(measurement, matmul(self.H, self.States[-1]))
        S = add(matmul(self.H, matmul(self.Covariances[-1], transpose(self.H))), self.R)
        W = matmul(self.Covariances[-1], matmul(transpose(self.H), inv(S)))

        self.States[-1] = add(self.States[-1], matmul(W, v))
        self.Covariances[-1] = subtract(self.Covariances[-1], matmul(W, matmul(S, transpose(W))))

    def retrodict(self) -> None:
        rangeCount = 10 if len(self.States) > 10 else len(self.States) - 1
        for i in range(rangeCount):
            pred_P: ndarray = add(matmul(self.F, matmul(self.Covariances[-(i+2)], transpose(self.F))), self.D)
            pred_X: ndarray = matmul(self.F, self.States[-(i+2)])
            W: ndarray = matmul(self.Covariances[-(i+2)], matmul(transpose(self.F), inv(pred_P)))

            self.States[-(i+2)] = add(self.States[-(i+2)], matmul(W, subtract(self.States[-(i+1)], pred_X)))                        
            self.Covariances[-(i+2)] = add(self.Covariances[-(i+2)], matmul(W, matmul(subtract(self.Covariances[-(i+1)], pred_P), transpose(W))))
                    
    @staticmethod
    def get_state_at_time(t: float) -> ndarray:
        v = 300.
        q = 9.
        A = v**2. / q
        omega = q / 2. * v

        return array([
            [A * sin(omega * t), A * sin(2. * omega * t),       # r
             v * cos(omega * t) / 2, v * cos(2 * omega * t),    # r'
             -q * sin(omega * t) / 4, -q * sin(2 * omega * t)]  # r''
        ]).transpose()
