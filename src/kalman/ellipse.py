from numpy import ndarray, identity, cos, sin, int32, transpose, zeros, hstack, shape, eye, add , sqrt, matmul, pi, linspace, array
from numpy.linalg import eig


class error_ellipse(object):
    @staticmethod
    def calculate_ellipse(State: ndarray, Covariance: ndarray) -> ndarray:
        """
        Calculate the error ellipse from given state and covariance and return the coords to plot
        """
        dim = int32(shape(State)[0] / 3)

        [V, D] = eig(matmul(eye(dim, dim * 3), matmul(Covariance, transpose(eye(dim, dim * 3)))))

        ny = matmul(eye(dim, dim * 3), State)
        t = linspace(0, 2 * pi)

        return array((add((V[0] * sqrt(D[0][0])) * transpose(cos(t[:])), ny[0]),
                      add((V[1] * sqrt(D[1][1])) * transpose(sin(t[:])), ny[1])))