# Calculate the error ellipse from given state and covariance and return the coords to plot
from numpy import *
import matplotlib.pyplot as plt


class error_ellipse():

    def calculate_ellipse(State: ndarray, Covariance: ndarray) -> ndarray:

        dim = int32(shape(State)[0]/3)
        [V, D] = linalg.eig(hstack((eye(dim), zeros((dim, dim)), zeros((dim, dim)))) @ \
            Covariance @\
            transpose(hstack((eye(dim), zeros((dim, dim)), zeros((dim, dim))))))
        ny = hstack((eye(dim), zeros((dim, dim)), zeros((dim, dim)))) @ State
        t = linspace(0, 2*pi)

        return array((add((V[0] * sqrt(D[0][0])) * transpose(cos(t[:])), ny[0]),
                      add((V[1] * sqrt(D[1][1])) * transpose(sin(t[:])), ny[1])))


# test
state = vstack(([[10000], [10000]], [[0], [0]], [[0], [0]]))
Cov = hstack((vstack((250 * eye(2), zeros((2, 2)), zeros((2, 2)))), vstack((zeros((2, 2)), 300 * eye(2), zeros((2, 2)))), vstack((zeros((2, 2)), zeros((2, 2)), 300 * eye(2)))))

a = error_ellipse.calculate_ellipse(state, Cov)
plt.plot(a[0], a[1])
plt.show()
