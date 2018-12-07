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