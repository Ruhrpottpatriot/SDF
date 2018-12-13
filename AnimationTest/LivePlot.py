import numpy as np
import matplotlib.pyplot as plt


timeSteps = [int(x) for x in np.arange(0, (400/3) * np.pi, 1)]

amplitude = np.power(300, 2) / 9
w = 9 / (2 * 300)

for t in timeSteps:
    x = amplitude * np.sin(w * t)
    y = amplitude * np.sin(2 * w * t)
    plt.scatter(x, y, c = 'k', marker = '.')
    plt.pause(0.01)

plt.show()