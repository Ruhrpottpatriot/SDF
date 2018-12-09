import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


fig = plt.figure()
ax = plt.axes(xlim=(0,2), ylim=(-2, 2))
line, = ax.plot([], [], lw=2)

def init():
    line.set_data([], [])
    return line,

def animate(i):
    x = np.linspace(0,2,1000)
    y = np.sin(2*np.pi*(x-0.01*i))
    line.set_data(x,y)
    return line,

ani = animation.FuncAnimation(fig, animate, init_func=init, frames=200, interval=20, blit=True)
plt.show()



#https://jakevdp.github.io/blog/2012/08/18/matplotlib-animation-tutorial/