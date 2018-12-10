import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time

#fig = plt.figure()
#ax = plt.axes(xlim=(0,2), ylim=(-2, 2))
#line, = ax.plot([], [], lw=2)


#def init():
#    line.set_data([], [])
#    return line,


#def animate(i):
#    x = np.linspace(0,2,1000)
#    y = np.sin(2*np.pi*(x-0.01*i))
#    line.set_data(x,y)
#    return line,


#ani = animation.FuncAnimation(fig, animate, init_func=init, frames=200, interval=20, blit=True)
#plt.show()

#https://jakevdp.github.io/blog/2012/08/18/matplotlib-animation-tutorial/








#x=np.linspace(0, 2*np.pi,1000)
#fig = plt.figure()
#ax = fig.add_subplot(1,1,1)
#for t in range(0,500):   #looping statement;declare the total number of frames
# y=np.sin(x-0.2*t)       # traveling Sine wave
# ax.clear()
# ax.plot(x,y)
# plt.pause(0.1)






class AnimationHandler:
    def __init__(self, ax):

        self.ax = ax

        self.lines   = [self.ax.plot([], []), self.ax.plot([], [])]
        self.colors  = ['cyan', 'red']
        self.n_steps = [360, 360]
        self.step = 0

    def init_animation(self):
        for anim_idx in [0, 1]:
            self.lines[anim_idx], = self.ax.plot([0, 10], [0, 0], c=self.colors[anim_idx], linewidth=2)
        self.ax.set_ylim([-2, 2])
        self.ax.axis('off')

        return tuple(self.lines)

    def update_slope(self, step, anim_idx):
        self.lines[anim_idx].set_data([0, 10], [0, np.sin(np.radians(step))])

    def animate(self, step):
        # animation 1
        if 0 < step < self.n_steps[0]:
            self.update_slope(step, anim_idx=0)

        # animation 2
        if self.n_steps[0] < step < sum(self.n_steps):
            self.update_slope(step - self.n_steps[0], anim_idx=1)

        return tuple(self.lines)


if __name__ == '__main__':
    fig, axes = plt.subplots()
    animator = AnimationHandler(ax=axes)
    my_animation = animation.FuncAnimation(fig,
                                           animator.animate,
                                           frames=sum(animator.n_steps),
                                           interval=10,
                                           blit=True,
                                           init_func=animator.init_animation,
                                           repeat=False)

    plt.show()