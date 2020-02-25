

import numpy as np
import math
import matplotlib.pyplot as plt



class Trajectory(object):
    def __init__(self, points, dt):
        self.points = points
        self.dt = dt

    def return_trajectory(self):
        assert 0, "Not Implemented in {}.".format(self.__class__)

    def plot_trajectory(self):

        x,y,theta,v,vx,vy,omega,ax,ay,pointsx,pointsy = self.return_trajectory()

        plt.plot(x,y)
        plt.plot(pointsx, pointsy, 'ro')
        plt.axis('equal')
        plt.show()


        plt.plot(vx)
        plt.title("X velocity")
        plt.show()
        plt.plot(vy)
        plt.title("Y velocity")
        plt.show()
        
        plt.plot(v)
        plt.title("Linear Velocity")
        plt.ticklabel_format(useOffset=False)
        plt.show()
        
        
        plt.plot(ax)
        plt.title("X acceleration")
        plt.show()
        plt.plot(ay)
        plt.title("Y acceleration")
        plt.show()
        
        #theta = [math.atan2(y_dot,x_dot) for (x_dot,y_dot) in zip(vx, vy)]

        #omega = [(y_dot_dot * x_dot - x_dot_dot * y_dot)/(x_dot**2 + y_dot**2) for (x_dot, y_dot, x_dot_dot, y_dot_dot) in zip(vx, vy, ax, ay)]


        plt.plot(theta)
        plt.title("Theta")
        plt.show()
        plt.plot(omega)
        plt.title("Omega")
        plt.show()