


import numpy as np

# maybe inherit some functions? 
from trajectory import Trajectory





class PolynomialTrajectory(Trajectory):
    def __init__(self, points, restrictions, T, dt = 0.02):
        super(PolynomialTrajectory,self).__init__(points, dt)
        self.T = T
        self.restrictions = []
        self.restrictions.append(self.set_restrictions(points[0],restrictions[0]))
        self.restrictions.append(self.set_restrictions(points[1],restrictions[1]))
        assert self.verify_sanity(self.points, self.restrictions, self.T), "Ill-defined Points, restrictions or T"
        self.num_points = len(points[0])
        self.size = PolynomialTrajectory.matrix_n(len(points[0]))
        self.A = np.zeros((self.size, self.size))
        self.define_A()
        self.define_b()
        self.calculate_constants()

    def verify_sanity(self, points, restrictions, T):

        num_points = len(points[0])
        num_points2 = len(points[1])
        num_T = len(T)
        num_restrictions = 0
        num_restrictions2 = 0
        for list_restriction in restrictions[0]:
            num_restrictions += len(list_restriction)
        for list_restriction in restrictions[1]:
            num_restrictions2 += len(list_restriction)
        if(num_restrictions == 4 * (num_points-1) and num_restrictions2 == 4*(num_points-1) and num_points == num_T and num_points2 == num_T):
            return 1
        else:
            return 0

    def set_restrictions(self,points, restrictions):
        # given 3 points, r defines restrictions [x0(t0), x0_dot(t0)] in P0, [x0(t1), x0_dot(t1), x0_dot_dot(t1), x1(t1)] in P1 and 
        # [x1(t2), x1_dot(t2)] in P2, x0 and x1 representing the curves
        r = []
        for i in range(len(points)):
            if(i == 0 or i == len(points) - 1):
                r.append([points[i]] + [a for a in restrictions[i]])
            else:
                r.append([points[i]] + [0] + [0] + [points[i]])
        print(r)
        return r

    @staticmethod
    def matrix_n(num_waypoints):
        # considering system order = 2 to create cubic splines
        return 4 * (num_waypoints-1)


    def A_small(self, d_order, t):
        if(d_order == 0):
            return np.array([t**3, t**2,t,1])
        elif(d_order == 1):
            return np.array([3*t**2, 2*t,1,0])
        else:
            return np.array([6*t, 2,0,0])



    # Ax = b
    
    def define_b(self):
        self.b = [[],[]]
        # given 3 points, first list_restriction defines restrictions [x0(t0), x0_dot(t0)] in P0, second [x0(t1), x0_dot(t1), x0_dot_dot(t1), x1(t1)] in P1 and 
        # last [x1(t2), x1_dot(t2)] in P2, x0 and x1 representing the curves
        print self.restrictions
        for restrictions, i in zip(self.restrictions, range(len(self.restrictions))):
            for list_restriction in restrictions:
                self.b[i].extend(list_restriction)
        self.b = np.array(self.b).T#.reshape((len(self.b[0]),2))
    

    def define_A(self):
        for count in range(self.num_points):
            if(count == 0):
                self.A[0, 0:4] = self.A_small(0, self.T[count])
                self.A[1, 0:4] = self.A_small(1, self.T[count])
            elif(count == self.num_points-1):
                self.A[-2, -4:] = self.A_small(0, self.T[count])
                self.A[-1, -4:] = self.A_small(1, self.T[count])
            else:
                self.A[count*4 - 2, (count-1)*4:(count-1)*4+4] = self.A_small(0, self.T[count])
                self.A[count*4 - 1, (count-1)*4:(count-1)*4+4] = self.A_small(1, self.T[count])
                self.A[count*4 - 1, (count)*4:(count)*4+4] = - self.A_small(1, self.T[count])
                self.A[count*4 - 0, (count-1)*4:(count-1)*4+4] = self.A_small(2, self.T[count])
                self.A[count*4 - 0, (count)*4:(count)*4+4] = - self.A_small(2, self.T[count])
                #self.A[count*4 - 0, (count-1)*4:(count-1)*4+4] = self.A_small(1, self.T[count])
                self.A[count*4 + 1, (count)*4:(count)*4+4] = self.A_small(0, self.T[count])
        
    def calculate_constants(self):
        print(self.A)
        print(self.b)
        print(self.b[:,0])
        print(self.b[:,1])
        self.cx = np.linalg.solve(self.A, self.b[:,0].reshape(len(self.b),1))
        self.cy = np.linalg.solve(self.A, self.b[:,1].reshape(len(self.b),1))
        self.cx = np.concatenate(self.cx,axis=0)
        self.cy = np.concatenate(self.cy,axis=0)
        print(self.cx)
        print(self.cy)
        

                

    
    def return_trajectory(self):

        x = []
        y = []
        vx = []
        vy = []
        ax = []
        ay = []
        t = []
        for t1,t2,i in zip(T[:-1], T[1:], range(len(T)-1)):
            t.append(np.linspace(t1,t2,(t2-t1)/self.dt).tolist())
            if(i != len(T)-2):
                t[i].remove(t2)
        

            
        def calculate_poly(c,t):
            return c[0] * t ** 3 + c[1] * t ** 2 + c[2] * t + c[3]

        def calculate_traj(time,const,vector):
            for traj, i in zip(time, range(len(time))):
                for timestep in traj:
                    vector.append(calculate_poly(const[i*4:i*4+4], timestep))
            return vector

        def derivative(c):
            const = []
            for i in range(0,len(c)//4):
                const.extend([0] + np.polyder(c[i*4:i*4+4]).tolist())
            return const


        
        for traj, i in zip(t, range(len(t))):
            for timestep in traj:
                x.append(calculate_poly(self.cx[i*4:i*4+4], timestep))
        for traj, i in zip(t, range(len(t))):
            for timestep in traj:
                y.append(calculate_poly(self.cy[i*4:i*4+4], timestep))
        

        #calculate_traj(t, self.cx, x)
        #calculate_traj(t, self.cy, y)
        
        calculate_traj(t, derivative(self.cx), vx)
        calculate_traj(t, derivative(self.cy), vy)
        calculate_traj(t, derivative(derivative(self.cx)), ax)
        calculate_traj(t, derivative(derivative(self.cy)), ay)
        
        
        #x = np.concatenate(x, axis=0)
        #y = np.concatenate(y, axis=0)
        #vx = np.concatenate(x, axis=0)
        #vy = np.concatenate(y, axis=0)
        #ax = np.concatenate(x, axis=0)
        #ay = np.concatenate(y, axis=0)
        t = np.concatenate(t, axis=0)
        

        #vx = np.gradient(x,t)
        #vy = np.gradient(y,t)

        import math
        v = [math.sqrt(aa**2 + bb**2) for aa,bb in zip(vx,vy)]
        
        #ax = np.gradient(vx,t)
        #ay = np.gradient(vy,t)
        
        theta = [math.atan2(y_dot,x_dot) for (x_dot,y_dot) in zip(vx, vy)]

        omega = [(y_dot_dot * x_dot - x_dot_dot * y_dot)/(x_dot**2 + y_dot**2) for (x_dot, y_dot, x_dot_dot, y_dot_dot) in zip(vx, vy, ax, ay)]

        return x,y,theta,v,vx,vy,omega,ax,ay,self.points[0],self.points[1]




















if __name__ == "__main__":
    # points = [[p0], [p1], ..., [pn-1], [pn]]
    # restrictions = [[v0], [], ..., [], [vn]]
    # for now, we can only set initial and final velocities
    restrictions = [[[0], [], [0]], [[1],[],[-1]]]
    points = [[0,0.3,-0.5], [0,0.3,0]]
    T = [0,1,4]
    a = PolynomialTrajectory(points,restrictions,T)
    a.plot_trajectory()
    '''
    print(a.A)
    print(a.b)
    print(a.c)
    import matplotlib.pyplot as plt 

    x = []
    t1 = np.linspace(T[0],T[1],1000)
    t2 = np.linspace(T[1],T[2],1000)
    t2 = t2[1:]
    def calculate_poly(c,t):
        return c[0] * t ** 3 + c[1] * t ** 2 + c[2] * t + c[3]


    for t in t1:
        x.append(calculate_poly(a.c[0:4], t))
    for t in t2:
        x.append(calculate_poly(a.c[-4:], t))


    restrictions = [[1], [], [-1]]
    pointsy = [0,0.3,0]
    T = [0,1,4]
    b = PolynomialTrajectory(pointsy,restrictions,T)
    print(b.A)
    print(b.b)
    print(b.c)
    import matplotlib.pyplot as plt 

    y = []
    for t in t1:
        y.append(calculate_poly(b.c[0:4], t))
    for t in t2:
        y.append(calculate_poly(b.c[-4:], t))

    ttotal =[]
    ttotal.extend(t1)
    ttotal.extend(t2)
    plt.plot(x,y)
    plt.plot(points, pointsy, 'ro')
    plt.show()

    x = np.concatenate(x, axis=0)
    y = np.concatenate(y, axis=0)

    vx = np.gradient(x,ttotal)
    vy = np.gradient(y,ttotal)

    plt.plot(vx)
    plt.title("X velocity")
    plt.show()
    plt.plot(vy)
    plt.title("Y velocity")
    plt.show()
    import math
    v = [math.sqrt(aa**2 + bb**2) for aa,bb in zip(vx,vy)]
    plt.plot(v)
    plt.title("Linear Velocity")
    plt.show()
    
    ax = np.gradient(vx,ttotal)
    ay = np.gradient(vy,ttotal)

    plt.plot(ax)
    plt.title("X acceleration")
    plt.show()
    plt.plot(ay)
    plt.title("Y acceleration")
    plt.show()
    
    theta_ref = [math.atan2(y_dot,x_dot) for (x_dot,y_dot) in zip(vx, vy)]

    omega_ref = [(y_dot_dot * x_dot - x_dot_dot * y_dot)/(x_dot**2 + y_dot**2) for (x_dot, y_dot, x_dot_dot, y_dot_dot) in zip(vx, vy, ax, ay)]


    plt.plot(theta_ref)
    plt.title("Theta")
    plt.show()
    plt.plot(omega_ref)
    plt.title("Omega")
    plt.show()
    '''
    

'''

import numpy as np



class PolynomialTrajectory:
    def __init__(self, points, restrictions, T):
        self.points = points
        self.restrictions = self.set_restrictions(points,restrictions)
        self.T = T
        assert self.verify_sanity(self.points, self.restrictions, self.T)
        self.num_points = len(points)
        self.size = PolynomialTrajectory.matrix_n(len(points))
        self.A = np.zeros((self.size, self.size))
        self.define_A()
        self.define_b()
        self.c = self.calculate_constants()

    def verify_sanity(self, points, restrictions, T):

        num_points = len(points)
        num_T = len(T)
        num_restrictions = 0
        for list_restriction in restrictions:
            num_restrictions += len(list_restriction)
        if(num_restrictions == 4 * (num_points-1) and num_points == num_T):
            return 1
        else:
            return 0

    def set_restrictions(self,points, restrictions):
        # given 3 points, r defines restrictions [x0(t0), x0_dot(t0)] in P0, [x0(t1), x0_dot(t1), x0_dot_dot(t1), x1(t1)] in P1 and 
        # [x1(t2), x1_dot(t2)] in P2, x0 and x1 representing the curves
        r = []
        for i in range(len(points)):
            if(i == 0 or i == len(points) - 1):
                r.append([points[i]] + [a for a in restrictions[i]])
            else:
                r.append([points[i]] + [0] + [0] + [points[i]])
        return r

    @staticmethod
    def matrix_n(num_waypoints):
        # considering system order = 2 to create cubic splines
        return 4 * (num_waypoints-1)


    def A_small(self, d_order, t):
        if(d_order == 0):
            return np.array([t**3, t**2,t,1])
        elif(d_order == 1):
            return np.array([3*t**2, 2*t,1,0])
        else:
            return np.array([6*t, 2,0,0])



    # Ax = b

    def define_b(self):
        self.b = []
        # given 3 points, first list_restriction defines restrictions [x0(t0), x0_dot(t0)] in P0, second [x0(t1), x0_dot(t1), x0_dot_dot(t1), x1(t1)] in P1 and 
        # last [x1(t2), x1_dot(t2)] in P2, x0 and x1 representing the curves
        for list_restriction in self.restrictions:
            self.b.extend(list_restriction)
        self.b = np.array(self.b).reshape((len(self.b),1))

    def define_A(self):
        for count in range(self.num_points):
            if(count == 0):
                self.A[0, 0:4] = self.A_small(0, self.T[count])
                self.A[1, 0:4] = self.A_small(1, self.T[count])
            elif(count == self.num_points-1):
                self.A[-2, -4:] = self.A_small(0, self.T[count])
                self.A[-1, -4:] = self.A_small(1, self.T[count])
            else:
                self.A[count*4 - 2, (count-1)*4:(count-1)*4+4] = self.A_small(0, self.T[count])
                self.A[count*4 - 1, (count-1)*4:(count-1)*4+4] = self.A_small(1, self.T[count])
                self.A[count*4 - 1, (count)*4:(count)*4+4] = - self.A_small(1, self.T[count])
                self.A[count*4 - 0, (count-1)*4:(count-1)*4+4] = self.A_small(2, self.T[count])
                self.A[count*4 - 0, (count)*4:(count)*4+4] = - self.A_small(2, self.T[count])
                #self.A[count*4 - 0, (count-1)*4:(count-1)*4+4] = self.A_small(1, self.T[count])
                self.A[count*4 + 1, (count)*4:(count)*4+4] = self.A_small(0, self.T[count])
        
    def calculate_constants(self):
        return np.linalg.solve(self.A, self.b)

                






















if __name__ == "__main__":
    # points = [[p0], [p1], ..., [pn-1], [pn]]
    # restrictions = [[v0], [], ..., [], [vn]]
    # for now, we can only set initial and final velocities
    restrictions = [[0], [], [0]]
    points = [0,0.3,-0.5]
    T = [0,1,4]
    a = PolynomialTrajectory(points,restrictions,T)
    print(a.A)
    print(a.b)
    print(a.c)
    import matplotlib.pyplot as plt 

    x = []
    t1 = np.linspace(T[0],T[1],1000)
    t2 = np.linspace(T[1],T[2],1000)
    t2 = t2[1:]
    def calculate_poly(c,t):
        return c[0] * t ** 3 + c[1] * t ** 2 + c[2] * t + c[3]


    for t in t1:
        x.append(calculate_poly(a.c[0:4], t))
    for t in t2:
        x.append(calculate_poly(a.c[-4:], t))


    restrictions = [[1], [], [-1]]
    pointsy = [0,0.3,0]
    T = [0,1,4]
    b = PolynomialTrajectory(pointsy,restrictions,T)
    print(b.A)
    print(b.b)
    print(b.c)
    import matplotlib.pyplot as plt 

    y = []
    for t in t1:
        y.append(calculate_poly(b.c[0:4], t))
    for t in t2:
        y.append(calculate_poly(b.c[-4:], t))

    ttotal =[]
    ttotal.extend(t1)
    ttotal.extend(t2)
    plt.plot(x,y)
    plt.plot(points, pointsy, 'ro')
    plt.show()

    x = np.concatenate(x, axis=0)
    y = np.concatenate(y, axis=0)

    vx = np.gradient(x,ttotal)
    vy = np.gradient(y,ttotal)

    plt.plot(vx)
    plt.title("X velocity")
    plt.show()
    plt.plot(vy)
    plt.title("Y velocity")
    plt.show()
    ax = np.gradient(vx,ttotal)
    ay = np.gradient(vy,ttotal)

    plt.plot(ax)
    plt.title("X acceleration")
    plt.show()
    plt.plot(ay)
    plt.title("Y acceleration")
    plt.show()
    import math
    theta_ref = [math.atan2(y_dot,x_dot) for (x_dot,y_dot) in zip(vx, vy)]

    omega_ref = [(y_dot_dot * x_dot - x_dot_dot * y_dot)/(x_dot**2 + y_dot**2) for (x_dot, y_dot, x_dot_dot, y_dot_dot) in zip(vx, vy, ax, ay)]


    plt.plot(theta_ref)
    plt.title("Theta")
    plt.show()
    plt.plot(omega_ref)
    plt.title("Omega")
    plt.show()


'''