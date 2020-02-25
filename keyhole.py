



from trajectory import *

class KeyholeTrajectory(Trajectory):

    def __init__(self, points, direction, v_des = 0.2, radius = 2, dt = 0.02):
        # points = [[x0, y0], [x1, y1]]
        # direction = 1 => horario, -1 => anti-horario
        super(KeyholeTrajectory,self).__init__(points, dt)
        self.direction = direction
        self.v_des = v_des
        self.radius = radius
        self.dt = dt
        assert self.verify_sanity()
        self.pos_inicial = self.points[0]
        self.pos_final = self.points[1]
        


    def verify_sanity(self):
        if(self.v_des <= 0):
            return 0, "Desired velocity should be greater than 0"
        elif(self.dt <= 0):
            return 0, "dt should be greater than 0"
        elif(self.radius <= np.hypot(self.points[0][0]-self.points[1][0], self.points[0][1]-self.points[1][1])/2):
            return 0, "Radius should be greater than half of the row distance"
        elif(self.direction == 0):
            return 0, "Direction should not be zero"
        elif(np.array(self.points).shape == (2,2)):
            return 1
        else:
            return 0, "Just 4 points are necessary: [[x0, y0], [x1,y1]]"

    @staticmethod
    def rMatrix(theta, delta_x, delta_y):
        return np.matrix([[math.cos(theta), -math.sin(theta), delta_x], 
                                    [math.sin(theta), math.cos(theta), delta_y],
                                    [0, 0, 0]])

    @staticmethod
    def rotate(a,theta, delta_x=0, delta_y=0):
        matriz_rotacao = KeyholeTrajectory.rMatrix(theta, delta_x, delta_y)
        aux = matriz_rotacao * np.array([a[0], a[1],1]).reshape((3,1))
        return np.array([aux[0], aux[1]]).reshape(2).tolist()



    @staticmethod
    def rotate_and_translate(xtraj, ytraj, theta=0, delta_x=0, delta_y=0):
        
        matriz_rotacao = KeyholeTrajectory.rMatrix(theta, delta_x, delta_y)
        aux = [matriz_rotacao * np.array([a,b,1]).reshape((3,1)) for a,b in zip(xtraj,ytraj)]

        xtraj2 = [a[0] for a in aux]
        ytraj2 = [a[1] for a in aux]

        xtraj2 = np.array(xtraj2).reshape((len(xtraj2))).tolist()
        ytraj2 = np.array(ytraj2).reshape((len(ytraj2))).tolist()

        return xtraj2, ytraj2


    @staticmethod
    def circle(r, t, theta=math.pi*2, alpha= 0, x=0 , y=0):
        tf = t[-1]
        xtraj = [r * math.cos(theta*i/tf) for i in t]
        ytraj = [r * math.sin(theta*i/tf) for i in t]

        xtraj, ytraj = KeyholeTrajectory.rotate_and_translate(xtraj, ytraj, alpha, x, y)

        return xtraj, ytraj

    @staticmethod
    def circle_arc(r, t, theta1=0, theta2=math.pi*2, alpha=0, x=0 , y=0):
        tf = t[-1]
        xtraj = [r * math.cos((theta2-theta1)*i/tf + theta1) for i in t]
        ytraj = [r * math.sin((theta2-theta1)*i/tf + theta1) for i in t]

        xtraj, ytraj = KeyholeTrajectory.rotate_and_translate(xtraj, ytraj, alpha, x, y)

        return xtraj, ytraj

    @staticmethod
    def line(p1, p2, t):
        a = (p2[0]-p1[0])/(t[-1]-t[0])
        a2 = (p2[1]-p1[1])/(t[-1]-t[0])
        x = [a*(tt - t[0]) + p1[0] for tt in t]
        y = [a2*(tt - t[0]) + p1[1] for tt in t]

        #print(x,y)
        #plt.plot(x,y)
        #plt.show()
        return x, y


    @staticmethod
    def arc(r, theta):
        return theta * r

    @staticmethod
    def dist(a,b):
        return math.hypot(a[0]-b[0], a[1]-b[1])

    @staticmethod
    def deg2rad(theta):
        return theta/180 * math.pi


    @staticmethod
    def keyhole_turning(r, w):
        theta = math.acos(w/(2*r))
        o2y = math.tan(theta) * w
        o1 = [r+w, 0]
        o2 = [r, o2y]

        return o1, o2, theta
        




    def return_trajectory(self):
        #global t, dt, tf, angle
        #angle = (math.atan2(pos_final[1]-pos_inicial[1], pos_final[0]-pos_inicial[0]))
        pos_inicial = self.pos_inicial
        pos_final = self.pos_final
        radius = self.radius
        v_des = self.v_des
        direcao = self.direction
        dt = self.dt

        pointsx = [pos_inicial[0], pos_final[0]]
        pointsy = [pos_inicial[1], pos_final[1]]

        angle = (math.atan2(pos_inicial[1]-pos_final[1], pos_inicial[0]-pos_final[0]))
        #if(direcao == -1):
        #   angle += math.pi
        distancia_entre_rows = math.hypot(pos_final[0]-pos_inicial[0], pos_final[1]-pos_inicial[1])
        ponto_medio = [(pos_final[0] + pos_inicial[0])/2.0, (pos_final[1] + pos_inicial[1])/2.0]


        print(angle)
        print(ponto_medio)


        distancia_percorrida = math.pi*distancia_entre_rows/2

        
        #radius = distancia_entre_rows/0.25

        o1, o2, theta = KeyholeTrajectory.keyhole_turning(radius, distancia_entre_rows)

        tf = KeyholeTrajectory.arc(radius,theta)/v_des
        print(tf)
        t = np.linspace(0, tf, tf//dt).tolist()

        tf2 = KeyholeTrajectory.arc(radius,theta+math.pi)/v_des
        
        t2 = np.linspace(0, tf2, tf2//dt).tolist()
        
        print(distancia_entre_rows)


        if(direcao == -1):
            o2 = [o2[0], -o2[1]]
        
        o1 = KeyholeTrajectory.rotate(o1, angle, pos_final[0], pos_final[1])
        o2 = KeyholeTrajectory.rotate(o2, angle, pos_final[0], pos_final[1])

        print(angle, theta)
        
        if(direcao == 1):
            xtraj, ytraj = KeyholeTrajectory.circle_arc(radius, t, math.pi+angle, math.pi+angle-theta, 0, o1[0],o1[1])
            xtraj2, ytraj2 = KeyholeTrajectory.circle_arc(radius, t2, angle-theta, math.pi+angle, 0, o2[0],o2[1])
        else:
            xtraj, ytraj = KeyholeTrajectory.circle_arc(radius, t, math.pi+angle, math.pi+angle+theta, 0, o1[0],o1[1])
            xtraj2, ytraj2 = KeyholeTrajectory.circle_arc(radius, t2, angle+theta, -math.pi+angle, 0, o2[0],o2[1])

        
        tf3 = KeyholeTrajectory.dist([xtraj2[-1], ytraj2[-1]], pos_final)/v_des
        
        t3 = np.linspace(0, tf3, tf3//dt).tolist()


        xtraj3, ytraj3 = KeyholeTrajectory.line([xtraj2[-1], ytraj2[-1]], pos_final, t3)


        t2 = t2[1:]
        xtraj2 = xtraj2[1:]
        ytraj2 = ytraj2[1:]
        t3 = t3[1:]
        xtraj3 = xtraj3[1:]
        ytraj3 = ytraj3[1:]
        
        '''
        t2 = [tt + t[-1] for tt in t2[1:]]
        xtraj2 = xtraj2[1:]
        ytraj2 = ytraj2[1:]
        t3 = [tt + t2[-1] for tt in t3[1:]]
        xtraj3 = xtraj3[1:]
        ytraj3 = ytraj3[1:]
        '''
        
        
        t.extend(t2)
        xtraj.extend(xtraj2)
        ytraj.extend(ytraj2)
        t.extend(t3)
        xtraj.extend(xtraj3)
        ytraj.extend(ytraj3)
        t = np.linspace(0, dt*len(t)-dt,len(t))
        vxtraj = np.gradient(xtraj,t)
        vytraj = np.gradient(ytraj,t)
        
        axtraj = np.gradient(vxtraj,t)
        aytraj = np.gradient(vytraj,t)



        theta_ref = [math.atan2(y_dot,x_dot) for (x_dot,y_dot) in zip(vxtraj, vytraj)]
        
        omega_ref = [(y_dot_dot * x_dot - x_dot_dot * y_dot)/(x_dot**2 + y_dot**2) for (x_dot, y_dot, x_dot_dot, y_dot_dot) in zip(vxtraj, vytraj, axtraj, aytraj)]
        omega_ref_dot = np.gradient(omega_ref, t)
        
        
        
        vel_ref= [math.sqrt(Vx**2+Vy**2) for (Vx, Vy) in zip(vxtraj, vytraj)]


        return xtraj, ytraj, theta_ref, vel_ref, vxtraj, vytraj, omega_ref, axtraj, aytraj, pointsx, pointsy


    

if __name__ == "__main__":
    points = [[0,0], [1,1]]
    radius = 2
    v_des = 0.2
    direction = 1
    a = KeyholeTrajectory(points, direction, v_des, radius)
    a.plot_trajectory()