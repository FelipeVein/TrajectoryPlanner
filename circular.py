

from trajectory import * 
class CircularTrajectory(Trajectory):

    def __init__(self, points, direction, v_des = 0.2, dt = 0.02):

        super(CircularTrajectory,self).__init__(points, dt)
        self.direction = - direction ### CAUTION!
        self.v_des = v_des
        self.dt = dt
        assert self.verify_sanity()
        self.pos_inicial = self.points[0]
        self.pos_final = self.points[1]
        


    def verify_sanity(self):
        if(self.v_des <= 0):
            return 0, "Desired velocity should be greater than 0"
        elif(self.dt <= 0):
            return 0, "dt should be greater than 0"
        elif(self.direction == 0):
            return 0, "Direction should not be zero"
        elif(np.array(self.points).shape == (2,2)):
            return 1
        else:
            return 0, "Just 4 points are necessary: [[x0, y0], [x1,y1]]"




    def return_trajectory(self):

        pos_inicial = self.pos_inicial
        pos_final = self.pos_final
        v_des = self.v_des
        direcao = self.direction
        dt = self.dt

        pointsx = [pos_inicial[0], pos_final[0]]
        pointsy = [pos_inicial[1], pos_final[1]]


        angle = (math.atan2(pos_final[1]-pos_inicial[1], pos_final[0]-pos_inicial[0]))
        if(direcao == -1):
            angle += math.pi
        distancia_entre_rows = math.hypot(pos_final[0]-pos_inicial[0], pos_final[1]-pos_inicial[1])
        ponto_medio = [(pos_final[0] + pos_inicial[0])/2.0, (pos_final[1] + pos_inicial[1])/2.0]


        matriz_rotacao = np.matrix([[math.cos(angle), -math.sin(angle), ponto_medio[0]], 
                                    [math.sin(angle), math.cos(angle), ponto_medio[1]],
                                    [0, 0, 0]])

        print(angle)
        print(ponto_medio)

        distancia_percorrida = math.pi*distancia_entre_rows/2
        tf = distancia_percorrida/v_des
        print(tf)
        t = np.linspace(0, tf, tf//dt)
        
        while(angle > math.pi):
            angle = angle - 2*math.pi
        while(angle < -math.pi):
            angle = angle + 2*math.pi
        xtraj = [distancia_entre_rows/2 * math.cos(math.pi*i/tf) for i in t]
        ytraj = [distancia_entre_rows/2 * math.sin(math.pi*i/tf) for i in t]
        if(np.sign(direcao) == 1):
            xtraj.reverse()
            ytraj.reverse()
        aux = [matriz_rotacao * np.array([a,b,1]).reshape((3,1)) for a,b in zip(xtraj,ytraj)]
        xtraj = [a[0] for a in aux]
        ytraj = [a[1] for a in aux]

        xtraj = np.array(xtraj).reshape((len(xtraj))).tolist()
        ytraj = np.array(ytraj).reshape((len(ytraj))).tolist()
        
        vxtraj = np.gradient(xtraj,t)
        vytraj = np.gradient(ytraj,t)
        axtraj = np.gradient(vxtraj,t)
        aytraj = np.gradient(vytraj,t)



        theta_ref = [math.atan2(y_dot,x_dot) for (x_dot,y_dot) in zip(vxtraj, vytraj)]
        omega_ref = [(y_dot_dot * x_dot - x_dot_dot * y_dot)/(x_dot**2 + y_dot**2) for (x_dot, y_dot, x_dot_dot, y_dot_dot) in zip(vxtraj, vytraj, axtraj, aytraj)]

        vel_ref= [math.sqrt(Vx**2+Vy**2) for (Vx, Vy) in zip(vxtraj, vytraj)]

    


        return xtraj, ytraj, theta_ref, vel_ref, vxtraj, vytraj, omega_ref, axtraj, aytraj, pointsx, pointsy






if __name__ == "__main__":
    points = [[0,0], [1,1]]
    v_des = 0.2
    direction = 1
    a = CircularTrajectory(points, direction, v_des)
    a.plot_trajectory()