import numpy as np
import matplotlib.pyplot as plt
import json


class PI_Controller:
    def __init__(self, pi_controller_data):
        self.K_p = pi_controller_data["K_p"]
        self.T_i = pi_controller_data["T_i"]

        self.A = np.matrix([0])
        self.B = np.matrix([self.K_p/self.T_i])
        self.C = np.matrix([1])
        self.D = np.matrix([self.K_p])

        x11 = pi_controller_data["init"]
        self.x = np.matrix([x11])

        self.sat = pi_controller_data["sat"]
        self.T_a = pi_controller_data["T_a"]
        self.anti = pi_controller_data["anti"]


    def simulation_euler(self, dt, iterations, u_dict):
        y = np.zeros(iterations)

        for i in range(iterations):
            u      = u_dict["e"][i]

            y[i]   =              self.C*self.x + self.D*u
            self.x = self.x + dt*(self.A*self.x + self.B*u)
            
        return y


    def simulation_euler_anti_windup(self, dt, iterations, u_dict, **kwargs):
        # Prepare arrays for signals
        y      = np.zeros(iterations)
        y_prim = np.zeros(iterations)

        for i in range(iterations):
            if "show" in kwargs.keys() and kwargs["show"]:
                if (i+1)%1000 == 0:
                    print(str(((i+1)/iterations)*100)+"%")

            u = u_dict["e"][i]
            y[i]   =              self.C*self.x + self.D*u
            self.x = self.x + dt*(self.A*self.x + self.B*u + self.anti)

            # anti wind-up system
            if   y[i] > self.sat:
                y_prim[i] = self.sat
            elif y[i] < -self.sat:
                y_prim[i] = -self.sat
            else:
                y_prim[i] = y[i]

            self.anti = (y_prim[i] - y[i])/self.T_a

        y_dict = {"y": y_prim}

        return y_dict
        

if __name__ == "__main__":
    # Simulation data
    pi_controller_data = json.load(open("./pi_controller_data.json"))
    pi_controller = PI_Controller(pi_controller_data)