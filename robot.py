import bldc_motor_driver
import pi_controller
import numpy as np
import json
import matplotlib.pyplot as plt




class Robot():
    def __init__(self):

        bldc_motor_data_path    = "./bldc_motor_data_kk_data.json"
        pi_controller_data_path = "./pi_controller_data.json"

        left_bldc_motor_data     = json.load(open(bldc_motor_data_path))
        left_pi_controller_data  = json.load(open(pi_controller_data_path))
        right_bldc_motor_data    = json.load(open(bldc_motor_data_path))
        right_pi_controller_data = json.load(open(pi_controller_data_path))

        self.left_bldc_motor_driver  = bldc_motor_driver.BLDC_Motor_Driver(left_bldc_motor_data,  left_pi_controller_data)
        self.right_bldc_motor_driver = bldc_motor_driver.BLDC_Motor_Driver(right_bldc_motor_data, right_pi_controller_data)

        # Simulation parameters
        self.dt = 0.001
        self.iterations = 10000
        t = np.ones(self.iterations)
        t = t*self.dt
        t = np.cumsum(t)
        self.t = t-self.dt

        self.u_dict = {
            "w_ref" : np.append(40*np.ones(int(self.iterations/2)), 10*np.ones(int(self.iterations/2))),
            "T_l"   : 0*np.ones(self.iterations)
        }

        self.R = 0                 # circle radius, variable
        self.l = 2            # distance between wheels, constant
        
        self.r = 0.5

        self.x = np.matrix([[0], 
                            [0], 
                            [0]])


# def matr(state):
#     return np.array([
#         [np.cos(self.x[2][0]),np.sin(self.x[2][0]), self.l], 
#         [np.cos(self.x[2][0]),np.sin(self.x[2][0]),-self.l]]) # to sie mnozy przez wektor wejśc


    def euler_simulation(self, u_dict, debug=False):
        self.x_coord = np.zeros(self.iterations)
        self.y_coord = np.zeros(self.iterations)

        self.left_w_ref = np.zeros(self.iterations)
        self.right_w_ref = np.zeros(self.iterations)

        for i in range(self.iterations):
            if debug:
                print()
                print()
                print()
            if i%1000 == 0:
                print(str((i/self.iterations)*100)+"%", "theta:")
                print(self.x)

            # punkt docelowy
            x_ref = u_dict["x_ref"][i]
            y_ref = u_dict["y_ref"][i]

            # wypakowanie stanu robota
            x_curr = self.x[0][0]
            y_curr = self.x[1][0]
            theta  = self.x[2][0]

            # prosta opisujaca styczna do okregu
            a_robot = np.tan(theta)
            b_robot = y_curr - a_robot*x_curr

            # prosta laczaca robota ze srodkiem okregu
            a2_robot = np.tan(theta-(np.pi/2))
            b2_robot = y_curr - a2_robot*x_curr

            # prosta laczaca robota z punktem docelowym
            a3_robot = (y_ref - y_curr)/(x_ref - x_curr)
            b3_robot = y_ref - a3_robot*x_ref

            x_between_robot_and_ref = (x_ref - x_curr)/2
            y_between_robot_and_ref = (y_ref - y_curr)/2
            
            # prosta prostopadla do prostej laczacej robota
            # i cel i przechodzaca przez srodek okregu
            if abs(a3_robot) < 0.001:
                a4_robot = 999999
            else:
                a4_robot = -1/a3_robot

            b4_robot = y_between_robot_and_ref - a4_robot*x_between_robot_and_ref

            x_center = (b2_robot - b4_robot)/(a4_robot - a2_robot)
            y_center = a4_robot*x_center + b4_robot

            if debug:
                print("1 circle center", x_center,  y_center)

            R_ref = np.sqrt((x_center-x_curr)**2 + (y_center-y_curr)**2)
            w_ref = 100

            w_l_ref = w_ref*(R_ref-(self.l/2))/self.r
            w_r_ref = w_ref*(R_ref+(self.l/2))/self.r
            self.left_w_ref[i]  = w_l_ref
            self.right_w_ref[i] = w_r_ref

            if debug:
                print("2, w refs", w_l_ref,  w_r_ref)

            # BLACK BOX START
            u_dict_left_bldc_motor_driver = {
                "w_ref": np.array([self.left_w_ref[i]]),
                "T_l":   np.array([0])
            }
            u_dict_right_bldc_motor_driver = {
                "w_ref": np.array([self.right_w_ref[i]]), 
                "T_l":   np.array([0])
            }

            y_dict_left_bldc_motor_driver  = \
                self.left_bldc_motor_driver.run_euler_simulation(self.dt, 1, u_dict_left_bldc_motor_driver)
            y_dict_right_bldc_motor_driver = \
                self.right_bldc_motor_driver.run_euler_simulation(self.dt, 1, u_dict_right_bldc_motor_driver)
                
            w_m_left  = y_dict_left_bldc_motor_driver["w_m"][0]
            w_m_right = y_dict_right_bldc_motor_driver["w_m"][0]
            # BLACK BOX END

            if debug:
                print("3 speeds", w_m_left,  w_m_right)

            v_l = w_m_left*self.r
            v_r = w_m_right*self.r



            matrx11 = np.array(np.cos(self.x[2][0]))[0][0]
            matrx12 = np.array(np.sin(self.x[2][0]))[0][0]
            matr = np.matrix([[matrx11, matrx12, self.l], 
                              [matrx11, matrx12,-self.l]])

            v = np.matrix([[v_l], [v_r]])
            self.x = self.x + np.transpose(matr)*v*self.dt

            if debug:
                print("3 speeds", w_m_left,  w_m_right)

            self.x_coord[i] = self.x[0][0]
            self.y_coord[i] = self.x[1][0]

        y_dict = {
            "x_coord": self.x_coord,
            "y_coord": self.y_coord
        }
        
        return y_dict


if __name__ == "__main__":
    robot = Robot()

    u_dict = {
        "x_ref": 21*np.ones(10000),
        "y_ref": 37*np.ones(10000)
    }

    y_dict = robot.euler_simulation(u_dict, True)
    x_coord = y_dict["x_coord"]
    y_coord = y_dict["y_coord"]


    plt.plot(x_coord, y_coord, "b")
    plt.plot(x_coord[0], y_coord[0], "ro")
    plt.plot(x_coord[-1], y_coord[-1], "go")
    plt.grid()
    plt.show()