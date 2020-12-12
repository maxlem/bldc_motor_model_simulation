import bldc_motor_driver
import numpy as np
import json


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
        self.dt = 0.01
        self.iterations = 10000
        t = np.ones(iterations)
        t = t*dt
        t = np.cumsum(t)
        self.t = t-dt


        self.u_dict = {
            "w_ref" : np.append(40*np.ones(int(iterations/2)), 10*np.ones(int(iterations/2))),
            "T_l"   : 0*np.ones(iterations)
        }


    def __speeds2position_derivative(self, ):



    def euler_simulation(self, u_dict):
        self.x = np.zeros(self.iterations)
        self.y = np.zeros(self.iterations)

        for i in range(len(self.iterations)):
            x_ref = u_dict["x_ref"][i]
            y_ref = u_dict["y_ref"][i]

            if i==0:
                e_x = x_ref
                e_y = y_ref
            else:
                e_x = x_ref - self.x[i-1]
                e_y = y_ref - self.y[i-1]

            # code for conversion into w_ref for left and right BLDC Motor
            

            u_dict_left_bldc_motor_driver = {
                "w_ref":, 
                "T_l": np.array([0])
            }
            u_dict_right_bldc_motor_driver = {
                "w_ref": , 
                "T_l": np.array([0])
            }
            y_dict_left_bldc_motor_driver  = self.left_bldc_motor_driver(dt, 1, u_dict_left_bldc_motor_driver)
            y_dict_right_bldc_motor_driver = self.right_bldc_motor_driver(dt, 1, u_dict_right_bldc_motor_driver)

            w_m_left  = y_dict_left_bldc_motor_driver["w_m"][0]
            w_m_right = y_dict_right_bldc_motor_driver["w_m"][0]