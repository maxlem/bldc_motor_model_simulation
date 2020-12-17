import speed_controller
import pi_controller
import json
import numpy as np
import matplotlib.pyplot as plt
import time


class Position_Controller:
    def __init__(self, position_controller_data):
        pi_controller_data = position_controller_data["pi_controller"]
        speed_controller_data = position_controller_data["speed_controller"]

        self.pi_controller    = pi_controller.PI_Controller(pi_controller_data)
        self.speed_controller = speed_controller.Speed_Controller(speed_controller_data)


    def simulation(self, dt, iterations, u_dict):
        # Prepare arrays for signals
        theta_m = np.zeros(iterations)
        w_ref   = np.zeros(iterations)
        w_m     = np.zeros(iterations)
        i_a     = np.zeros(iterations)
        i_b     = np.zeros(iterations)
        i_c     = np.zeros(iterations)
        T_e     = np.zeros(iterations)
        v_a     = np.zeros(iterations)
        v_b     = np.zeros(iterations)
        v_c     = np.zeros(iterations)
        v_s     = np.zeros(iterations)

        # Simulation main loop
        for i in range(iterations):
            if i%1000 == 0:
                print(str((i/iterations)*100)+"%")

            # Feedback loop sum block
            if i == 0:
                e = u_dict["theta_m_ref"][i]
            else:
                e = u_dict["theta_m_ref"][i] - theta_m[i-1]
                
            # PI controller block
            u_dict_pi_controller = {"e": np.array([e])}
            y_dict_pi_controller = self.pi_controller.simulation_euler_anti_windup(dt, 1, u_dict_pi_controller)
            w_ref[i] = y_dict_pi_controller["y"][0]

            
            # speed_controller
            u_dict_speed_controller = {"w_ref": np.array([w_ref[i]]), 
                                        "T_l": np.array([u_dict["T_l"][i]])}

            y_dict_speed_controller = self.speed_controller.simulation(dt, 1, u_dict_speed_controller)

            # Load data to arrays
            theta_m[i] = y_dict_speed_controller["theta_m"][0]
            w_m[i]     = y_dict_speed_controller["w_m"][0]
            i_a[i]     = y_dict_speed_controller["i_a"][0]
            i_b[i]     = y_dict_speed_controller["i_b"][0]
            i_c[i]     = y_dict_speed_controller["i_c"][0]
            v_a[i]     = y_dict_speed_controller["v_a"][0]
            v_b[i]     = y_dict_speed_controller["v_b"][0]
            v_c[i]     = y_dict_speed_controller["v_c"][0]
            T_e[i]     = y_dict_speed_controller["T_e"][0]

        # Prepare output dict
        y_dict = {
            "theta_m": theta_m,     
            "w_m": w_m,           
            "i_a": i_a,                         # \
            "i_b": i_b,                         # - plot
            "i_c": i_c,                         # /
            "v_a": v_a,                         # - plot
            "v_b": v_b,                         # - plot
            "v_c": v_c,                         # - plot
            "T_e": T_e,                         # - plot
            "V_s": v_s,                         # - plot
            "w_ref": w_ref
        }
        
        return y_dict




if __name__ == "__main__":
    # Simulation data
    simulation_data_path     = "./simulation_data.json"
    simulation_data          = json.load(open(simulation_data_path))
    position_controller_data = simulation_data["position_controller"]

    dt = 0.01
    iterations = 10000
    t = np.ones(iterations)
    t = t*dt
    t = np.cumsum(t)
    t = t-dt
    t = t

    # Simulation input
    u_dict = {
        "theta_m_ref" : np.append(10*np.ones(int(iterations/5)), 
                    np.append(50*np.ones(int(iterations/5)), 
                    np.append(30*np.ones(int(iterations/5)), 
                    np.append(60*np.ones(int(iterations/5)), 80*np.ones(int(iterations/5)))))),
        "T_l" : np.append(0.00*np.ones(int(iterations/4)), 
                    np.append(0.025*np.ones(int(iterations/4)), 
                    np.append(0.01*np.ones(int(iterations/4)), 
                    0.03*np.ones(int(iterations/4)))))
    }

    # Run simulation
    position_controller = Position_Controller(position_controller_data)
    y_dict = position_controller.simulation(dt, iterations, u_dict)

    # Plot position
    ylim_coeff = 1.2
    plt.plot(t, u_dict["theta_m_ref"], "-c", label="θ_ref[rad/s]")
    plt.plot(t, y_dict["theta_m"], "-k", label="θm[rad/s] Kp=50")
    plt.xlabel("t[s]")
    plt.ylabel("θ[rad]")
    plt.legend(loc="right")
    plt.grid(which="both")
    plt.minorticks_on()
    plt.show()

