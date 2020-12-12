import numpy as np
import json
import matplotlib.pyplot as plt


class BLDC_Motor:
    def __init__(self, motor_data):
        self.motor_data = motor_data
        
        # electrical
        electrical_dict = motor_data["electrical"]
        R   = electrical_dict["R"]
        L   = electrical_dict["L"]
        K_e = electrical_dict["K_e"]

        self.R = R
        self.L = L
        self.K_e = K_e

        # mechanical
        mechanical_dict = motor_data["mechanical"]
        J   = mechanical_dict["J"]
        K_t = mechanical_dict["K_t"]
        K_f = mechanical_dict["K_f"]
        p   = mechanical_dict["p"]

        self.J = J
        self.K_t = K_t
        self.p = p

        # initial conditions
        initial_dict = motor_data["initial"]
        x1 = initial_dict["i_a"]
        x2 = initial_dict["i_b"]
        x3 = initial_dict["w_m"]
        x4 = initial_dict["theta_m"]

        # state-space model
        a11 = -R/L
        a22 = -R/L
        a33 = -K_f/J
        a43 =  1
        self.A = np.matrix([[a11, 0, 0, 0], [0, a22, 0, 0], [0, 0, a33, 0], [0, 0, a43, 0]])
        
        b11 =  2/(3*L)
        b12 =  1/(3*L)
        b21 = -1/(3*L)
        b22 =  1/(3*L)
        b33 =  1/J
        # b33 =  p/J    # 06847897.pdf
        self.B = np.matrix([[b11, b12, 0], [b21, b22, 0], [0, 0, b33], [0, 0, 0]])
        self.C = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [-1, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.x = np.matrix([[x1], [x2], [x3], [x4]])
        

    def __get_electrical_position(self, theta_m):
        theta_e = (self.p/2)*theta_m
        return theta_e


    def __rem_2pi(self, theta_e):
        theta_e_norm = theta_e%(2*np.pi)
        return theta_e_norm


    def __F_function_kk(self, theta_e):
        theta_e_norm = self.__rem_2pi(theta_e)

        if           0 <= theta_e_norm and theta_e_norm < np.pi/6:
            output = (6/np.pi)*(theta_e_norm)
        elif   np.pi/6 <= theta_e_norm and theta_e_norm < 5*np.pi/6:
            output = 1
        elif 5*np.pi/6 <= theta_e_norm and theta_e_norm < 7*np.pi/6:
            output = 1 - (6/np.pi)*(theta_e_norm - 5*np.pi/6)
        elif 7*np.pi/6 <= theta_e_norm and theta_e_norm <= 11*np.pi/6:
            output = -1
        elif 11*np.pi/6 <= theta_e_norm and theta_e_norm <= 2*np.pi:
            output = -1 + (6/np.pi)*(theta_e_norm - 11*np.pi/6)
        else:
            print("Unexpected angle value:", theta_e_norm)
        return output


    def __controller_kk(self, theta_e):
        theta_e_norm = self.__rem_2pi(theta_e)

        if            0 <= theta_e_norm and theta_e_norm <=    np.pi/6:   # sector 6
            i_abc = np.array([0, -1, 1])
            H_123 = np.array([0, 0, 1])
        elif    np.pi/6 <= theta_e_norm and theta_e_norm <=  3*np.pi/6:   # sector 1
            i_abc = np.array([1, -1, 0])
            H_123 = np.array([1, 0, 1])
        elif  3*np.pi/6 <= theta_e_norm and theta_e_norm <=  5*np.pi/6:   # sector 2
            i_abc = np.array([1, 0, -1])
            H_123 = np.array([1, 0, 0])
        elif  5*np.pi/6 <= theta_e_norm and theta_e_norm <=  7*np.pi/6:   # sector 3
            i_abc = np.array([0, 1, -1])
            H_123 = np.array([1, 1, 0])
        elif  7*np.pi/6 <= theta_e_norm and theta_e_norm <=  9*np.pi/6:   # sector 4
            i_abc = np.array([-1, 1, 0])
            H_123 = np.array([0, 1, 0])
        elif  9*np.pi/6 <= theta_e_norm and theta_e_norm <= 11*np.pi/6:   # sector 5
            i_abc = np.array([-1, 0, 1])
            H_123 = np.array([0, 1, 1])
        elif 11*np.pi/6 <= theta_e_norm and theta_e_norm <= 12*np.pi/6:   # sector 6
            i_abc = np.array([0, -1, 1])
            H_123 = np.array([0, 0, 1])

        return i_abc, H_123



    def get_sector(self, theta_e_norm):
        sector = 1

        if            0 <= theta_e_norm and theta_e_norm <=    np.pi/6:   # sector 6
            sector = 6
        elif    np.pi/6 <= theta_e_norm and theta_e_norm <=  3*np.pi/6:   # sector 1
            sector = 1
        elif  3*np.pi/6 <= theta_e_norm and theta_e_norm <=  5*np.pi/6:   # sector 2
            sector = 2
        elif  5*np.pi/6 <= theta_e_norm and theta_e_norm <=  7*np.pi/6:   # sector 3
            sector = 3
        elif  7*np.pi/6 <= theta_e_norm and theta_e_norm <=  9*np.pi/6:   # sector 4
            sector = 4
        elif  9*np.pi/6 <= theta_e_norm and theta_e_norm <= 11*np.pi/6:   # sector 5
            sector = 5
        elif 11*np.pi/6 <= theta_e_norm and theta_e_norm <= 12*np.pi/6:   # sector 6
            sector = 6
        
        return sector


    def __inverter_function_kk(self, theta_e, v_s):
        theta_e_norm = self.__rem_2pi(theta_e)

        if           0 <= theta_e_norm and theta_e_norm <= np.pi/6:
            v_a =  0
            v_b = -v_s/2
            v_c =  v_s/2
        elif   np.pi/6 <= theta_e_norm and theta_e_norm <= 3*np.pi/6:
            v_a =  v_s/2
            v_b = -v_s/2
            v_c =  0
        elif 3*np.pi/6 <= theta_e_norm and theta_e_norm <= 5*np.pi/6:
            v_a =  v_s/2
            v_b =  0
            v_c = -v_s/2 
        elif 5*np.pi/6 <= theta_e_norm and theta_e_norm <= 7*np.pi/6:
            v_a = 0
            v_b =  v_s/2
            v_c = -v_s/2
        elif 7*np.pi/6 <= theta_e_norm and theta_e_norm <= 9*np.pi/6:
            v_a = -v_s/2
            v_b =  v_s/2
            v_c = 0
        elif 9*np.pi/6 <= theta_e_norm and theta_e_norm <= 11*np.pi/6:
            v_a = -v_s/2
            v_b = 0
            v_c = v_s/2
        elif 11*np.pi/6 <= theta_e_norm and theta_e_norm <= 12*np.pi/6:
            v_a =  0
            v_b = -v_s/2
            v_c =  v_s/2

        return v_a, v_b, v_c


    def refresh(self, motor_data):
        # initial conditions
        initial_dict = motor_data["initial"]
        x1 = initial_dict["i_a"]
        x2 = initial_dict["i_b"]
        x3 = initial_dict["w_m"]
        x4 = initial_dict["theta_m"]
        self.x = np.matrix([[x1], [x2], [x3], [x4]])


    def simulation_euler(self, dt, iterations, u_dict):
        # Prepare arrays for signals

        # Windings currents
        i_a = np.zeros(iterations)
        i_b = np.zeros(iterations)
        i_c = np.zeros(iterations)

        # Windings phase voltages
        v_a = np.zeros(iterations)
        v_b = np.zeros(iterations)
        v_c = np.zeros(iterations)

        # Windings phase to phase voltages
        v_ab = np.zeros(iterations)
        v_bc = np.zeros(iterations)
        v_ca = np.zeros(iterations)
        
        # Windings bemfs
        e_a = np.zeros(iterations)
        e_b = np.zeros(iterations)
        e_c = np.zeros(iterations)

        # Ideal Current waveforms
        iH_a = np.zeros(iterations)     # \ 
        iH_b = np.zeros(iterations)     # - Not so ideal xd
        iH_c = np.zeros(iterations)     # / 

        # Hall sensors
        H_1 = np.zeros(iterations)
        H_2 = np.zeros(iterations)
        H_3 = np.zeros(iterations)

        w_m = np.zeros(iterations)
        theta_m = np.zeros(iterations)
        theta_e = np.zeros(iterations)
        T_e = np.zeros(iterations)

        v_ab_sub_e_ab = np.zeros(iterations)
        v_bc_sub_e_bc = np.zeros(iterations)

        # Simulation
        for i in range(iterations):
            # if (i%10000 == 0):
            #     print(i, iterations)

            y = self.C*self.x
            
            # Unpack output signals
            i_a[i]     = np.array(y)[0][0]
            i_b[i]     = np.array(y)[1][0]
            i_c[i]     = np.array(y)[2][0]
            w_m[i]     = np.array(y)[3][0]
            theta_m[i] = np.array(y)[4][0]

            # Get value of theta_m such that it is in range <0, 2pi>
            theta_m[i] = self.__rem_2pi(theta_m[i])
            theta_e[i] = self.__get_electrical_position(theta_m[i])

            # Hall sensors
            iH_abc, H_123 = self.__controller_kk(theta_e[i])
            iH_a[i] = iH_abc[0]
            iH_b[i] = iH_abc[1]
            iH_c[i] = iH_abc[2]
            H_1[i] = H_123[0]
            H_2[i] = H_123[1]
            H_3[i] = H_123[2]

            # Trapeze abc
            induced_voltage_a = self.__F_function_kk(theta_e[i])
            induced_voltage_b = self.__F_function_kk(theta_e[i] - 2*np.pi/3)
            induced_voltage_c = self.__F_function_kk(theta_e[i] - 4*np.pi/3)

            # Back emfs
            e_a[i] = self.K_e*w_m[i]*induced_voltage_a
            e_b[i] = self.K_e*w_m[i]*induced_voltage_b
            e_c[i] = self.K_e*w_m[i]*induced_voltage_c

            # Electrical torque
            T_e[i]  = (self.K_t)*((induced_voltage_a*i_a[i]) + (induced_voltage_b*i_b[i]) + (induced_voltage_c*i_c[i]))

            # Inverter function
            v_a[i], v_b[i], v_c[i] = self.__inverter_function_kk(theta_e[i], u_dict["v_s"][i])
            v_ab[i] = v_a[i] - v_b[i]
            v_bc[i] = v_b[i] - v_c[i]
            v_ca[i] = v_c[i] - v_a[i]

            v_ab_sub_e_ab[i] = e_a[i] - e_b[i]
            v_bc_sub_e_bc[i] = e_b[i] - e_c[i]

            # Euler
            u11 = v_ab[i] - v_ab_sub_e_ab[i]
            u21 = v_bc[i] - v_bc_sub_e_bc[i]

            u31 = T_e[i]-u_dict["T_l"][i]
            u = np.matrix([[u11], [u21], [u31]])

            self.x = self.x + dt*(self.A*self.x + self.B*u)


        simulation_output = {
            "i_a": i_a,                         # \
            "i_b": i_b,                         # - plot
            "i_c": i_c,                         # /
            "e_a": e_a,                         # \
            "e_b": e_b,                         # - plot 
            "e_c": e_c,                         # /
            "v_ab": v_ab,
            "v_bc": v_bc,
            "v_ca": v_ca,
            "v_a": v_a,                         # \
            "v_b": v_b,                         # - plot
            "v_c": v_c,                         # /
            "w_m": w_m,
            "theta_m": theta_m,                 # - plot
            "theta_e": theta_e,                 # - plot
            "v_ab_sub_e_ab": v_ab_sub_e_ab,
            "v_bc_sub_e_bc": v_bc_sub_e_bc,
            "T_e": T_e,                         # - plot
            "iH_a": iH_a,
            "iH_b": iH_b,
            "iH_c": iH_c,
            "H_1": H_1,                         # \
            "H_2": H_2,                         # - plot
            "H_3": H_3                          # /
        }

        return simulation_output


    def check_back_emf_plot(self):
        xticks        = [0, np.pi/6, 3*np.pi/6, 5*np.pi/6, 7*np.pi/6, 9*np.pi/6, 11*np.pi/6, 12*np.pi/6]
        xticks_labels = ['0', 'π/6', '3π/6', '5π/6', '7π/6', '9π/6', '11π/6', '2π']
        yticks        = [-1, 0, 1]
        yticks_labels = ["-ke*ωm", "0", "ke*ωm"]

        num = 10000
        theta_e = np.linspace(0, 2*np.pi, num)
        e_a = np.zeros(num)
        e_b = np.zeros(num)
        e_c = np.zeros(num)
        
        for i in range(len(theta_e)):
            e_a[i] = self.__F_function_kk(theta_e[i])
            e_b[i] = self.__F_function_kk(theta_e[i] - 2*np.pi/3)
            e_c[i] = self.__F_function_kk(theta_e[i] - 4*np.pi/3)

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

        axes    = [ax1, ax2, ax3]
        ylabels = ["ea[V]", "eb[V]", "ec[V]"]
        es = [e_a, e_b, e_c]

        for i in range(len(axes)):
            axes[i].plot(theta_e, es[i])
            axes[i].set_xlabel('θe[rad]')
            axes[i].set_ylabel(ylabels[i])
            axes[i].set_xlim([theta_e[0], theta_e[-1]])
            axes[i].minorticks_on()
            axes[i].grid(b=True, which='both')
            axes[i].set_xticks(xticks)
            axes[i].set_xticklabels(xticks_labels)
            axes[i].set_yticks(yticks)
            axes[i].set_yticklabels(yticks_labels)
        plt.show()


    def check_back_controller(self, which=False):
        xticks        = [0, np.pi/6, 3*np.pi/6, 5*np.pi/6, 7*np.pi/6, 9*np.pi/6, 11*np.pi/6, 12*np.pi/6]
        xticks_labels = ['0', 'π/6', '3π/6', '5π/6', '7π/6', '9π/6', '11π/6', '2π']

        num = 10000
        theta_e = np.linspace(0, 2*np.pi, num)
        i_a = np.zeros(num)
        i_b = np.zeros(num)
        i_c = np.zeros(num)
        H_1 = np.zeros(num)
        H_2 = np.zeros(num)
        H_3 = np.zeros(num)
        
        for i in range(len(theta_e)):
            i_abc, H_123 = self.__controller_kk(theta_e[i])
            i_a[i] = i_abc[0]
            i_b[i] = i_abc[1]
            i_c[i] = i_abc[2]
            H_1[i] = H_123[0]
            H_2[i] = H_123[1]
            H_3[i] = H_123[2]

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

        axes    = [ax1, ax2, ax3]
        ylabels = ["Fa(theta_e)", "Fb(theta_e)", "Fc(theta_e)"]
        is_     = [i_a, i_b, i_c]
        halls   = [H_1, H_2, H_3]

        for i in range(len(axes)):
            axes[i].plot(theta_e, is_[i])
            axes[i].plot(theta_e, halls[i])
            axes[i].set_xlabel('θe[rad]')
            axes[i].set_ylabel(ylabels[i])
            axes[i].set_xlim([theta_e[0], theta_e[-1]])
            axes[i].minorticks_on()
            axes[i].grid(b=True, which='both')
            axes[i].set_xticks(xticks)
            axes[i].set_xticklabels(xticks_labels)
        plt.show()


    def check_inverter_function_plot(self, which=False):
        xticks        = [0, np.pi/6, 3*np.pi/6, 5*np.pi/6, 7*np.pi/6, 9*np.pi/6, 11*np.pi/6, 12*np.pi/6]
        xticks_labels = ['0', 'π/6', '3π/6', '5π/6', '7π/6', '9π/6', '11π/6', '2π']
        yticks        = [-6, 0, 6]
        yticks_labels = ["0.5Vs", "0", "0.5Vs"]

        num = 10000
        v_s = 12
        theta_e = np.linspace(0, 2*np.pi, num)
        v_a = np.zeros(num)
        v_b = np.zeros(num)
        v_c = np.zeros(num)
        
        for i in range(len(theta_e)):
            v_a[i], v_b[i], v_c[i] = self.__inverter_function_kk(theta_e[i], v_s)

        fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

        axes             = [ax1, ax2, ax3]
        ylabels          = ["Va[V]", "Vb[V]", "Vc[V]"]
        induced_voltages = [v_a, v_b, v_c]

        for i in range(len(axes)):
            axes[i].plot(theta_e, induced_voltages[i])
            axes[i].set_xlabel('θe[rad]')
            axes[i].set_ylabel(ylabels[i])
            axes[i].set_xlim([theta_e[0], theta_e[-1]])
            axes[i].minorticks_on()
            axes[i].grid(b=True, which='both')
            axes[i].set_xticks(xticks)
            axes[i].set_xticklabels(xticks_labels)
            axes[i].set_yticks(yticks)
            axes[i].set_yticklabels(yticks_labels)
        plt.show()


if __name__ == "__main__":
    motor_data = json.load(open("./bldc_motor_data_kk_data.json"))
    bldc_motor = BLDC_Motor(motor_data)
    bldc_motor.check_back_emf_plot()
    bldc_motor.check_inverter_function_plot()
    # exit()

    t_end = 100
    dt = 0.01
    iterations = int(t_end/dt)

    t = np.ones(iterations)
    t = t*dt
    t = np.cumsum(t)
    t = t-dt

    v_s = 12*np.ones(iterations)
    T_l = 0*np.ones(iterations)
    u_dict = {
        "v_s": v_s,
        "T_l": T_l
        }

    simulation_output = bldc_motor.simulation_euler(dt, iterations, u_dict)

    # plt.plot(t, v_s, label="Vs[V]")
    plt.plot(t, simulation_output["T_e"], label="ωm[rad/s]")
    plt.xlim([t[0], t[-1]])
    plt.xlabel("t[s]")
    plt.legend(loc="upper right")
    plt.minorticks_on()
    plt.grid(which='both')
    plt.show()


