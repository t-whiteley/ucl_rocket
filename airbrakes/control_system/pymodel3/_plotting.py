import matplotlib.pyplot as plt

class Plot():
    def __init__(self):
        self.t_axis = []

        self.a_real = []
        self.v_real = []
        self.s_real = []

        self.a_fil = []
        self.v_fil = []
        self.s_fil = []

        self.ap_pred = []
        self.error = []
        self.servo_sig = []
        self.area_adj = []
    
    def append(self, t, a_r, v_r, s_r, a_fil, v_fil, s_fil, ap_pred, err, servo_sig):
        self.t_axis.append(t)
        self.a_real.append(a_r)
        self.v_real.append(v_r)
        self.s_real.append(s_r)
        self.a_fil.append(a_fil)
        self.v_fil.append(v_fil)
        self.s_fil.append(s_fil)
        self.ap_pred.append(ap_pred)
        self.error.append(err)
        self.servo_sig.append(servo_sig)
        # self.area_adj.append(area_adj)
    
    def plot(self):
        # Create a 2x2 grid for plots
        fig, axs = plt.subplots(2, 2, figsize=(10, 7))

        # Plot for Acceleration
        axs[0, 0].plot(self.t_axis, self.a_real, label="a_real")
        axs[0, 0].plot(self.t_axis, self.a_fil, label="a_fil")
        axs[0, 0].axhline(y=0, color='black')
        axs[0, 0].legend()
        axs[0, 0].set_xlabel("Time (s)")
        axs[0, 0].set_ylabel("Acceleration")
        axs[0, 0].set_title("Acceleration Curves")

        # Plot for Height
        axs[0, 1].plot(self.t_axis, self.s_real, label="s_real", color='black')
        axs[0, 1].plot(self.t_axis, self.s_fil, label="s_fil", color='green')
        axs[0, 1].plot(self.t_axis, self.ap_pred, label="ap_pred", color='purple')
        axs[0, 1].axhline(y=max(self.s_real), linestyle='dotted', color='black', label=f'ap_real {round(max(self.s_real))}')
        axs[0, 1].axhline(y=max(self.s_fil), linestyle='dotted', color='green', label=f'ap_fil {round(max(self.s_fil))}')
        axs[0, 1].axhline(y=0, color='black')
        axs[0, 1].legend()
        axs[0, 1].set_xlabel("Time (s)")
        axs[0, 1].set_ylabel("Height")
        axs[0, 1].set_title("Height Curves")

        # Plot for Servo
        axs[1, 1].plot(self.t_axis, self.servo_sig, label="servo_signal")
        axs[1, 1].axhline(y=0, color='black')
        axs[1, 1].legend()
        axs[1, 1].set_xlabel("Time (s)")
        axs[1, 1].set_ylabel("Servo Signal")
        axs[1, 1].set_title("Servo Signal")

        # Plot for Speed
        axs[1, 0].plot(self.t_axis, self.v_fil, label="v_fil")
        axs[1, 0].plot(self.t_axis, self.v_real, label="v_real")
        axs[1, 0].axhline(y=0, color='black')
        axs[1, 0].legend()
        axs[1, 0].set_xlabel("Time (s)")
        axs[1, 0].set_ylabel("Volts")
        axs[1, 0].set_title("Speed Curves")

        # Adjust layout
        plt.subplots_adjust(hspace=0.4, wspace=0.3)

        plt.tight_layout()
        plt.show()