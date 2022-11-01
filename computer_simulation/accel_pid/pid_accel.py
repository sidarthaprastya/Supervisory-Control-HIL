import matplotlib.pyplot as plt
from simple_pid import PID
import time

#motor_laplace is actually a velocity function
def motor(input, output): 
    output[1] = output[0]
    # Insert to Laplace equation (10/s+10) with time samp = 10 ms
    # output[0] = 0.0476 * input[0] + 0.0476 * input[1] + 0.9048 * output[1]
    output[0] = 0.00413 * input[0] + 0.00413 * input[1] + 0.992 * output[1]
    return output

def pid(input, kp, ki, kd, setpoint, time_samp):
    global integral, last_error
    error = setpoint - input
    integral += error * time_samp
    deriv = (last_error - error) / time_samp
    last_error = error
    return (kp * error + ki * integral + kd * deriv)

KP = 0.8
KI = 0.1
KD = 0.0

setpoint = 2
time_samp = 0.01   # 10 ms
last_error = 0
integral = 0

curr_accel = 0

x = [0, 0]
y = [0, 0]
plt.axis([0, 100, -2, 2])
x_axis = []
ydata = []
yin = []

for i in range(100):
    x[1] = x[0]
    if (i < 20 or i > 70):
        setpoint = 0
    else:
        setpoint = 1
    
    x[0] = pid(curr_accel, KP, KI, KD, setpoint, time_samp)

    # Add integrator to change the equation from vel to accel
    x[0] += x[1]
    
    y = motor(x, y)
    curr_accel = (y[0] - y[1])/0.01
    x_axis.append(i)
    ydata.append(curr_accel)
    yin.append(setpoint)

  
plt.plot(x_axis, ydata, label="Accel")
plt.plot(x_axis, yin, label="Setpoint")
plt.title("PID Acceleration")
plt.xlabel("Time (x10 ms)")
plt.ylabel("Accel (m/s^2)")
plt.legend()
# plt.show()
plt.savefig("accel.jpg")

    
