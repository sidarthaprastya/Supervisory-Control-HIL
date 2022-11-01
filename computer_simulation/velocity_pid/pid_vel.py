import matplotlib.pyplot as plt
from simple_pid import PID
import time

#motor_laplace is actually a velocity function
def motor(input, output): 
    output[1] = output[0]
    output[0] = 0.00413 * input[0] + 0.00413 * input[1] + 0.992 * output[1]
    return output

def pid(input, kp, ki, kd, setpoint, time_samp):
    global integral, last_error
    error = setpoint - input
    integral += error * time_samp
    deriv = (last_error - error) / time_samp
    last_error = error
    return (kp * error + ki * integral + kd * deriv)

KP = 50.0
KI = 15.0
KD = 0.1
# KP = 10.0
# KI = 1.5
# KD = 0

setpoint = 0
time_samp = 0.01   # 10 ms
last_error = 0
integral = 0

x = [0, 0]
y = [0, 0]
plt.axis([0, 100, -2, 2])
x_axis = []
ydata = []
yin = []

for i in range (100):
    x[1] = x[0]
    # #-------------------------------
    # # Uncomment to select
    # #-------------------------------
    # # Unit Step
    # setpoint = 1
    # #-------------------------------
    # # Unit Ramp Up
    # if (i < 20):
    #     setpoint = 0
    # elif (i >= 20 and setpoint < 1):
    #     setpoint += 0.05
    # else:
    #     setpoint = 1
    # #--------------------------------
    # # Unit Ramp Down
    if (i < 20):
        setpoint = 0
    elif (i >= 20 and setpoint > -1):
        setpoint -= 0.05
    else:
        setpoint = -1
    # #---------------------------------

    x[0] = pid(y[0], KP, KI, KD, setpoint, time_samp)
    y = motor(x, y)
    x_axis.append(i)
    ydata.append(y[0])
    yin.append(setpoint)
    

plt.plot(x_axis, ydata, label="laplace")
plt.plot(x_axis, yin, label="input")
# plt.plot(tao_axis, tao_line)
plt.title("PID Velocity")
plt.xlabel("Time (x10 ms)")
plt.ylabel("Velocity (m/s)")
plt.legend()
# plt.show()
plt.savefig("velocity.jpg")
    
