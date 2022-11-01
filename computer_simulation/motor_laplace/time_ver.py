import matplotlib.pyplot as plt

def motor(input, output):
    output[1] = output[0]
    # Insert to Laplace equation (10/s+10) with time samp = 10 ms
    # output[0] = 0.0476 * input[0] + 0.0476 * input[1] + 0.9048 * output[1]
    output[0] = 0.00413 * input[0] + 0.00413 * input[1] + 0.992 * output[1]
    return output

x = [0, 0]
y = [0, 0]
plt.axis([0, 1300, 0, 1.2])
x_axis = []
ydata = []
yin = []

# for i in range(0, 10):
#     x[1] = x[0]
#     x[0] = 0
#     y = motor(x, y)
#     x_axis.append(i)
#     ydata.append(y[0])
#     yin.append(x[0])

for i in range(0, 1300):
    x[1] = x[0]
    x[0] = 1
    y = motor(x, y)
    x_axis.append(i)
    ydata.append(y[0])
    yin.append(x[0])

tao_line = [0, 1]
tao_axis = [120, 120]
plt.plot(x_axis, ydata, label="laplace")
plt.plot(x_axis, yin, label="input")
plt.plot(tao_axis, tao_line, label="tau=1.2s")
plt.legend()
plt.savefig("time_const.jpg")

    
