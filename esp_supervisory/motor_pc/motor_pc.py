import serial
import time
import numpy as np
import matplotlib.pyplot as plt

def motor(input, output): 
    output[1] = output[0]
    output[0] = 0.00413 * input[0] + 0.00413 * input[1] + 0.992 * output[1]
    return output

xmin = 0
ser = serial.Serial('COM5', 115200, bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=None)
plt.axis([0, 1000, -3, 3])
xdata = []
ydata = []
posdata = []
acceldata = []
x = [0.0,0.0]
y = [0.0,0.0]
i = 0
posisi = 0.0
if ser.is_open:
    # Melakukan write awal dengan nilai 0 agar program karena mikon hanya dapat bekerja setelah menerima input
    ser.write(("%f;%f;%f\r\n" % (0.0, 0.0, 0.0)).encode("utf-8"))
    
    while (True):
        try:
            size = ser.inWaiting()
            if size:
                x[1] = x[0]
                [sel, x[0]] = [float(v) for v in (ser.readline().decode("utf-8").split(";"))]
                if(sel == 0.0):
                    x[0] += x[1]
                
                y = motor(x, y)

                posisi += y[0] * 0.01 
                accel = (y[0] - y[1]) / 0.01

                ser.write(("%f;%f;%f\r\n" % (y[0], posisi, accel)).encode("utf-8"))
                
                posdata.append(posisi)
                acceldata.append(accel)
                ydata.append(y[0])
                xdata.append(i)
                
                i += 1
                if (i % 10 == 0):
                    plt.clf()
                    if(i <= 1000):
                        plt.axis([0, 1000, -3, 3])
                    
                    else:
                        plt.axis([0, i, -3, 3])
                    
                    # Hanya melakukan plot kecepatan secara realtime agar tidak berat
                    plt.plot(xdata, ydata, label="Profil kecepatan")
                    plt.draw()
                    plt.pause(0.05)

        except Exception as e:
            plt.clf()
            plt.axis([0, i, -3, 3])
            plt.plot(xdata, ydata, label="Profil kecepatan")
            plt.plot(xdata, posdata, label="Profil posisi")
            plt.plot(xdata, acceldata, label="Profil Akselerasi")
            plt.legend()
            plt.savefig("hasil.jpg") # Menyimpan gambar lengkap
            print(e)
            break
    
    