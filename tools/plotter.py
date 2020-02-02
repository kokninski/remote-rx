import serial
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/ttyUSB0')
print(ser.name)

plt.plot(0,0)
plt.xlim(0,4096)
plt.ylim(0,4096)
try:
    while True:
        line = ser.readline().decode("utf-8")
        try:
            dt, ch1_x, ch1_y, ch2_x, ch2_y = line.split(',')
        except ValueError:
            pass
        else:
            dt = int(dt)
            ch1_x = int(ch1_x)
            ch1_y = int(ch1_y)
            ch2_x = int(ch2_x)
            ch2_y = int(ch2_y)

            print(f"t: {dt}, x: {ch1_x} y: {ch1_y}, x: {ch2_x} y: {ch2_y}")
            plt.cla()
            plt.scatter(ch1_x, ch1_y)
            plt.xlim(0,4096)
            plt.ylim(0,4096)
            # plt.line(dt)
            plt.pause(0.05)
except:
    print("Exiting")
    raise
finally:
    ser.close()