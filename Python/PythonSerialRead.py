import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

# style.use('fivethirtyeight')


def clean(L):
    newln = []
    for i in range(len(L)):
        temp = L[i][2:]
        newln.append(temp[:-5])
    return newln


def write(L):
    nm = 0
    file = open("data.txt", mode='w')
    for i in range(len(L)):
        file.write(L[i] + '|')
        file.write(str(nm))
        file.write('\n')
        nm = nm + 1
    file.close


def plot():
    for line in open("data.txt", mode='r'):
        values = [int(s) for s in line.split('|')]
        X.append(values[0])
        Y.append(values[1])
    print(X)
    print(Y)
    plt.title("Serial Read")
    plt.grid(1)
    plt.xlabel('Angle [°]')
    plt.ylabel('Anal. input [del] °')
#    plt.clear()
    plt.plot(Y, X)
    plt.show()


def animate(i):
    graph_data = open("data.txt", mode='r').read()
    lines = graph_data.split('|')
    xs = []
    ys = []
    for line in lines:
        if len(line) > 1:
            x, y = line.split(',')
            xs.append(x)
            ys.append(y)
    # ax1.clear()
    ax1.plot(Y. X)
    plt.show()


try:
    ser = serial.Serial("COM5", 115200)
    # ser.open()
except:
    print("Preveri USB in COM port")


fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)


X, Y = [], []
count = 0
rawdata = []
nwdata = []
# ser.isOpen() &&
i = 0
while(i < 2):
    while(count < 1024):
        x = ser.readline()
        # print(x)
        rawdata.append(str(x))
        count = count + 1

    # print(rawdata)
    nwdata = clean(rawdata)
    # print(nwdata)
    write(nwdata)
    while(count < 1024):
        x = ser.readline()
        # print(x)
        rawdata.append(str(x))
        count = count + 1

    # print(rawdata)
    nwdata = clean(rawdata)
    # print(nwdata)
    write(nwdata)

    #ani = animation.FuncAnimation(fig, animate, interval=1000)
    # plt.show
    i = i+1
plot()
ser.close()

# dejan
