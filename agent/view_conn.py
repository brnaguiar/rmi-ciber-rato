import matplotlib.pyplot as plt
import numpy
import matplotlib.animation as animation
import signal

def handler(signum, frame):
    print("exiting...")

signal.signal(signal.SIGINT, handler)

connections = numpy.load("connections.npz")['connections']

#print(connections)
new_conn = connections.reshape(connections.shape[0]*connections.shape[1], connections.shape[1])
x = new_conn[:, 0]
y = new_conn[:, 1]

for i in range(0, len(x), 2):
    plt.tight_layout()
    plt.plot(x[i:i+2], y[i:i+2], 'ro-')
    print("[{}, {}], [{}, {}]".format(x[i], y[i], x[i+1], y[i+1]))

def animate(i):
    connections = numpy.load("connections.npz")['connections']
    #print(connections)
    new_conn = connections.reshape(connections.shape[0]*connections.shape[1], connections.shape[1])
    x = new_conn[-10:, 0]
    y = new_conn[-10:, 1] 
    #plt.cla()
    for i in range(0, len(x), 2):
        plt.tight_layout()
        plt.plot(x[i:i+2], y[i:i+2], 'ro-')
        print("[{}, {}], [{}, {}]".format(x[i], y[i], x[i+1], y[i+1]))

ani = animation.FuncAnimation(plt.gcf(), animate, interval=50)

plt.tight_layout() 
plt.show() 


# def animate(i):
#     connections = numpy.load("connections.npz")['connections']
#     new_conn = connections.reshape(connections.shape[0]*connections.shape[1], connections.shape[1])
#     x = new_conn[:, 0]
#     y = new_conn[:, 1]
#     plt.cla()

        #plt.clear()

 


#x, y = connections[:, :, 0], connections[:, :, :, 1]

#print("Connections:")
#print(connections)

#print(x)






