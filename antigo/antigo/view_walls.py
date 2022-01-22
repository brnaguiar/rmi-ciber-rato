import matplotlib.pyplot as plt
import numpy
import matplotlib.animation as animation
import signal

def handler(signum, frame):
    print("exiting...")

signal.signal(signal.SIGINT, handler)

wall_s = numpy.load("connections.npz")['walls'] #connectionsconnections

#print(tree_s)

x = wall_s[:, 0]
y = wall_s[:, 1] 

print("ola: ", x)

for i in range(0, len(x)):
    plt.tight_layout()
    plt.plot(x[i], y[i], 'ro-')

def animate(i):
    wall_s = numpy.load("connections.npz")['walls'] #connectionsconnections
    #print(tree_s)
    x = wall_s[:, 0]
    y = wall_s[:, 1]
    #print("ola: ", x)
    plt.cla() 
    for i in range(0, len(x)):
        plt.tight_layout()
        plt.plot(x[i], y[i], 'ro-')
        #print("[{}, {}], [{}, {}]".format(x[i], y[i], x[i+1], y[i+1]))

    print( " " )    

ani = animation.FuncAnimation(plt.gcf(), animate, interval=50)      

print( " " ) 

plt.tight_layout()
plt.show()   