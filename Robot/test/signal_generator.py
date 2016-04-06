import math
import matplotlib.pyplot as plt
import random

def signal_generator():
    """ Generate a periodic signal from the specified parameters. """

    threshold = 0.02

    amp_1 = random.random()
    amp_2 = random.random()
    frq_1 = random.random()*2.*math.pi
    frq_2 = random.random()*2.*math.pi
    ph_1 = random.random()*2.*math.pi
    ph_2 = random.random()*2.*math.pi

    tick = 0
    ts = 0.01

    val = (amp_1*math.sin(frq_1*tick+ph_1)+amp_2*math.cos(frq_2*tick+ph_2))/(amp_1+amp_2) 
    while(val > threshold or val < -threshold):
        val = (amp_1*math.sin(frq_1*tick+ph_1)+amp_2*math.cos(frq_2*tick+ph_2))/(amp_1+amp_2) 
        tick += ts
    yield val

    while(1):
        yield (amp_1*math.sin(frq_1*tick+ph_1)+amp_2*math.cos(frq_2*tick+ph_2))/(amp_1+amp_2)
        tick += ts

for i in xrange(20):
    x = []
    y = []
    for j,k in zip(xrange(1000),signal_generator()):
        x.append(j)
        y.append(k)

    plt.figure(1)
    plt.plot(x,y)

    plt.ylabel("Signal")
    plt.xlabel("Step")
    plt.ylim([-1.0,1.0])
    plt.show()

