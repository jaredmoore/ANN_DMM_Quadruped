"""
    Contains the classes MNode and MNodeNet necessary for constructing a muscle node network
    for joint actuation.
"""
import math
import random

from gauss_node import GaussianNode

class MNode(object):
    """ Simulate an individual muscle node in the network.

    Provides a wrapper to hold some distribution or signal function for a node
    to activate based on an input signal ranging between -1 and 1.

    Attributes:
    node: the node activation function 
    deg: the position of the node on a unit circle
    x: x coordinate of the node on a unit circle
    y: y coordinate of the node on a unit circle
    """

    def __init__(self,**kwargs):#deg,nodetype=None,params=None):
        """ Initialize the muscle node. 
        
        Args:
            kwargs: dictionary of provided keywords.
            Necessary Keywords:
                deg: degree location of the node on the unit circle
            Optional Keywords:
                params: parameters to apply to the gaussian node
                nodetype: type of node oscillation distribution
        """
        if('nodetype' in kwargs):
            pass
        else:
            if('params' in kwargs):
                self.node = GaussianNode(kwargs['params'])
            else:
                self.node = GaussianNode()
            self.deg = kwargs['deg']
            self.x = float("{0:.4f}".format(math.cos(math.radians(self.deg))))
            self.y = float("{0:.4f}".format(math.sin(math.radians(self.deg))))
  
    def copy(self):
        return MNode(deg=self.deg,params=self.node.get_params())

    def __str__(self):
        return "<MNode deg=\'"+str(self.deg)+"\' "+\
                "x=\'"+str(self.x)+"\' y=\'"+str(self.y)+"\'>"+str(self.node)+\
                "</MNode>\n"

    def get_raw_activation(self,x):
        """ Get the raw activation for the node at point x. """
        return self.node.get_activation(x)

    def get_activation(self,x):
        """ Get the activation by axes for the node at point x. """
        act = self.node.get_activation(x)
        return act*self.x, act*self.y 
    
    def mutate(self,position=None,mut_perc=0):
        """ Mutate the node parameters. 
        
        If a mutation percentage is given, go through each parameter and 
        mutate if it's triggered.  Otherwise, simply call mutate().

        Args:
            position: mutate the position of the nodes as well.
            mut_perc: chance of mutation
        """
        if mut_perc:
            if(position):
                if(random.random() < mut_perc):
                    # Mutate the position of a node.
                    # Std. Dev of 10. allows for a total degree change of +- 30 degrees
                    self.deg = float("{0:.4f}".format(random.gauss(self.deg, 10.)))
                    if(self.deg < 0.):
                        self.deg += 360.
                    self.x = float("{0:.4f}".format(math.cos(math.radians(self.deg))))
                    self.y = float("{0:.4f}".format(math.sin(math.radians(self.deg))))
            self.node.mutate(mut_perc=mut_perc)
        else:
            if(position):
                if(random.randint(0,100) < 10):
                    # Mutate the position of a node.
                    # Std. Dev of 10. allows for a total degree change of +- 30 degrees
                    self.deg = float("{0:.4f}".format(random.gauss(self.deg, 10.)))
                    if(self.deg < 0.):
                        self.deg += 360.
                    self.x = float("{0:.4f}".format(math.cos(math.radians(self.deg))))
                    self.y = float("{0:.4f}".format(math.sin(math.radians(self.deg))))
                else:
                    self.node.mutate()
            else:
                # Mutate only the nodes.
                self.node.mutate()
