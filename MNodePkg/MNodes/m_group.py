"""
    Implements a muscle group for one joint.
"""

import math
import random

from mus_nodes import MNode

class MuscleGroup(object):
    """
        A group of MNodes for actuating one joint.

        Attributes:
        nodes: list of tuples [node, deg, x, y]
            node: MNode object
            degree: position of the node on the unit circle
            x: x coordinate on unit circle
            y: y coordinate on unit circle
    """

    def __init__(self, num_nodes=6, params=[]):
        """ Create a group of MNodes to actuate a joint.

        """
        self.nodes = []
        
        if(num_nodes > 0):
            offset = 360./num_nodes/2. # Offset the nodes so they don't start at 0.
            for i in range(num_nodes):
                deg = i*(360./num_nodes)+offset
                if params:
                    self.nodes.append(MNode(deg=deg,params=params[i]))
                else:
                    self.nodes.append(MNode(deg=deg))
    
    def __str__(self):
        ret = "<MuscleGroup>\n"
        for node in self.nodes:
            ret += str(node)
        ret += "</MuscleGroup>\n"
        return ret

    def copy(self):
        mg = MuscleGroup(num_nodes=0)
        for n in self.nodes:
            n_n = n.copy()
            mg.nodes.append(n_n)
        return mg

    def add_node(self,node):
        """ Add a node to the muscle group. 
        
        Arguments:
        node: node to copy settings from.
        
        Returns:
        newly constructed node
        """
        self.nodes.append(MNode(deg=node.deg,params=node.node.get_params()))

    def create_node(self,**kwargs):
        """ Create a node from the specified parameters.

        Args:
            kwargs: dictionary holding the node parameters
            Contains:
                deg: degree for the node
                params: list of parameters for the node
        """
        self.nodes.append(MNode(deg=kwargs['deg'],params=kwargs['params']))

    def mutate(self,position=None,mut_perc=0):
        """ Mutate some facet of the network.

        Mutate a randomly selected node in the network unless there is a
        specified mutation percentage.  In that case, we want to then
        go through each parameter and decide whether to mutate it or not.

        Attributes:
        position: Decide whether to mutate the position of nodes.
        mut_perc: Percent chance of mutation.

        TODO: Need to decide the right combination of parameters for mutation.

        Right now mutation is given a weighted power.  90% of the time, we will mutate the 
        actual node. 
        """
        if mut_perc:
            for node in self.nodes:
                node.mutate(mut_perc=mut_perc,position=position)
        else:
            self.nodes[random.randint(0,len(self.nodes)-1)].mutate(position=position)

    def get_raw_activations(self,x):
        """ Get the single activation value for each node in the network. 
        
        Arguments:
        x: input value for the activation
        """
        return [n.get_raw_activation(x) for n in self.nodes]

    def get_activations(self,x):
        """ Get the activations by axis for each node in the network. 
        
        Arguments:
        x: input value for the activation
        """
        x_acts = []
        y_acts = []

        for n in self.nodes:
            ax1, ax2 = n.get_activation(x)
            x_acts.append(ax1)
            y_acts.append(ax2)

        return x_acts, y_acts 

    def get_avg_activations(self,inp):
        """ Get the activations by axis distilled into one value per axis.

        Args:
        inp: input value to query

        Returns:
        x,y: tuple detailing the two different desired positions for the joints.
        """

        x_pos = 0
        y_pos = 0

        for n in self.nodes:
            ax1, ax2 = n.get_activation(inp)
            x_pos += ax1
            y_pos += ax2

        # Change in version 2!
        if x_pos < -1:
            x_pos = -1
        elif x_pos > 1:
            x_pos = 1
            
        if y_pos < -1:
            y_pos = -1
        elif y_pos > 1:
            y_pos = 1

        # Candidate Change for version 3
        if(math.sqrt(x_pos**2+y_pos**2) > 1.):
            hyp = math.sqrt(x_pos**2+y_pos**2)
            ang_y = math.asin((y_pos*math.sin(math.radians(90)))/hyp)
            ang_x = math.radians(90.)-ang_y
            o_x_pos = x_pos
            o_y_pos = y_pos
            x_pos = math.sin(ang_x)/math.sin(math.radians(90))
            y_pos = math.sin(ang_y)/math.sin(math.radians(90))

            # Ensure the sign stays the same.
            if (x_pos > 0 and o_x_pos < 0) or (o_x_pos > 0 and x_pos < 0):
                x_pos *= -1
            if (y_pos > 0 and o_y_pos < 0) or (o_y_pos > 0 and y_pos < 0):
                y_pos *= -1

        # Removed in version 2!
        # x_pos /= len(self.nodes)
        # y_pos /= len(self.nodes)
        
        return x_pos, y_pos

    def get_avg_activations_ind_mus_nodes(self,inp):
        """ Get the activations by axis distilled into one value per axis with passing the inputs to individual muscle nodes.

        Args:
        inp: input value to query

        Returns:
        x,y: tuple detailing the two different desired positions for the joints.
        """

        x_pos = 0
        y_pos = 0

        if len(self.nodes) != len(inp):
            print("Error: Number of nodes and number of inputs do not match up!")
            print(len(self.nodes),len(inp))
            exit()

        for n,i in zip(self.nodes,inp):
            ax1, ax2 = n.get_activation(i)
            x_pos += ax1
            y_pos += ax2

        # Change in version 2!
        if x_pos < -1:
            x_pos = -1
        elif x_pos > 1:
            x_pos = 1
            
        if y_pos < -1:
            y_pos = -1
        elif y_pos > 1:
            y_pos = 1

        # Candidate Change for version 3
        if(math.sqrt(x_pos**2+y_pos**2) > 1.):
            hyp = math.sqrt(x_pos**2+y_pos**2)
            ang_y = math.asin((y_pos*math.sin(math.radians(90)))/hyp)
            ang_x = math.radians(90.)-ang_y
            o_x_pos = x_pos
            o_y_pos = y_pos
            x_pos = math.sin(ang_x)/math.sin(math.radians(90))
            y_pos = math.sin(ang_y)/math.sin(math.radians(90))

            # Ensure the sign stays the same.
            if (x_pos > 0 and o_x_pos < 0) or (o_x_pos > 0 and x_pos < 0):
                x_pos *= -1
            if (y_pos > 0 and o_y_pos < 0) or (o_y_pos > 0 and y_pos < 0):
                y_pos *= -1

        # Removed in version 2!
        # x_pos /= len(self.nodes)
        # y_pos /= len(self.nodes)
        
        return x_pos, y_pos