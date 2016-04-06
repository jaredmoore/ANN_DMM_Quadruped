"""
    Implement a guassian distribution for node activation level.
"""
import math
import random

class GaussianNode(object):
    """ Create a gaussian distribution based node activation.

    Attributes:
    a: maximum activation output level for a gaussian
    b: point at which the maximum output is reached
    c: spread of values in the gaussian, larger means wider
    """

    def __init__(self,params=None):
        """ Initalized the gaussian node.

        Attributes:
        params: list containing the a, b, and c parameters
        """
        if(params):
            self.a = params[0]
            self.b = params[1]
            self.c = params[2]
        else:
            self.a = random.randrange(0,100)/100.
            self.b = random.randrange(-125,125)/100.
            self.c = random.randrange(1,100)/100.

    def __str__(self):
        return "<GaussianNode a=\'"+str(self.a)+"\' b=\'"+str(self.b)+"\' c=\'"+str(self.c)+"\'/>"

    def copy(self):
        return GaussianNode(params=[self.a,self.b,self.c])

    def get_params(self):
        """ Return a list of the nodes params. """
        return [self.a,self.b,self.c]
    
    def get_activation(self,x):
        """ Get the activation at the given point.

        x: point to calculate for gaussian
        """

        return self.a * math.exp(-math.pow((x-self.b),2.)/(2.*math.pow(self.c,2.)))

    def __mutate_a(self,mut_sd):
        """ Mutate the a parameter.

        This should be a private method!
        """
        self.a = float("{0:.4f}".format(random.gauss(self.a, mut_sd)))
        if(self.a > 1.):
            self.a = 1.
        elif(self.a < 0.):
            self.a = 0.

    def __mutate_b(self,mut_sd):
        """ Mutate the b parameter.

        This should be a private method!
        """
        self.b = float("{0:.4f}".format(random.gauss(self.b, mut_sd)))
        if(self.b > 1.):
            self.b = 1.
        elif(self.b < -1.):
            self.b = -1.

    def __mutate_c(self,mut_sd):
        """ Mutate the c parameter.

        This should be a private method!
        """
        self.c = float("{0:.4f}".format(random.gauss(self.c, mut_sd)))
        if(self.c > 1.):
            self.c = 1.
        elif(self.c < 0.1):
            self.c = 0.1

    def mutate(self,mut_sd=0.05,mut_perc=0):
        """ Mutate one of the attributes in the distribution. 
        
        Using a std. dev. of 0.05 for gaussian mutation yields a possible range of
        mutation of 0.15.

        If mut_perc is given, go through each parameter and mutate it if it's triggered.

        Args:
            mut_sd: standard deviation of mutate for guassian distribution
            mut_perc: percentage of mutation for each individual parameter

        TODO: Possibly add this mutation std. dev. as a parameter? (JMM)
        """
        if mut_perc:
            if random.random() < mut_perc:
                self.__mutate_a(mut_sd)
            if random.random() < mut_perc:
                self.__mutate_b(mut_sd)
            if random.random() < mut_perc:
                self.__mutate_c(mut_sd)
        else:
            param = random.randint(0,2)
            if(param == 0):
                self.__mutate_a(mut_sd)
            elif(param == 1):
                self.__mutate_b(mut_sd)
            else:
                self.__mutate_c(mut_sd)
