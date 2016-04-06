"""
    Class to hold information about various sensors on a robot.
"""

import random
import ode
from temp_signal import TSignal

class Sensors(object):
    """ Provides methods to emulate sensors on a robot. """

    def __init__(self,man):
        """ Initializer """
    
        self.man = man

        """ Touch Sensor Data """
        self.touch_index = 0
        self.touch = {}
        self.touching = []

        """ Joint Position Sensor Data """
        self.joints = []

        """ Temporal Signal Generators """
        self.tsignals = []

    def copy(self):
        """ Copy self. """
        s = Sensors(self.man)
        s.touch = self.touch
        s.touching = self.touching
        s.joints = self.joints
        for tsig in self.tsignals:
            s.add_temp_signal(params=tsig.serialize())

        return s

    def __str__(self):
        """ To string function. """
        out = ""
        for i in xrange(len(self.tsignals)-1):
            out += str(self.tsignals[i])+","
        out += str(self.tsignals[-1])+"\n"
        return out

    def reset(self):
        """ Reset the information held by the sensor class. """
        self.touch_index = 0
        self.touch = {}
        self.touching = []
        self.joints = []
        self.tsignals = []

    def mutate(self):
        """ Mutate the a signal node if necessary. """
        if len(self.tsignals) > 0:
            index = random.randint(0,len(self.tsignals)-1)
            self.tsignals[index].mutate()

    """ Touch Sensor Information """
    def add_touch_sensor(self,body_nums):
        """ Add a touch sensor to the dictionary.

        Args:
            body_nums: body number to add as a touch sensor.
        """
        if type(body_nums) is list:
            for bnum in body_nums:
                self.touch[bnum] = self.touch_index
                self.touch_index += 1
                self.touching.append(0)
        else:
            self.touch[body_nums] = self.touch_index
            self.touch_index += 1
            self.touching.append(0)

    def is_touch_sensor(self,body_num):
        """ See if a body is a touch sensor.  

        Args:
            body_num: body number to check
        Returns:
            1 if a touch sensor, 0 if not
        """
        if body_num in self.touch:
            return 1
        else:
            return 0

    def activate_touch_sensor(self,body_num):
        """ Activate a touch sensor. 

        Args:
            body_num: body number of the sensor to trigger
        """
        self.touching[self.touch[body_num]] = 1

    def get_touch_sensor_states(self):
        """ Return a list containing the current touch sensor states. """
        return self.touching

    def clear_touching(self):
        """ Reset the touch values to 0. """
        for t in self.touching:
            t = 0

    def clear_touching_fixed(self):
        """ Reset the touch values to 0. """
        for i in range(len(self.touching)):
            self.touching[i] = 0

    """ Joint Position Sensor Information """
    def add_joint_sensor(self,joint):
        """ Add a joint sensor to the suite. 

        Args:
            joint: ODE joint object
        """
        self.joints.append(joint)

    def register_joint_sensors(self,joint_nums):
        """ Register a list of joints as joint sensors.

        Args:
            joint_nums: list of joint numbers to register
        """
        if type(joint_nums) is list:
            self.joints += joint_nums
#            for jnum in joint_nums:
#                self.joints.append(joint_nums)
        else:
            self.joints.append(joint_nums)

    def get_joint_sensors(self):
        """ Get the position of each joint sensor.

        Returns:
            list containing position of joints from 0 to 1.
        """
        positions = []

        for j in self.joints:
            low1 = self.man.get_uni_joint_limit(j,ode.ParamLoStop)
            high1 = self.man.get_uni_joint_limit(j,ode.ParamHiStop)
            cur_pos = self.man.get_uni_joint_position(j,0)
            if high1-low1 == 0:
                positions.append(0)
            else:
                positions.append(cur_pos/(high1-low1))

            low2 = self.man.get_uni_joint_limit(j,ode.ParamLoStop2)
            high2 = self.man.get_uni_joint_limit(j,ode.ParamHiStop2)
            cur_pos = self.man.get_uni_joint_position(j,1)
            if high2-low2 == 0:
                positions.append(0)
            else:
                positions.append(cur_pos/(high2-low2))
        return positions

    def clear_joint_sensors(self):
        """ Clear the list of joint sensors. """
        self.joints = []

    def add_temp_signal(self,params=0):
        """ Add a temporal signal node to the sensor suite. 

        Args:
            params: parameters to create a predefined temporal signal node
        """
        self.tsignals.append(TSignal(params=params))

    def get_temp_signals(self):
        """ Get the signal from all temporal signal nodes. """
        return [tsig.next() for tsig in self.tsignals]
            
    def clear_temp_signals(self):
        """ Clear the list of temporal signals. """
        self.tsignals = []
