"""
    Wrapper to conduct the actual simulation of a quadruped robot.  Access through methods: 
    evaluate individual, and physics only validation.
"""

import sys, os, random
import itertools
import math

sys.path.insert(0, '../../../')

from ODESystem import ODEManager
from ODESystem import ODEVisualizer
from ODESystem import Placement

import MultiNEAT as NEAT
from Controllers import MultiNEAT_logging as mnlog

from Robot import Sensors

import ode

man = 0
quadruped = 0

class Quadruped(object):
    """ Represent the quadruped robot. """

    def __init__(self,man,base_pos=[0,0,0],morphology_genome={}):
        """ Initialize the robot in the ODE environment. 

        Arguments:
            man: ODE Manager for the Physics Simulation
            base_pos: base position to start the robot from
            morphology_genome: dict of dicts which contain different parameters for the morphology (TODO)
        """
        self.man = man
        self.body_keys = []

        # Sensors for robot.
        self.sensor = Sensors(man)

        # Hardware Limits

        # Initialize the robot.
        self.__create_robot(base_pos=base_pos,morphology=morphology_genome)

    def __create_robot(self,base_pos=[0,0,0],morphology={}):
        """ Create the robot used in the experiment. 

        Arguments:
            base_pos: base position to start the robot from
            morphology: optional dict of dicts defining measurements for various parts of the robot.
        """

        # Constants for the different body parts.
        
        # Main Body
        BODY_DIMS = morphology['body_dims'] if 'body_dims' in morphology else [3.0,0.5,1.0]
        BODY_POS  = [morphology['body_pos'][0]-base_pos[0],morphology['body_pos'][1]-base_pos[1],morphology['body_pos'][2]-base_pos[2]] \
            if 'body_pos' in morphology else [0.0+base_pos[0],1.60+base_pos[1],0.0+base_pos[2]]
        BODY_MASS = morphology['body_mass'] if 'body_mass' in morphology else 10.

        # Upper Legs
        F_UPP_DIMS = R_UPP_DIMS = morphology['u_l_dims'] if 'u_l_dims' in morphology else [0.5,.10]
        F_UPP_MASS = R_UPP_MASS = morphology['u_l_mass'] if 'u_l_mass' in morphology else 2.

        R_UPP_DIMS = [.5,.10]
        R_UPP_MASS = 2.

        # Custom Measurements Per Leg Group

        # Front Upper Legs
        if 'f_u_dims' in morphology:
            F_UPP_DIMS = morphology['f_u_dims']
        if 'f_u_mass' in morphology:
            F_UPP_MASS = morphology['f_u_mass']

        # Rear Upper Legs
        if 'r_u_dims' in morphology:
            R_UPP_DIMS = morphology['r_u_dims']
        if 'r_u_mass' in morphology:
            R_UPP_MASS = morphology['r_u_mass']

        # Middle Legs
        F_MID_DIMS = R_MID_DIMS = morphology['l_m_dims'] if 'l_m_dims' in morphology else [.75,.10]
        F_MID_MASS = R_MID_MASS = morphology['l_m_mass'] if 'l_m_mass' in morphology else 1.

        # Custom Measurements Per Leg Group

        # Front Lower Legs
        if 'f_m_dims' in morphology:
            F_MID_DIMS = morphology['f_m_dims']
        if 'f_m_mass' in morphology:
            F_MID_MASS = morphology['f_m_mass']

        # Rear Lower Legs
        if 'r_m_dims' in morphology:
            R_MID_DIMS = morphology['r_m_dims']
        if 'r_m_mass' in morphology:
            R_MID_MASS = morphology['r_m_mass']

        # Joint Power for the legs
        F_UPP_FORCE = R_UPP_FORCE = F_MID_FORCE = R_MID_FORCE = F_LOW_FORCE = R_LOW_FORCE = [80.,80.]

        joint_range = math.radians(120.)

        # Keep track of body and joint numbers.
        b_num = 0
        j_num = 0

        # Create the Main Body
        self.body_keys.append(self.man.create_box(b_num,BODY_DIMS,BODY_POS,density=BODY_MASS)) 
        b_num += 1

        # Create the Right Upper Front Leg
        self.body_keys.append(self.man.create_capsule(b_num, F_UPP_MASS, F_UPP_DIMS[0], F_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        #rot_t = [90.0,-60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [BODY_DIMS[0]/2.*.85,BODY_POS[1],BODY_DIMS[2]/2.+F_UPP_DIMS[1]/2.+.05]
        Placement.place_capsule_at_trans(self.man.bodies[b_num],pos=con_point,rot=rot_t)
        self.man.create_universal(j_num, con_point,[0,b_num],axis1=[0,0,-1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Upper Front Leg
        self.body_keys.append(self.man.create_capsule(b_num, F_UPP_MASS, F_UPP_DIMS[0], F_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [90.0,-60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [BODY_DIMS[0]/2.*.85,BODY_POS[1],-(BODY_DIMS[2]/2.+F_UPP_DIMS[1]/2.+.05)]
        Placement.place_capsule_at_trans(self.man.bodies[b_num],pos=con_point,rot=rot_t)
        self.man.create_universal(j_num, con_point,[0,b_num],axis1=[0,0,-1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Right Upper Rear Leg
        self.body_keys.append(self.man.create_capsule(b_num, R_UPP_MASS, R_UPP_DIMS[0], R_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [90.0,60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [-BODY_DIMS[0]/2.*.8,BODY_POS[1],BODY_DIMS[2]/2.+R_UPP_DIMS[1]/2.+0.05]
        Placement.place_capsule_at_trans(self.man.bodies[b_num],pos=con_point,rot=rot_t)
        self.man.create_universal(j_num, con_point,[0,b_num],axis1=[0,0,1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Upper Rear Leg
        self.body_keys.append(self.man.create_capsule(b_num, R_UPP_MASS, R_UPP_DIMS[0], R_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [90.0,60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [-BODY_DIMS[0]/2.*.8,BODY_POS[1],-(BODY_DIMS[2]/2.+R_UPP_DIMS[1]/2.+0.05)]
        Placement.place_capsule_at_trans(self.man.bodies[b_num],pos=con_point,rot=rot_t)
        self.man.create_universal(j_num, con_point,[0,b_num],axis1=[0,0,1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Right Lower Front Leg
        self.body_keys.append(self.man.create_capsule(b_num, F_MID_MASS, F_MID_DIMS[0], F_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(self.man.bodies[b_num-4],self.man.bodies[b_num],rot=rot_t) 
        self.man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,-1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Lower Front Leg
        self.body_keys.append(self.man.create_capsule(b_num, F_MID_MASS, F_MID_DIMS[0], F_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(self.man.bodies[b_num-4],self.man.bodies[b_num],rot=rot_t) 
        self.man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,-1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Right Lower Rear Leg
        self.body_keys.append(self.man.create_capsule(b_num, R_MID_MASS, R_MID_DIMS[0], R_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,-135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(self.man.bodies[b_num-4],self.man.bodies[b_num],rot=rot_t) 
        self.man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Lower Rear Leg
        self.body_keys.append(self.man.create_capsule(b_num, R_MID_MASS, R_MID_DIMS[0], R_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,-135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(self.man.bodies[b_num-4],self.man.bodies[b_num],rot=rot_t) 
        self.man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Add in information about the feet.
        self.sensor.add_touch_sensor([5,6,7,8])

        # Add in joint positions sensors.
        self.sensor.register_joint_sensors([0,1,2,3,4,5,6,7])

    def actuate_joints_by_pos(self,positions=[[0.,0.] for i in xrange(8)]):
        """ Actuate the joints of the quadruped to the specified position. 

        Arguments:
            positions: position of the joints to actuate to.
        """

        for i,p in enumerate(positions):
            self.man.actuate_universal(i,p[0],p[1])

    def get_sensor_states(self):
        """ Get the states of the various sensors on the robot. """
        sensors = [i for i in self.sensor.get_touch_sensor_states()]
        sensors += [i for i in self.sensor.get_joint_sensors()]
        return sensors

    def reset_touch_sensors(self):
        """ Reset the touch sensors. """
        self.sensor.clear_touching()

############################################################################################################        

def euclidean_distance(p1,p2):
    """ Calculate the 2d Euclidean distance of two coordinates.
    
    Args:
        p1: position 1
        p2: position 2
    Returns:
        Euclidean distance between the two points.
    """
    return math.sqrt((p1[0]-p2[0])**2 + (p1[2]-p2[2])**2) 

############################################################################################################

# Collision callback
def near_callback(args, geom1, geom2):
    """Callback function for the collide() method.

    This function checks if the given geoms do collide and
    creates contact joints if they do.
    """

    # Check to see if the two objects are connected.  Don't collide.
    if(man.are_connected(geom1.getBody(), geom2.getBody())):
        return

    # Check to see if one of the objects is a foot sensor and the other
    # is the ground.
    if ((geom1 == man.floor and quadruped.sensor.is_touch_sensor(man.get_body_key(geom2.getBody()))) or
        (quadruped.sensor.is_touch_sensor(man.get_body_key(geom1.getBody())) and geom2 == man.floor)):
        body_id = -1
        if geom1 == man.floor:
            body_id = man.get_body_key(geom2.getBody())
        else:
            body_id = man.get_body_key(geom1.getBody())
        quadruped.sensor.activate_touch_sensor(body_id)

    # Check if the objects do collide
    contacts = man.generate_contacts(geom1, geom2)

    # Create contact joints
    man.world,man.contactgroup = args
    for c in contacts:
        c.setBounce(0.2)
        c.setMu(5000)
        j = man.create_contact_joint(c)
        j.attach(geom1.getBody(), geom2.getBody())

class Simulation(object):
    """ Define a simulation to encapsulate an ODE simulation. """

    def __init__(self, log_frames=0, run_num=0, eval_time=10., dt=.02, n=4, con_type="single",hyperNEAT=False,substrate=False,periodic=True):
        """ Initialize the simulation class. """
        global simulate

        man = ODEManager(near_callback, stepsize=dt/n, log_data=log_frames, run_num=run_num)

        # Settings for the simulation.
        self.log_frames = log_frames
        self.run_num = run_num
        self.eval_time = eval_time
        self.elapsed_time = 0.
        self.dt = dt            # Timestep for simulation.
        self.n = n              # How many timesteps to simulate per callback.

        # Quadruped Specific Setup
        quadruped = Quadruped(man=man)

        self.current_network = 0

        self.hyperNEAT = True if hyperNEAT else False
        self.substrate = substrate

        # Whether we include a periodic oscillating input signal.
        self.periodic = periodic

    def update_callback(self):
        """ Function to handle updating the joints and such in the simulation. """
        
        if self.current_network:
            inputs = []
            if self.periodic:
                inputs.append(math.sin(2.*math.pi*(self.elapsed_time)))
            inputs += quadruped.get_sensor_states()
            inputs.append(1.0) # Bias node

            # Send inputs to the ANN and get the outputs.
            self.current_network.Input(inputs) 
            self.current_network.Activate()
            nn_activations = self.current_network.Output()

            quadruped.actuate_joints_by_pos(positions=[[nn_activations[i],nn_activations[i+1]] for i in range(0,len(nn_activations),2)])
        else:   
            print("Not Current Network!")
            # Actuate all joints at a steady speed.
            quadruped.actuate_joints_by_pos(positions=[[0.,math.sin(2.*math.pi*(self.elapsed_time))] for i in xrange(8)])

    def reset_simulation(self):
        """ Reset the simulation. """
        
        man.delete_joints()
        man.delete_bodies()
        self.elapsed_time = 0.

    def simulate(self):
        """ Perform physics simulation. """

        if self.elapsed_time < self.eval_time: 
            self.update_callback()
        if self.elapsed_time >= self.eval_time:
            fit = euclidean_distance(man.get_body_position(0),[0,0,0])
            self.reset_simulation()
            return False, fit
        quadruped.reset_touch_sensors()
        return True, 0 

    def physics_only_simulation_validator(self):
        """ Conduct a physics only simulation while logging the body data.

        We then use this information to play back the simulation on a different machine.
        
        Args:
            output_path: path to write the log file to.
            ind_num: individual number
        """
      
        go_on, fit = self.simulate()
        while go_on:
            # Simulate physics
            man.step(near_callback, self.n)
            self.elapsed_time += self.dt
            go_on, fit = self.simulate()

        print(fit)

    def physics_only_simulation(self):
       
        go_on, fit = self.simulate()
        while go_on:
            # Simulate physics
            man.step(near_callback, self.n)
            self.elapsed_time += self.dt
            go_on, fit = self.simulate()

        return fit

    def evaluate_individual(self,genome):
        """ Evaluate an individual solution. 

        Args:
            genome: genome of the individual to evaluate

        Returns:
            fitness value of the individual
        """
        global man,current_network,quadruped

        # Initialize the manager to be unique to the process.
        man = ODEManager(near_callback, stepsize=self.dt/self.n, log_data=self.log_frames)

        # Initialize the quadruped
        quadruped = Quadruped(man=man)

        # Load in the ANN from the population
        self.current_network = NEAT.NeuralNetwork()
        if not self.hyperNEAT:
            genome.BuildPhenotype(self.current_network)
        else:
            genome.BuildHyperNEATPhenotype(self.current_network,self.substrate)

        # Conduct the evaluation
        fit = self.physics_only_simulation() 
        #print(genome.GetID(), fit)

        return fit, len(self.current_network.neurons), len(self.current_network.connections)

    def validator(self,NEAT_file):
        """ Validate a single run. 

        Args:
            NEAT_File: file for the NEAT genome
        """
        global man, quadruped

        # Initialize the manager to be unique to the process.
        man = ODEManager(near_callback, stepsize=self.dt/self.n, log_data=self.log_frames, run_num=self.run_num)

        # Initialize the quadruped
        quadruped = Quadruped(man=man)

        # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
        if self.log_frames:
            man.log_world_setup(self.eval_time,ind_num=self.run_num)

        # Load in the best performing NEAT genome
        genome = NEAT.Genome(NEAT_file)
        self.current_network = NEAT.NeuralNetwork()
        if not self.hyperNEAT:
            genome.BuildPhenotype(self.current_network)
        else:
            genome.BuildHyperNEATPhenotype(self.current_network,self.substrate)

        fit = self.physics_only_simulation_validator()

        print(fit)

    def debug_validator(self):
        """ Validate a single run. """
        global man, quadruped

        # Initialize the manager to be unique to the process.
        man = ODEManager(near_callback, stepsize=self.dt/self.n, log_data=self.log_frames, run_num=self.run_num)

        # Initialize the quadruped
        quadruped = Quadruped(man=man)

        # If logging the output, tell manager to write the body type, dimensions, and position to the logging file.
        if self.log_frames:
            man.log_world_setup(self.eval_time,ind_num=self.run_num)

        fit = self.physics_only_simulation_validator()

        print(fit)
