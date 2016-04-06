"""
    Wrapper to conduct the actual simulation of a quadruped robot.  Access through methods: 
    evaluate individual, and physics only validation.
"""

import sys, os, random
import itertools
import math

sys.path.insert(0, '../../')

from ODESystem import ODEManager
from ODESystem import ODEVisualizer
from ODESystem import Placement

import MultiNEAT as NEAT
from Controllers import MultiNEAT_logging as mnlog

from MNodePkg import MuscleNetwork
from MNodePkg import MusNet_logging as muslog
from MNodePkg import MusXMLParser

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

def ReadMuscleNetwork(Mus_Net_File):
    """ Read a muscle network into a string from a file.

    Args:
        run_num: run number
        output_path: filepath to look for file
        gen: generation to look at
    Returns:
        string containing the muscle network xml information
    """
    filepath = Mus_Net_File
    best_per_gen = []
    mus_str = ""
    reading_mus = False
    with open(filepath,"r") as f:
        for line in f:
            pline = line.strip()
            if pline[0] != "<":
                pass
            elif pline == "<MuscleNetwork>":
                mus_str += line
                reading_mus = True
            elif pline == "</MuscleNetwork>":
                mus_str += line
                reading_mus = False
                best_per_gen.append(mus_str)
                mus_str = ""
            elif reading_mus:
                mus_str += line
    print(best_per_gen[0])
    return best_per_gen[0]

def LoadBestIndividual(Mus_Net_File):
    """ Load the best individual from te given file.

    Args:
        run_num: run number
        output_path: filepath to look for file.
        gen: generation to look at.
    Returns:
        muscle network built from the file.
    """
    mus_str = ReadMuscleNetwork(Mus_Net_File) 
    return MusXMLParser.parse_muscle_network(mus_str)

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

    def __init__(self, log_frames=0, run_num=0, eval_time=10., dt=.02, n=4, con_type="single",hyperNEAT=False,substrate=False,periodic=True,sym_mus_mods=False):
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
        self.cur_mus_net = 0

        self.hyperNEAT = True if hyperNEAT else False
        self.substrate = substrate

        self.ann_mus_mod_con_type = con_type

        # Whether we include a periodic oscillating input signal.
        self.periodic = periodic

        # Whether the Muscle Models are symmetric (Same mus groups just mirrored left/right or not.)
        self.sym_mus_mods = sym_mus_mods

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
            
            activations = []

            # Determine the activation strategy being employed (Symmetric muscle groups or not.)
            if not self.sym_mus_mods:
                # Determine whether working with individually connected nodes or not.
                if self.ann_mus_mod_con_type == "single":
                    activations = self.cur_mus_net.get_avg_activations(0,inputs=nn_activations)
                elif self.ann_mus_mod_con_type == "ind":
                    # Wrap the activations by how many nodes per muscle group (4 in this case.)
                    nn_activations = [nn_activations[4*i:4*(i+1)] for i in range(len(nn_activations)/4)]

                    # Get the muscle network activations.
                    activations = self.cur_mus_net.get_avg_activations_ind_mus_nodes(0,inputs=nn_activations)
            else:
                # Determine whether working with individually connected nodes or not.
                if self.ann_mus_mod_con_type == "single":
                    # Get activations for the left and right side separately.
                    left_activations = self.cur_mus_net.get_avg_activations(0,inputs=nn_activations[:len(nn_activations)/2])
                    right_activations = self.cur_mus_net.get_avg_activations(0,inputs=nn_activations[len(nn_activations)/2:])

                    # Flip the outward axis of movement for the right side of the animat.
                    # This step enforces symmetry.
                    # Note: Don't need this step for this quadruped, morphologically I flipped the axes
                    # in the robot setup.
                    #right_activations = [[i[0],-1*i[1]] for i in right_activations]

                    # Combine the activations.
                    for l_a, r_a in zip(left_activations, right_activations):
                        activations.append(l_a)
                        activations.append(r_a)
                elif self.ann_mus_mod_con_type == "ind":
                    # Wrap the activations by how many nodes per muscle group (4 in this case.)
                    nn_activations = [nn_activations[4*i:4*(i+1)] for i in range(len(nn_activations)/4)]

                    # Get activations for the left and right side separately.
                    left_activations = self.cur_mus_net.get_avg_activations_ind_mus_nodes(0,inputs=nn_activations[:len(nn_activations)/2])
                    right_activations = self.cur_mus_net.get_avg_activations_ind_mus_nodes(0,inputs=nn_activations[len(nn_activations)/2:])

                    # Flip the outward axis of movement for the right side of the animat.
                    # This step enforces symmetry.
                    # Note: Don't need this step for this quadruped, morphologically I flipped the axes
                    # in the robot setup.
                    #right_activations = [[i[0],-1*i[1]] for i in right_activations]

                    # Combine the activations.
                    for l_a, r_a in zip(left_activations, right_activations):
                        activations.append(l_a)
                        activations.append(r_a)

            quadruped.actuate_joints_by_pos(positions=activations)
        else:   
            #print("Not Current Network!")
            # Actuate all joints at a steady speed.
            #quadruped.actuate_joints_by_pos(positions=[[0.,math.sin(2.*math.pi*(self.elapsed_time))] for i in xrange(8)])
            quadruped.actuate_joints_by_pos(positions=[
                [0.2,0.4],
                [0.2,0.],
                [0.2,0.],
                [0.2,0.2],
                [0.5,-0.3],
                [0.2,0.],
                [0.2,0.],
                [0.2,0.2] ])

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

    def evaluate_individual(self,genome,mus_net):
        """ Evaluate an individual solution. 

        Args:
            genome: genome of the individual to evaluate

        Returns:
            fitness value of the individual
        """
        global man,current_network,cur_mus_net,quadruped

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

        # Detect if there is a mismatch between genome id and muscle network id.
        if genome.GetID() != mus_net.gid:
            print("WARNING!WARNING!WARNING!WARNING!WARNING!")

        # Setup the muscle network
        self.cur_mus_net = mus_net

        # Conduct the evaluation
        fit = self.physics_only_simulation() 
        #print(genome.GetID(), fit)

        return fit, len(self.current_network.neurons), len(self.current_network.connections)

    def evaluate_individual_wrap(self,com_args):
        """ Decompress the arguments provided to multiprocessing.  

        Args:
            com_args: compressed arguments for evaluate_individual()
        """

        return self.evaluate_individual(*com_args)

    def validator(self,NEAT_file,Mus_Net_File):
        """ Validate a single run. 

        Args:
            NEAT_File: file for the NEAT genome
            Mus_Net_File: file for the muscle network
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

        # Load in the best performing Muscle Network
        self.cur_mus_net = LoadBestIndividual(Mus_Net_File) 

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
