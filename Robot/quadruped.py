"""
    Implements a Quadruped robot meant to be instantiated with an ODE manager instance.  

    TODO: Add functionality to read dimensions from a configuration file.
"""

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
        self.body_keys.append(man.create_box(b_num,BODY_DIMS,BODY_POS,density=BODY_MASS)) 
        b_num += 1

        # Create the Right Upper Front Leg
        self.body_keys.append(man.create_capsule(b_num, F_UPP_MASS, F_UPP_DIMS[0], F_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        #rot_t = [90.0,-60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [BODY_DIMS[0]/2.*.85,BODY_POS[1],BODY_DIMS[2]/2.+F_UPP_DIMS[1]/2.+.05]
        Placement.place_capsule_at_trans(man.bodies[b_num],pos=con_point,rot=rot_t)
        man.create_universal(j_num, con_point,[0,b_num],axis1=[0,0,-1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Upper Front Leg
        self.body_keys.append(man.create_capsule(b_num, F_UPP_MASS, F_UPP_DIMS[0], F_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [90.0,-60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [BODY_DIMS[0]/2.*.85,BODY_POS[1],-(BODY_DIMS[2]/2.+F_UPP_DIMS[1]/2.+.05)]
        Placement.place_capsule_at_trans(man.bodies[b_num],pos=con_point,rot=rot_t)
        man.create_universal(j_num, con_point,[0,b_num],axis1=[0,0,-1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Right Upper Rear Leg
        self.body_keys.append(man.create_capsule(b_num, R_UPP_MASS, R_UPP_DIMS[0], R_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [90.0,60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [-BODY_DIMS[0]/2.*.8,BODY_POS[1],BODY_DIMS[2]/2.+R_UPP_DIMS[1]/2.+0.05]
        Placement.place_capsule_at_trans(man.bodies[b_num],pos=con_point,rot=rot_t)
        man.create_universal(j_num, con_point,[0,b_num],axis1=[0,0,1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Upper Rear Leg
        self.body_keys.append(man.create_capsule(b_num, R_UPP_MASS, R_UPP_DIMS[0], R_UPP_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [90.0,60.0,0.0]
        rot_t = [90.0,0.0,0.0]
        con_point = [-BODY_DIMS[0]/2.*.8,BODY_POS[1],-(BODY_DIMS[2]/2.+R_UPP_DIMS[1]/2.+0.05)]
        Placement.place_capsule_at_trans(man.bodies[b_num],pos=con_point,rot=rot_t)
        man.create_universal(j_num, con_point,[0,b_num],axis1=[0,0,1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_UPP_FORCE[0],fmax2=F_UPP_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Right Lower Front Leg
        self.body_keys.append(man.create_capsule(b_num, F_MID_MASS, F_MID_DIMS[0], F_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(man.bodies[b_num-4],man.bodies[b_num],rot=rot_t) 
        man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,-1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Lower Front Leg
        self.body_keys.append(man.create_capsule(b_num, F_MID_MASS, F_MID_DIMS[0], F_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(man.bodies[b_num-4],man.bodies[b_num],rot=rot_t) 
        man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,-1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Right Lower Rear Leg
        self.body_keys.append(man.create_capsule(b_num, R_MID_MASS, R_MID_DIMS[0], R_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,-135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(man.bodies[b_num-4],man.bodies[b_num],rot=rot_t) 
        man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,1],axis2=[1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

        # Create the Left Lower Rear Leg
        self.body_keys.append(man.create_capsule(b_num, R_MID_MASS, R_MID_DIMS[0], R_MID_DIMS[1], [0.0,0.0,0.0],rot=[0.,0.,0.]))
        # rot_t = [0.0,-135.0,0.0]
        rot_t = [0.0,0.0,0.0]
        con_point = Placement.place_capsule_trans(man.bodies[b_num-4],man.bodies[b_num],rot=rot_t) 
        man.create_universal(j_num, con_point,[b_num-4,b_num],axis1=[0,0,1],axis2=[-1,0,0],loStop1=-joint_range,hiStop1=joint_range,loStop2=-joint_range,hiStop2=joint_range,fmax=F_LOW_FORCE[0],fmax2=F_LOW_FORCE[1])
        b_num += 1
        j_num += 1

    def actuate_joints_by_pos(self,positions=[[0.,0.] for i in xrange(8)]):
        """ Actuate the joints of the quadruped to the specified position. 

        Arguments:
            positions: position of the joints to actuate to.
        """

        for i,p in enumerate(positions):
            man.actuate_universal(i,p[0],p[1])