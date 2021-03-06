'''
    Manager implementation to create and manage an ODE instance.
'''
import math
import ode

from vector_ops import *
from placement import Placement

ANG_TO_RAD = math.pi/180.
RAD_TO_ANG = 180./math.pi

class ODEManager:
    """An instance manager for ODE"""

    def __init__(self, col_callback, stepsize=0.005,log_data=False,output_path="",gravity=-9.81,fluid_dynamics=0,run_num=0,erp=0.5,cfm=1E-4):
        # Create a world object
        self.world = ode.World()
        self.world.setGravity((0,gravity,0) )
        self.world.setERP(erp)
        self.world.setCFM(cfm)
        self.world.setContactMaxCorrectingVel(5.)

        # Run Number
        self.run_num = run_num

        # Create a space object
        self.space = ode.Space()

        # Create a plane geom which prevent the objects from falling forever
        self.floor = ode.GeomPlane(self.space, (0,1,0), 0)

        # A joint group for the contact joints that are generated whenever
        # two bodies collide
        self.contactgroup = ode.JointGroup()

        # A list of ODE Bodies
        self.bodies = {}

        # Maintain a data structure of the surfaces of each body if fluid dynamics are required.
        self.fluid_dynamics=fluid_dynamics
        if(fluid_dynamics):
            self.surfaces = {}

        # A list of geoms for the bodies
        self.geoms = {}

        # A dict of terrain geoms for the world.
        self.terrain_geoms = {}

        # A list of joints in the world.
        self.joints = {}

        # Register the collision callback
        self.col_callback = col_callback

        # Set the stepsize for the instance.
        self.stepsize = stepsize

        # Create a cushion for joint limits.
        self.joint_cushion = 0.#5.

        # Create a maximum velocity that a joint is able to move in one step.
        self.max_joint_vel = 36000. * self.stepsize*ANG_TO_RAD

        # Logging functionality
        self.log_data = log_data
        self.output_path = output_path
        if self.log_data:
            self.positions = []
            self.quaternions = []
            self.joint_angles = []

    def create_sphere(self, key, density, radius, pos):
        """Create a sphere and its corresponding geom.

        Arguments:
        @param key: number id to assign to the sphere
        @type key: int
        @param density: density of a sphere
        @type density: float
        @param radius: radius of a sphere
        @type radius: float
        @param pos: list of floats of the position for the sphere
        @type pos: [float,float,float]
        """
        # Auto label the joint key or not.
        key = len(self.bodies) if key == -1 else key

        # Create body
        body = ode.Body(self.world)
        M = ode.Mass()
        M.setSphere(density, radius)
        body.setMass(M)

        # Set the position of the body.
        body.setPosition((pos))

        # Set parameters for drawing the body
        body.shape = "sphere"
        body.radius = radius

        # Create a sphere geom for collision detection
        geom = ode.GeomSphere(self.space, radius)
        geom.setBody(body)

        # Append to the manager lists.
        self.bodies[key] = body
        self.geoms[key] = geom

        return key

    def self_create_box(self,density, lx, ly, lz):
        """Create a box body and its corresponding geom.
        
        Arguments:
        density - density of the given body
        lx : x dimension
        ly : y dimension
        lz : z dimension

        """
       
        # Create body
        body = ode.Body(self.world)
        M = ode.Mass()
        M.setBox(density, lx, ly, lz)
        body.setMass(M)
                            
        # Set parameters for drawing the body
        body.shape = "box"
        body.boxsize = (lx, ly, lz)
                                        
        # Create a box geom for collision detection
        geom = ode.GeomBox(self.space, lengths=body.boxsize)
        geom.setBody(body)
        return body, geom
   
    def create_capsule(self,key,density,length,radius,pos,base=0,rot=0,R=0.):
        """Creates a capsule body and corresponding geom.
        
        Arguments:
        key : number id to assign to the capsule
        density : density of the given body
        length : length of the capsule
        radius : radius of the capsule
        pos : position of the center of the capsule (x,y,z list)
        base : place new object at negative end of base object

        """
        # Auto label the joint key or not.
        key = len(self.bodies) if key == -1 else key

        # create capsule body (aligned along the z-axis so that it matches the
        #   GeomCCylinder created below, which is aligned along the z-axis by
        #   default)
        body = ode.Body(self.world)
        M = ode.Mass()
        M.setCapsule(density, 3, radius, length)
        body.setMass(M)
        
        # create a capsule geom for collision detection
        geom = ode.GeomCCylinder(self.space, radius, length)
        geom.setBody(body)
        
        # set the position of the capsule
        body.setPosition((pos[0],pos[1],pos[2]))

        # set parameters for drawing the body
        body.shape = "capsule"
        body.length = length
        body.radius = radius

        # set the rotation of the capsule
        if(rot):
            body.setRotation(self.form_rotation(rot))
       
        # set the rotation of the capsule directly
        if R:
            body.setRotation(R)

        self.bodies[key] = body
        self.geoms[key] = geom

        if(base):
            Placement.place_object(self.bodies[base],body)

        if(self.fluid_dynamics):
            self.create_surfaces(key,1.)  

        return key      
   
    def create_cylinder(self,key,density,length,radius,pos,base=0,rot=0,R=0.):
        """Creates a cylinder body and corresponding geom.
        
        Arguments:
        key : number id to assign to the cylinder
        density : density of the given body
        length : length of the cylinder
        radius : radius of the cylinder
        pos : position of the center of the cylinder (x,y,z list)
        base : place new object at negative end of base object

        """
        # Auto label the joint key or not.
        key = len(self.bodies) if key == -1 else key

        # create cylinder body (aligned along the z-axis so that it matches the
        #   GeomCylinder created below, which is aligned along the z-axis by
        #   default)
        body = ode.Body(self.world)
        M = ode.Mass()
        M.setCylinder(density, 3, radius, length)
        body.setMass(M)
        
        # create a cylinder geom for collision detection
        geom = ode.GeomCylinder(self.space, radius, length)
        geom.setBody(body)
        
        # set the position of the cylinder
        body.setPosition((pos[0],pos[1],pos[2]))

        # set parameters for drawing the body
        body.shape = "cylinder"
        body.length = length
        body.radius = radius

        # set the rotation of the cylinder
        if(rot):
            body.setRotation(self.form_rotation(rot))
       
        # set the rotation of the cylinder directly
        if R:
            body.setRotation(R)

        self.bodies[key] = body
        self.geoms[key] = geom

        if(base):
            Placement.place_object(self.bodies[base],body)

        if(self.fluid_dynamics):
            self.create_surfaces(key,1.)  

        return key

    def create_box(self,key,dim,pos,terrain=False,density=10.,amp_adj=1.):
        """Create a box.
       
        Arguments:
            key: number id to assign to the box
            dim: list of l,w,h dimension for the box
            pos: list of x,y,z position for center of box
            terrain: flag indicating whether the box is terrain or not
            density: mass of the box
            amp_adj: factor for hydrodynamic calculations
        """
        # Auto label the joint key or not.
        key = len(self.bodies) if key == -1 else key

        if not terrain:
            body, geom = self.self_create_box(density,dim[0],dim[1],dim[2])
            body.setPosition((pos[0],pos[1],pos[2]))
            self.bodies[key] = body
            self.geoms[key] = geom

            if(self.fluid_dynamics):
                self.create_surfaces(key,amp_adj)
        else:
            geom = ode.GeomBox(self.space, lengths=(dim[0],dim[1],dim[2]))
            geom.setPosition((pos[0],pos[1],pos[2]))
            geom.static = True
            geom.shape = "box"
            geom.boxsize = (dim[0],dim[1],dim[2])
            geom.color=[0.1,0.6,0.6,1.0]
            self.terrain_geoms[key] = geom 

        return key

    def create_surfaces(self,key,amp_adj):
        """ Create information on the surfaces of a given body for processing fluid calculations. 

        Arguments:
        key: number id to assign to the dictionary of surfaces
        """

        x = y = z = 0
        if(self.bodies[key].shape == 'capsule'):
            x = self.bodies[key].radius
            y = self.bodies[key].radius
            z = self.bodies[key].length
        elif(self.bodies[key].shape == 'box'):
            sides = self.bodies[key].boxsize
            x = sides[0]
            y = sides[1]
            z = sides[2]

        surfaces = []
        surfaces.append({'area':y*z, 'norm':[1,0,0], 'amp_adj':amp_adj})
        surfaces.append({'area':x*z, 'norm':[0,1,0], 'amp_adj':amp_adj})
        surfaces.append({'area':y*z, 'norm':[-1,0,0], 'amp_adj':amp_adj})
        surfaces.append({'area':x*z, 'norm':[0,-1,0], 'amp_adj':amp_adj})
        surfaces.append({'area':y*x, 'norm':[0,0,1], 'amp_adj':amp_adj})
        surfaces.append({'area':y*x, 'norm':[0,0,-1], 'amp_adj':amp_adj})
        self.surfaces[key] = surfaces

    def create_fixed(self,key,pos,bod):
        """ Create a fixed joint with the world.

        Arguments:
        key : number id to assign the hinge
        pos : position of the joint
        bod : body to attach the world to 

        """
        # Auto label the joint key or not.
        key = len(self.joints) if key == -1 else key

        j = ode.FixedJoint(self.world)
        j.attach(self.bodies[bod],ode.environment)
        j.setFixed()

        j.style = "fixed"
        self.joints[key] = j

        return key

    def create_hinge(self,key,pos,bod_keys,axis=[0.,0.,1.],lims=[-1.,1.],max_force=100.):
        """ Create a hinge joint.

        Arguments:
        key : number id to assign the hinge
        pos : position of the joint
        bod_keys : list of two bodies to attach the joint to

        """
        # Auto label the joint key or not.
        key = len(self.joints) if key == -1 else key

        j = ode.HingeJoint(self.world)
        j.attach(self.bodies[bod_keys[0]], self.bodies[bod_keys[1]])
        j.setAnchor((pos))
        j.setAxis((axis))
        j.setParam(ode.ParamLoStop, lims[0])
        j.setParam(ode.ParamHiStop, lims[1])
        j.setParam(ode.ParamFMax, max_force)
        j.style = "hinge"
        self.joints[key] = j

        return key

    def create_ball(self,key,pos,bod_keys):
        """ Create a hinge joint.

        Arguments:
        key : number id to assign the hinge
        pos : position of the joint
        bod_keys : list of two bodies to attach the joint to

        """
        # Auto label the joint key or not.
        key = len(self.joints) if key == -1 else key

        j = ode.BallJoint(self.world)
        j.attach(self.bodies[bod_keys[0]], self.bodies[bod_keys[1]])
        j.setAnchor((pos))
        j.style = "ball"
        self.joints[key] = j

        return key

    def create_universal(self,key,pos,bod_keys,axis1 = [1,0,0],axis2 = [0,0,1],
        loStop1 = -ode.Infinity, hiStop1 = ode.Infinity,
        loStop2 = -ode.Infinity, hiStop2 = ode.Infinity,
        fmax = -1, fmax2 = -1, world_joint=0):
        """ Create a universal joint.

        Arguments:
        @param key : number id to assign the hinge
        @param pos : position of the joint
        @param bod_keys : list of two bodies to attach the joint to
        @param axis1 : first axis of hinge
        @param axis2 : second axis of hinge
        @param loStop1 : low limit of hinge 1
        @param hiStop1 : high limit of hinge 1
        @param loStop2 : low limit of hinge 2
        @param hiStop2 : high limit of hinge 2
        @param fmax    : max output potential for first hinge
        @param fmax2   : max output potential for second hinge
        @param world_joint: if the body is to be connected to the world
        """
        # Auto label the joint key or not.
        key = len(self.joints) if key == -1 else key

        if(pos == "None"):
            anchor = self.bodies[bod_keys[0]].getRelPointPos(
                (0.,-self.bodies[bod_keys[0]].length/2.,0.))
        else:
            anchor = pos

        j = ode.UniversalJoint(self.world)
        if(world_joint==0):
            j.attach(self.bodies[bod_keys[0]], self.bodies[bod_keys[1]])
        else:
            j.attach(self.bodies[bod_keys[0]], ode.environment)
        j.setAnchor((anchor))
        j.setAxis1((axis1))
        j.setAxis2((axis2))
        j.setParam(ode.ParamLoStop, loStop1-(self.joint_cushion*ANG_TO_RAD))
        j.setParam(ode.ParamHiStop, hiStop1+(self.joint_cushion*ANG_TO_RAD))
        j.setParam(ode.ParamLoStop2, loStop2-(self.joint_cushion*ANG_TO_RAD))
        j.setParam(ode.ParamHiStop2, hiStop2+(self.joint_cushion*ANG_TO_RAD))
        j.setParam(ode.ParamFudgeFactor,.5)
        j.setParam(ode.ParamFudgeFactor2,.5)
        if fmax == -1:
            j.setParam(ode.ParamFMax, 10.)
        else:
            j.setParam(ode.ParamFMax, fmax)

        if fmax2 == -1:
            j.setParam(ode.ParamFMax2, 10.)
        else:
            j.setParam(ode.ParamFMax2, fmax2)

        j.style = "universal"
        self.joints[key] = j

        return key

    def create_flexible_universal(self,key,pos,bod_keys,axis1 = [1,0,0],axis2 = [0,0,1],
        erp1 = 0.1, cfm1=0.1, erp2 = 0.1, cfm2 = 0.2):
        """ Create a universal joint.

        Arguments:
        @param key : number id to assign the hinge
        @param pos : position of the joint
        @param bod_keys : list of two bodies to attach the joint to
        @param axis1 : first axis of hinge
        @param axis2 : second axis of hinge
        @param erp1 : erp hinge 1
        @param cfm1 : cfm hinge 1
        @param erp2 : erp hinge 2
        @param cfm2 : cfm hinge 2
        """
        # Auto label the joint key or not.
        key = len(self.joints) if key == -1 else key

        if(pos == "None"):
            anchor = self.bodies[bod_keys[0]].getRelPointPos(
                (0.,-self.bodies[bod_keys[0]].length/2.,0.))
        else:
            anchor = pos

        j = ode.UniversalJoint(self.world)
        j.attach(self.bodies[bod_keys[0]], self.bodies[bod_keys[1]])
        j.setAnchor((anchor))
        j.setAxis1((axis1))
        j.setAxis2((axis2))
        j.setParam(ode.ParamLoStop, 0)
        j.setParam(ode.ParamHiStop, 0)
        j.setParam(ode.ParamLoStop2, 0)
        j.setParam(ode.ParamHiStop2, 0)
        j.setParam(ode.ParamStopERP, erp1)
        j.setParam(ode.ParamStopCFM, cfm1)
        j.setParam(ode.ParamStopERP2, erp2)
        j.setParam(ode.ParamStopCFM2, cfm2)
        j.setParam(ode.ParamFudgeFactor,.5)
        j.setParam(ode.ParamFudgeFactor2,.5)
        j.setParam(ode.ParamFMax, 10.)
        j.setParam(ode.ParamFMax2, 10.)

        j.style = "universal_flex"
        self.joints[key] = j

        return key

    def create_hinge2(self,key,pos,bod_keys,axis1 = [1,0,0],axis2 = [0,0,1],
        loStop1 = -ode.Infinity, hiStop1 = ode.Infinity,
        fmax = -1, fmax2 = -1):
        """ Create a universal joint.

        Arguments:
        @param key : number id to assign the hinge
        @param pos : position of the joint
        @param bod_keys : list of two bodies to attach the joint to
        @param axis1 : first axis of hinge
        @param axis2 : second axis of hinge
        @param loStop1 : low limit of hinge 1
        @param hiStop1 : high limit of hinge 1
        @param fmax    : max output potential for first hinge
        @param fmax2   : max output potential for second hinge
        """
        # Auto label the joint key or not.
        key = len(self.joints) if key == -1 else key

        if(pos == "None"):
            anchor = self.bodies[bod_keys[0]].getRelPointPos(
                (0.,-self.bodies[bod_keys[0]].length/2.,0.))
        else:
            anchor = pos

        j = ode.Hinge2Joint(self.world)
        j.attach(self.bodies[bod_keys[0]], self.bodies[bod_keys[1]])
        j.setAnchor((anchor))
        j.setAxis1((axis1))
        j.setAxis2((axis2))
        j.setParam(ode.ParamLoStop, loStop1-(self.joint_cushion*ANG_TO_RAD))
        j.setParam(ode.ParamHiStop, hiStop1+(self.joint_cushion*ANG_TO_RAD))
        j.setParam(ode.ParamFudgeFactor,.5)
        j.setParam(ode.ParamFudgeFactor2,.5)
        if fmax == -1:
            j.setParam(ode.ParamFMax, 10.)
        else:
            j.setParam(ode.ParamFMax, fmax)

        if fmax2 == -1:
            j.setParam(ode.ParamFMax2, 10.)
        else:
            j.setParam(ode.ParamFMax2, fmax2)

        j.style = "hinge2"
        self.joints[key] = j

        return key

    def get_body_position(self,key):
        """ Get the position of a body.

        Args:
            key: id associated with the body
        Returns:
            position of the body
        """
        return self.bodies[key].getPosition()

    def get_num_joints(self):
        """ Get the number of joints in the simulation."""
        return len(self.joints)

    def get_joint(self,joint_num):
        """ Get the joint denoted by ID.

        Args:
            joint_num: joint number of joint to get
        Returns:
            ODE joint object
        """
        return self.joints[joint_num]

    def set_uni_joint_force(self,jnum,max_force):
        """ Set the maximum joint force for the given joint. """
        self.joints[jnum].setParam(ode.ParamFMax, max_force)
        self.joints[jnum].setParam(ode.ParamFMax2, max_force)

    def set_uni_joint_stop_erp_cfm(self,jnum,erp1=-1,cfm1=-1,erp2=-1,cfm2=-1):
        """ Set the stop erp and cfm for a universal joint.

        Args:
            jnum: joint to set
            erp1: erp for axis 1
            cfm1: cfm for axis 1
            erp2: erp for axis 2
            cfm2: cfm for axis 2
        """
        if erp1 != -1:
            self.joints[jnum].setParam(ode.ParamStopERP, erp1)
        if cfm1 != -1:
            self.joints[jnum].setParam(ode.ParamStopCFM, cfm1)
        if erp2 != -1:
            self.joints[jnum].setParam(ode.ParamStopERP2, erp2)
        if cfm2 != -1:
            self.joints[jnum].setParam(ode.ParamStopCFM2, cfm2)


    def get_uni_joint_limit(self,jnum,param):
        """ Get the joint limit to the corresponding param. 

        Args:
            jnum: joint number
            param: ode joint parameter
        Returns:
            joint limit for that parameter
        """
        j = self.joints[jnum]
        if param == ode.ParamLoStop or param == ode.ParamLoStop2:
            return j.getParam(param)+self.joint_cushion*ANG_TO_RAD
        elif param == ode.ParamHiStop or param == ode.ParamHiStop2:
            return j.getParam(param)-self.joint_cushion*ANG_TO_RAD

    def get_uni_joint_position(self,jnum,axis):
        """ Get the position of a universal joint.

        Args:
            jnum: joint number to get
            axis: 0 or 1
        Returns:
            joint position for the specified axis
        """
        if axis == 0:
            return self.joints[jnum].getAngle1()
        else:
            return self.joints[jnum].getAngle2()

    def get_uni_joint_rel_position(self,jnum):
        """ Get the relative positions of a universal joint.

        Args:
            jnum: joint number to get

        Returns:
            position of the joints with respect to their limits (-1. to 1.)
        """
        j_pos = [0.,0.]

        # First axis
        low_lim = self.get_uni_joint_limit(jnum,ode.ParamLoStop)
        upp_lim = self.get_uni_joint_limit(jnum,ode.ParamHiStop)
        midpoint = low_lim + (upp_lim - low_lim)/2.
        # Check in case the limits of the joint are the same.
        if upp_lim == low_lim:
            j_pos[0] = 0
        else:
            j_pos[0] = (self.joints[jnum].getAngle1() - midpoint)/((upp_lim-low_lim)/2.)

        # Second axis
        low_lim = self.get_uni_joint_limit(jnum,ode.ParamLoStop2)
        upp_lim = self.get_uni_joint_limit(jnum,ode.ParamHiStop2)
        midpoint = low_lim + (upp_lim - low_lim)/2.
        # Check in case the limits of the joint are the same.
        if upp_lim == low_lim:
            j_pos[1] = 0
        else:
            j_pos[1] = (self.joints[jnum].getAngle2() - midpoint)/((upp_lim-low_lim)/2.)

        return j_pos

    def get_hinge_joint_rel_position(self,jnum):
        """ Get the relative position of a hinge joint.

        Args:
            jnum: joint number to get

        Returns:
            position of the joint with respect to its limits (-1. to 1.)
        """
        low_lim = self.joints[jnum].getParam(ode.ParamLoStop)
        upp_lim = self.joints[jnum].getParam(ode.ParamHiStop)
        midpoint = low_lim + (upp_lim - low_lim)/2.
        if upp_lim - low_lim <= 0:
            return 0.
        return (self.joints[jnum].getAngle() - midpoint)/((upp_lim-low_lim)/2.)

    def actuate_hinge(self, key, vel):
        """ 
        Move a hinge by a specified velocity.

        Arguments:
        @param key: number id associated with the hinge
        @type key: int
        @param vel: velocity to move the joint (radians).
        @type vel: float
        """
        if __debug__:
            if not self.joints[key].style == "hinge": 
                raise AssertionError("Joint #"+str(key)+" is not a hinge!")
        self.joints[key].setParam(ode.ParamVel,vel)

    def actuate_hinge2(self, key, vel):
        """ 
        Move a hinge by a specified velocity.

        Arguments:
        @param key: number id associated with the hinge
        @type key: int
        @param vel: velocity to move the joint (radians).
        @type vel: float
        """
        if __debug__:
            if not self.joints[key].style == "hinge2": 
                raise AssertionError("Joint #"+str(key)+" is not a hinge!")

        j = self.joints[key]

        # Ensure that the joint is not nearing its stops.
        cur_pos_1 = j.getAngle1()*RAD_TO_ANG
        low_lim_1 = j.getParam(ode.ParamLoStop)*RAD_TO_ANG+self.joint_cushion
        upp_lim_1 = j.getParam(ode.ParamHiStop)*RAD_TO_ANG-self.joint_cushion

        # Find the range of motion of the joint taking into account the joint cushion.
        range_1   = upp_lim_1 - low_lim_1 - 2.*self.joint_cushion

        # Find the midpoint of the range of motion taking into account the joint cushion.
        mid_pt_1  = low_lim_1 + (range_1/2.) + self.joint_cushion

        # Check to make sure we aren't within 10 degrees of each joint stop and moving towards it.  Scale to 0 if we
        # are within 3 degrees.
        diff_1 = vel
        slow_range = 5.
        if cur_pos_1 < low_lim_1:
            diff_1 = 0
        elif cur_pos_1 > upp_lim_1: 
            diff_1 = 0

        j.setParam(ode.ParamVel,diff_1)

    def actuate_universal_signal(self, key, counter, frq_1, frq_2, off_1, off_2, fun_1, fun_2):
        """
        Move a universal joint by getting signals with the given frequency and offset.

        Arguments:
        @param key: number id associated with the joint.
        @type key: int
        @param counter: current time for oscillatory signal 
        @type counter: float
        @param frq_1: frequency of oscillation for first hinge
        @type frq_1: float
        @param frq_2: frequency of oscillation for second hinge
        @type frq_2: float
        @param off_1: phase offset for first hinge
        @type off_1: float
        @param off_2: phase offset for second hinge
        @type off_2: float
        @param fun_1: function to use for first hinge (1 - sin, 2 - cos)
        @type fun_1: int
        @param fun_2: function to use for second hinge (1 - sin, 2 - cos)
        @type fun_2: int
        """
        signal_1 = 2.*math.pi*frq*counter
        signal_2 = 2.*math.pi*frq*counter

        signal_1 = sin(signal_1) if fun_1 == 1 else cos(signal_1)
        signal_2 = sin(signal_2) if fun_2 == 1 else cos(signal_2)

        self.actuate_universal(key, signal_1, signal_2)

    def actuate_universal(self, key, pos_1, pos_2):
        """ 
        Move a universal joint to a specified position.

        Arguments:
        @param key: number id associated with the joint.
        @type key: int
        @param pos_1: velocity to move the joint (radians).
        @type pos_1: float
        @param pos_2: velocity of second hinge (radians).
        @type pos_2: float

        """

        j = self.joints[key]

        if __debug__:
            if not j.style == "universal": 
                raise AssertionError("Joint #"+str(key)+" is not a universal!")
        
        # Ensure that the joint is not nearing its stops.
        cur_pos_1 = j.getAngle1()*RAD_TO_ANG
        cur_pos_2 = j.getAngle2()*RAD_TO_ANG
        low_lim_1 = j.getParam(ode.ParamLoStop)*RAD_TO_ANG+self.joint_cushion
        upp_lim_1 = j.getParam(ode.ParamHiStop)*RAD_TO_ANG-self.joint_cushion
        low_lim_2 = j.getParam(ode.ParamLoStop2)*RAD_TO_ANG+self.joint_cushion
        upp_lim_2 = j.getParam(ode.ParamHiStop2)*RAD_TO_ANG-self.joint_cushion

        # Find the range of motion of the joint taking into account the joint cushion.
        range_1   = upp_lim_1 - low_lim_1 - 2.*self.joint_cushion
        range_2   = upp_lim_2 - low_lim_2 - 2.*self.joint_cushion

        # Find the midpoint of the range of motion taking into account the joint cushion.
        mid_pt_1  = low_lim_1 + (range_1/2.) + self.joint_cushion
        mid_pt_2  = low_lim_2 + (range_2/2.) + self.joint_cushion

        # Find the target position of each joint.
        tar_pos_1 = mid_pt_1 + (pos_1 * range_1/2.)
        tar_pos_2 = mid_pt_2 + (pos_2 * range_2/2.)

        scaling_factor = 10. # Used to boost speeds of joint movement.
        diff_1    = -1. * scaling_factor * (cur_pos_1 - tar_pos_1)
        diff_2    = -1. * scaling_factor * (cur_pos_2 - tar_pos_2)

        # Check to make sure we aren't within 10 degrees of each joint stop and moving towards it.  Scale to 0 if we
        # are within 3 degrees.
        slow_range = 5.
        if tar_pos_1 < cur_pos_1 and diff_1 < 0. and cur_pos_1 - low_lim_1 < slow_range:
            diff_1 *= (tar_pos_1 - low_lim_1)/slow_range
        elif tar_pos_1 > cur_pos_1 and diff_1 > 0. and upp_lim_1 - cur_pos_1 < slow_range: 
            diff_1 *= (upp_lim_1 - tar_pos_1)/slow_range
        if tar_pos_2 < cur_pos_2 and diff_2 < 0. and cur_pos_2 - low_lim_2 < slow_range:
            diff_2 *= (tar_pos_2 - low_lim_2)/slow_range
        elif tar_pos_2 > cur_pos_2 and diff_2 > 0. and upp_lim_2 - cur_pos_2 < slow_range: 
            diff_2 *= (upp_lim_2 - tar_pos_2)/slow_range

        # Translate diff to radians.
        diff_1 *= ANG_TO_RAD
        diff_2 *= ANG_TO_RAD
        
        diff_1    = diff_1 if abs(diff_1) < self.max_joint_vel else math.copysign(self.max_joint_vel,diff_1)
        diff_2    = diff_2 if abs(diff_2) < self.max_joint_vel else math.copysign(self.max_joint_vel,diff_2) 

        # Zero out the forces if the joints are within 2 degrees of the target. (Reduces jitter.)
        if abs(diff_1) < 0.0349:
            diff_1 = 0.
        if abs(diff_2) < 0.0349:
            diff_2 = 0.
        
        j.setParam(ode.ParamVel,diff_1)
        j.setParam(ode.ParamVel2,diff_2)

    def delete_joints(self):
        """ Delete the joints held in the manager."""    
        self.joints.clear()

    def delete_bodies(self):
        """ Delete the bodies held in the manager."""
        self.geoms.clear()
        self.bodies.clear()
        if self.log_data:
            self.dump_body_data(ind_num=self.run_num)
            self.dump_joint_data(ind_num=self.run_num)

    def delete_terrain(self):
        """ Delete the terrain geoms in the manager."""
        self.terrain_geoms.clear()

    def toggle_terrain_enabled(self,threshold=5.):
        """ Enable/disable terrain geoms if they are within the threshold distance.

        Args:
            threshold: distance by which to enable or disable a terrain geom.
        """
        b_pos = self.bodies[0].getPosition()
        for k,g in self.terrain_geoms.iteritems():
            g_pos = g.getPosition()
            if dist3((b_pos),(g_pos)) <= threshold:
                g.enable()
                g.color = [0.4,0.4,0.4,1.0]
            else:
                g.disable()
                g.color = [0.2,0.2,0.2,1.0]

    def form_rotation(self,rot_deg):
        """ Form the rotation matrix for ode to apply to a body.

        Arguments:
        @param rot_deg: list of rotations(roll,pitch,yaw) to apply to the body (x,y,z)
            also (gamma,beta,alpha)
        @type rot_deg: float
        """

        x = rot_deg[0] * ANG_TO_RAD
        y = rot_deg[1] * ANG_TO_RAD
        z = rot_deg[2] * ANG_TO_RAD

        r = [0 for i in range(9)]

        r[0] = math.cos(z)*math.cos(y)
        r[1] = math.sin(z)
        r[2] = -math.sin(y)
        r[3] = -math.sin(z)
        r[4] = math.cos(z)*math.cos(x)
        r[5] = math.sin(x)
        r[6] = math.sin(y)
        r[7] = -math.sin(x)
        r[8] = math.cos(y)*math.cos(x)

        alpha = rot_deg[2]*ANG_TO_RAD
        beta  = rot_deg[1]*ANG_TO_RAD
        gamma = rot_deg[0]*ANG_TO_RAD
        
        rot = [0 for i in range(9)]
        rot[0] = math.cos(alpha)*math.cos(beta)
        rot[1] = math.cos(alpha)*math.sin(gamma)-math.sin(alpha)*math.cos(gamma)
        rot[2] = math.cos(alpha)*math.sin(beta)*math.cos(gamma)+math.sin(alpha)*math.sin(gamma)
        rot[3] = math.sin(alpha)*math.cos(beta)
        rot[4] = math.sin(alpha)*math.sin(beta)*math.sin(gamma)+math.cos(alpha)*math.cos(gamma)
        rot[5] = math.sin(alpha)*math.sin(beta)*math.cos(gamma)-math.cos(alpha)*math.sin(gamma)
        rot[6] = -math.sin(beta)
        rot[7] = math.cos(beta)*math.sin(gamma)
        rot[8] = math.cos(beta)*math.cos(gamma)

        return r

    def rotate_body_by_deg(self, key, rot_deg):
        """ Rotate a given body by the specified degrees.
        
        Arguments:
        key : number id of the body
        rot_deg : list of rotations (roll, pitch, yaw) to apply to the body
                  (x,y,z) also (gamma, beta, alpha)

        Implementation Details:
        rot : list of values for the rotation matrix, positions relate
              to the following matrix.
              [0 1 2][3 4 5][6 7 8]

        """

        self.bodies[key].setRotation(self.form_rotation(rot_deg))
    
    # Step function for the manager.
    def step_physics(self,callback):
        self.space.collide((self.world,self.contactgroup), callback)

        self.world.step(self.stepsize)

        self.contactgroup.empty()

    # Step the physics the defined number of times.
    def step(self,callback, steps):
        for i in range(steps):
            self.step_physics(callback)
        if self.log_data:
            self.log_body_data()
            self.log_joint_data()

    def get_body_key(self,body):
        """ Get the key of the body.

        Args:
            body: ODE body to get key of
        Returns:
            key of the body 
        """
        for k,b in self.bodies.iteritems():
            if b == body:
                return k
        return -1

    def log_world_setup(self,eval_time,ind_num=0):
        """ Log the initial configuration for the robot. 
        
        Args:
            eval_time: evaluation time for a simulation
        """

        # Setup the positions and quaternions file.
        with open(self.output_path+str(ind_num)+"_logged_output.dat", "w") as f:
            f.write("Version 0.1\n")
            f.write("<setup information>\n")
            f.write(str(self.stepsize)+"\n")
            f.write(str(eval_time)+"\n")
            for body in self.bodies:
                f.write(self.bodies[body].shape+", ")
                pos = list(self.bodies[body].getPosition())
                for p in pos:
                    f.write(str(p)+", ")
                if self.bodies[body].shape == "box":
                    for d in self.bodies[body].boxsize:
                        f.write(str(d)+", ")
                elif self.bodies[body].shape == "capsule" or self.bodies[body].shape == "cylinder":
                    f.write(str(self.bodies[body].length)+", ")
                    f.write(str(self.bodies[body].radius)+", ")                    
                elif self.bodies[body].shape == "sphere":
                    f.write(str(self.bodies[body].radius)+", ")
                quat = list(self.bodies[body].getQuaternion())

                # Apply fix if body is a cylinder or capsule.
                if self.bodies[body].shape == "capsule" or self.bodies[body].shape == "cylinder":
                    quat = QMultiply0(quat,[math.cos(-math.pi/2./2.),math.sin(-math.pi/2./2.),0.,0.])
                for q in quat[0:len(quat)-1]:
                    f.write(str(q)+",")
                f.write(str(quat[-1])+"\n")
            f.write("<\setup information>\n")

        # Setup the joint angles file.
        with open(self.output_path+str(ind_num)+"_joint_angles.dat", "w") as f:
            f.write("Timestep")
            for joint in self.joints:
                f.write(", ")
                if self.joints[joint].style == "hinge":
                    f.write("Hinge_Angle_"+str(joint)+", Hinge_Rel_Angle_"+str(joint))
                elif self.joints[joint].style == "universal":
                    f.write("Universal_Angle_1_"+str(joint)+", Universal_Angle_2_"+str(joint))
                    f.write(", Universal_Rel_Angle_1_"+str(joint)+", Universal_Rel_Angle_2_"+str(joint))
            f.write("\n")

    def log_body_data(self,ind_num=0):
        """ Log the provided position and quaternion data to a log file.

        Args:
            ind_num: individual number to log.
        """

        # Log the body positions to a file.
        ts_pos = []
        ts_quat = []
        for body in self.bodies:
            p = list(self.bodies[body].getPosition())
            q = list(self.bodies[body].getQuaternion())

            # Apply fix if body is a cylinder or capsule.
            if self.bodies[body].shape == "capsule" or self.bodies[body].shape == "cylinder":
                q = QMultiply0(q,[math.cos(-math.pi/2./2.),math.sin(-math.pi/2./2.),0.,0.])

            ts_pos.extend(p)
            ts_quat.extend(q)
        self.positions.append(ts_pos)
        self.quaternions.append(ts_quat)

    def log_joint_data(self,ind_num=0):
        """ Log the joint angles of the individual to a log file.

        Note:
            Currently implemented for hinge and universal joints.

        Args:
            ind_num: individual number to log
        """

        ts_angles = []
        for joint in self.joints:
            if self.joints[joint].style == "hinge":
                ts_angles.append(math.degrees(self.joints[joint].getAngle()))
                ts_angles.append(self.get_hinge_joint_rel_position(joint))
            elif self.joints[joint].style == "universal":
                # Log the actual joint angles
                ts_angles.append(math.degrees(self.get_uni_joint_position(joint,0)))
                ts_angles.append(math.degrees(self.get_uni_joint_position(joint,1)))

                # Log the relative angles in terms of joint limits
                rel_positions = self.get_uni_joint_rel_position(joint)
                ts_angles.append(rel_positions[0])
                ts_angles.append(rel_positions[1])
        self.joint_angles.append(ts_angles)


    def dump_body_data(self,ind_num=0):
        """ Dump the provided position and quaternion data to a log file.

        Args:
            ind_num: individual number to log.
        """

        # Log the body positions to a file.
        with open(self.output_path+str(ind_num)+"_logged_output.dat", "a") as f:
            print(len(self.positions),len(self.positions[0]))
            print(len(self.quaternions),len(self.quaternions[0]))
            for p,q in zip(self.positions,self.quaternions):
                f.write(str(p[0])+', '+', '.join(map(str,p[1:]))+"\n")
                f.write(str(q[0])+', '+', '.join(map(str,q[1:]))+"\n")

    def dump_joint_data(self,ind_num=0):
        """ Dump the joint positions to a log file.

        Args:
            ind_num: individual number to log.
        """
        with open(self.output_path+str(ind_num)+"_joint_angles.dat", "a") as f:
            for i,line in enumerate(self.joint_angles):
                f.write(str(i)+", "+str(line[0])+', '+', '.join(map(str,line[1:]))+"\n")

    def are_connected(self,b1,b2):
        """ Interface to call the ode dBodyAreConnected()

        Args:
            b1, b2: bodies to check if they are connected.
        """
        return True if ode.areConnected(b1,b2) else False

    def generate_contacts(self,g1,g2):
        """ Generate the contact points between two bodies. """
        return ode.collide(g1,g2)

    def create_contact_joint(self,c):
        """ Create a contact joint between two objects. 
        
        Args:
            c: contact point between two bodies
        """
        return ode.ContactJoint(self.world, self.contactgroup, c)

    def sim_fluid_dynamics(self):
        """ Simulate an aquatic environment.

        Only done if the fluid_dynamics flag is on in manager.

        NOTE: Fluid dynamics are only in place for capsules and boxes.
        """

        drag_coefficient = .35
        amp_adj = 1.

        for body in self.bodies:
            vel = self.bodies[body].getLinearVel()
            rot = self.bodies[body].getRotation()

            # Iterate through the 6 faces of a body (assuming box x,y,z parameters)
            for i in xrange(6):
                area = self.surfaces[body][i]['area'] # Area of the surface
                norm = self.surfaces[body][i]['norm'] # Norm vector [x,y,z] of the surface
                amp_adj = self.surfaces[body][i]['amp_adj'] # Apply an amplitude adjustment for a body generating thrust

                adnorm = rotate3(rot,norm) # Reorient the normal to the bodies rotation in the world.

                component = adnorm[0]*vel[0]+adnorm[1]*vel[1]+adnorm[2]*vel[2]  # Get the component of force perpendicular to surface.
                component *= drag_coefficient*area  # Compensating for the size of the surface.
                if(component < 0):
                    component = 0    # If less than zero, no drag force to apply (Surface is opposite to drag force.)
                force = self.world.impulseToForce(self.stepsize,[-component*adnorm[0]*amp_adj, -component*adnorm[1]*amp_adj, -component*adnorm[2]*amp_adj])
                self.bodies[body].addForce([force[0],force[1],force[2]])  # Add force to the body from calculated surface.
                #self.bodies[body].addForce([-component*adnorm[0]*amp_adj, -component*adnorm[1]*amp_adj, -component*adnorm[2]*amp_adj])  # Add force to the body from calculated surface.

    def sim_flow_dynamics(self, vel):
        """ Simulate an aquatic environment.

        Only done if the fluid_dynamics flag is on in manager.

        NOTE: Fluid dynamics are only in place for capsules and boxes.

        Args:
            vel: three element list with velocity of flow hitting the robot
        """

        drag_coefficient = .1

        for body in self.bodies:
            body_vel = self.bodies[body].getLinearVel()
            rot = self.bodies[body].getRotation()

            vel_dif = [(v-b)*2. for v,b in zip(vel,body_vel)]

            # Iterate through the 6 faces of a body (assuming box x,y,z parameters)
            for i in xrange(6):
                area = self.surfaces[body][i]['area'] # Area of the surface
                norm = self.surfaces[body][i]['norm'] # Norm vector [x,y,z] of the surface

                adnorm = rotate3(rot,norm) # Reorient the normal to the bodies rotation in the world.

                component = adnorm[0]*vel_dif[0]+adnorm[1]*vel_dif[1]+adnorm[2]*vel_dif[2]  # Get the component of force perpendicular to surface.
                component *= drag_coefficient*area  # Compensating for the size of the surface.
                if(component < 0):
                    component = 0    # If less than zero, no drag force to apply (Surface is opposite to drag force.)
                force = self.world.impulseToForce(self.stepsize,[component*adnorm[0], component*adnorm[1], component*adnorm[2]])
                self.bodies[body].addForce([force[0],force[1],force[2]])  # Add force to the body from calculated surface.

    def add_impulse_to_body(self,key,impulse,vector=[0,1,0]):
        """ Add an impulse to a body.

        Args:
            key: body id to add force to
            impulse: the impulse to add to the body
            vector: axis along which to add the force to the body
        """ 
        force = self.world.impulseToForce(self.stepsize,[impulse*v for v in vector])
        self.bodies[key].addRelForce([force[0],force[1],force[2]]) 

        return force

    def get_rotation_of_body(self,key):
        """ Get the rotation in radians of the given body.

        Args:
            key: body id to get rotation of

        Returns:
            rotation vector of [X,Y,Z]
        """
        return GetRotationRad(self.bodies[key].getQuaternion())

    def get_end_positions(self,key):
        """ Get the endpoint positions for a capsule in world coordinates.

        Args:
            key: body id to get relative point positions of.

        Returns: 
            list of the two positions
        """
        pos = []
        pos.append(self.bodies[key].getRelPointPos((-self.bodies[key].length/2.,0.,0.)))
        pos.append(self.bodies[key].getRelPointPos((self.bodies[key].length/2.,0.,0.)))

        return pos

    def adjust_vert_position(self,key,vert_adjustment):
        """ Move the body up or down to a new vertical position based on the adjustment.

        Args:
            key: body id to set position of
            vert_position: vertical adjustment to make to the body

        """
        cur_pos = self.bodies[key].getPosition()
        self.bodies[key].setPosition((cur_pos[0],cur_pos[1]+vert_adjustment,cur_pos[2]))        
