'''
    Manager implementation to create and manage an ODE instance.
'''
import math
import ode
from placement import Placement

ANG_TO_RAD = math.pi/180.
RAD_TO_ANG = 180./math.pi

class ValODEManager:
    """An instance manager for ODE"""

    def __init__(self, logfile, stepsize=0.005):
        self.logfile = logfile 
        
        # Create a world object
        self.world = ode.World()
        self.world.setGravity( (0,0,0) )
        self.world.setERP(0.5)
        self.world.setCFM(1E-4)
        
        # Create a space object
        self.space = ode.Space()

        # Create a plane geom which prevent the objects from falling forever
        self.floor = ode.GeomPlane(self.space, (0,1,0), 0)

        # A list of ODE Bodies
        self.bodies = {}

        # A list of geoms for the bodies
        self.geoms = {}

        # Set the stepsize for the instance.
        self.stepsize = stepsize

        self.positions = []
        self.quaternions = []
        self.curstep = 0
        self.read_file()   
 
    def read_file(self):
        """ Read in the positions and quaternions from a file. """
        with open(self.logfile, "r") as f:
            i = 0
            for line in f:
                if i % 2 == 0:
                    self.positions.append(map(float,line.split(',')))
                else:
                    self.quaternions.append(map(float,line.split(',')))
                i += 1

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
   
    def create_capsule(self,key,density,length,radius,pos,base=0,rot=0):
        """Creates a capsule body and corresponding geom.
        
        Arguments:
        key : number id to assign to the capsule
        density : density of the given body
        length : length of the capsule
        radius : radius of the capsule
        pos : position of the center of the capsule (x,y,z list)
        base : place new object at negative end of base object

        """

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
        
        self.bodies[key] = body
        self.geoms[key] = geom

        if(base):
            Placement.place_object(self.bodies[base],body)
    
    def create_box(self,key,dim,pos):
        """Create a box.
       
        Arguments:
        @param key: number id to assign to the box
        @type key: int
        @param dim: list of l,w,h dimension for the box
        @type: [int,int,int]
        @param pos: list of x,y,z position for center of box
        @type: [int,int,int]

        """

        body, geom = self.self_create_box(10, dim[0],dim[1],dim[2])
        body.setPosition((pos[0],pos[1],pos[2]))
        self.bodies[key] = body
        self.geoms[key] = geom

    def create_universal(self,key,pos,bod_keys,axis1 = [1,0,0],axis2 = [0,0,1],
        loStop1 = -ode.Infinity, hiStop1 = ode.Infinity,
        loStop2 = -ode.Infinity, hiStop2 = ode.Infinity):
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

        """
        pass

    def actuate_hinge(self, key, vel):
        """
        Move a hinge by a specified velocity.

        Arguments:
        @param key: number id associated with the hinge
        @type key: int
        @param vel: velocity to move the joint (radians).
        @type vel: float
        """
        pass

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
        pass

    def actuate_universal(self, key, pos_1, pos_2):
        """
        Move a universal joint by a specified velocity(s).

        Arguments:
        @param key: number id associated with the joint.
        @type key: int
        @param pos_1: velocity to move the joint (radians).
        @type pos_1: float
        @param pos_2: velocity of second hinge (radians).
        @type pos_2: float

        """
        pass

    def get_body_position(self,key):
        """ Get the position of a body.

        Args:
            key: id associated with the body
        Returns:
            position of the body
        """
        return self.bodies[key].getPosition()
    
    def delete_joints(self):
        """ Empty """
        pass

    def delete_bodies(self):
        """ Delete the bodies held in the manager."""
        self.geoms.clear()
        self.bodies.clear()
        self.curstep = 0

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

    # Collision callback
    def near_callback(self,args, geom1, geom2):
        """Callback function for the collide() method.

        This function checks if the given geoms do collide and
        creates contact joints if they do.
        """
        pass

    def set_body_positions(self):
        """ Get the body positions from the logfile and place them in the world. """

        p = self.positions[self.curstep]
        q = self.quaternions[self.curstep]

        for b,(i,j) in enumerate(zip(xrange(0,len(p),3),xrange(0,len(q),4))):
            print(i,p[i],p[i+1],p[i+2])
            self.bodies[b].setPosition((p[i],p[i+1],p[i+2]))
            self.bodies[b].setQuaternion((q[i],q[i+1],q[i+2],q[i+3]))
        self.curstep += 1

    # Step function for the manager.
    def step_physics(self,callback):
        self.space.collide((self.world,self.contactgroup), self.near_callback)

        self.world.step(self.stepsize)

        self.contactgroup.empty()

    # Step the physics the defined number of times.
    def step(self,callback, steps):
        for i in range(steps):
            pass
        self.set_body_positions()
        #self.step_physics(callback)
