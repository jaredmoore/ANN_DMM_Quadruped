'''
    File to handle visuals for OpenGL when viewing ODE evolved solutions.
'''

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

from manager import ODEManager

from vector_ops import *

import math

import numpy as np

from PIL import Image

CAPSULE_SLICES = 16
CAPSULE_STACKS = 12

class Camera:
    def __init__(self):
        self.rotx, self.roty = math.pi/4, math.pi/4.
        self.distance = 100
        self.moving = False
        self.ex, self.ey = 0, 0
        self.size = (800, 600)

    def load_matrices(self):
        glViewport(0, 0, *self.size)
        y = math.cos(self.roty) * self.distance
        x = math.sin(self.roty) * math.cos(self.rotx) * self.distance
        z = math.sin(self.roty) * math.sin(self.rotx) * self.distance

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, self.size[0]/float(self.size[1]), 1, 1000)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(x,y,z, 0,0,0, 0,1,0)

    def on_mouse_button (self, b, s, x, y):
        self.moving = not s
        self.ex, self.ey = x, y
        if b in [3, 4]:
            dz = (1 if b == 3 else -1)
            self.distance += self.distance/15.0 * dz;

    def on_mouse_move(self, x, y, z = 0):
        if self.moving:
            self.rotx += (x-self.ex) / 300.0
            self.roty += -(y-self.ey) / 300.0
            self.ex, self.ey = x, y

    def set_size(self, w, h):
        self.size = w, h

class ODEVisualizer:
    """An OpenGL Wrapper for the ODE Programs"""

    def __init__(self, manager, idle_callback, log_frames=-1, filepath="/Users/JMoore/Desktop/logging_frames/"):
#        self.idle_callback = idle_callback
        self.man = manager
        self.idle_callback = ""    

        self.paused = 1
        self.step = 0 

        self.eye = [4.8,4.8,4.8]
        self.center = [0.5,0.5,0.]

        self.width = 1024
        self.height = 768

        # Decide whether to log the video or not.
        self.log_frames = log_frames
        self.logging_on = False # Don't log paused frames!

        self.filepath = filepath
        self.img_num = 0

    def start_visuals(self):
        """ Start the visuals.
        """
        # Initialize Glut
        glutInit ([])
    
        # Open a window
        glutInitDisplayMode (GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STENCIL | GLUT_ALPHA)

        # Setup the camera
        self.cam = Camera()
        self.cam.set_size(self.width, self.height)
        self.cam.distance=8
        
        x = 0
        y = 0
        glutInitWindowPosition (x, y);
        glutInitWindowSize (self.width, self.height);
        glutCreateWindow ("testode")
    
        self.init_GL()      
 
        glutMouseFunc (self._mouse_button)
        glutMotionFunc (self._mouse_move)
        glutReshapeFunc (self.cam.set_size)
        glutKeyboardFunc (self._keyfunc)
        glutDisplayFunc (self._drawfunc)
        glutIdleFunc (self._idlefunc)
        glutMainLoop ()

    # set the callback function for the visualizer
    def set_callback(self,idle_callback):
        self.idle_callback = idle_callback

    def init_GL(self):
        """ Initialize OpenGL """
        # Initialize
        glClearColor(1.,1.,1.,0) 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
       
        # Light source
#        glEnable(GL_LIGHTING)
#        glLightfv(GL_LIGHT1,GL_POSITION,[0,10,10,0])
#        glLightfv(GL_LIGHT1,GL_DIFFUSE,[1.,1,1,1])
#        glLightfv(GL_LIGHT1,GL_SPECULAR,[0,0,0,.8])
#        glEnable(GL_LIGHT1)

        glEnable(GL_LIGHTING)
        glLightfv(GL_LIGHT1,GL_POSITION,[10,1000,10,0])
        glLightfv(GL_LIGHT1,GL_DIFFUSE,[1.,1,1,1])
        glLightfv(GL_LIGHT1,GL_SPECULAR,[1.0,1.0,1.0,.3])
        glEnable(GL_LIGHT1)

        # Smooth highlighting of the individual components.
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)
        glEnable(GL_NORMALIZE)
        
        # Shading
        glShadeModel(GL_SMOOTH)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        glClearColor(1.0,1.0,1.0,1.0)

    # prepare_GL
    def prepare_GL(self):
        """Prepare drawing.
        """
        pass

    def _mouse_move(self, *args):
        self.cam.on_mouse_move(*args)

    def _mouse_button(self, b, *args):
            self.cam.on_mouse_button(b, *args)

    def set_camera_viewpoint(self,pos):
        """ Set the camera viewpoint to the specified position.

        Args:
            pos: position to look at [x,y,z]
        """
#        diff = [n-o for n,o in zip(pos,self.center)]
#        glTranslated(diff[0],diff[1],diff[2])
#        self.eye = [e-d for e,d in zip(self.eye,diff)]
        self.center = pos
        self.eye = [pos[0] + 4.8, self.eye[1], pos[2] + 4.8]

    def _draw_plane(self,geom,all):
        """Draw a grid plane for reference purposes.
        
        Arguments:
        geom - plane object
        all - whether to include the grid and sphere
        """
        color = [0.2,0.2,0.2,1.]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        
        quad = gluNewQuadric()
        gluQuadricTexture(quad,GL_TRUE)
        
        p = geom.getParams()[0] # the normal vector to the plane
        d = geom.getParams()[1] # the distance to the origin
        q = (0.0,0.0,1.0)       # the normal vector of default gluDisks
        
        # calculate the cross product to get the rotation axis
        c = cross(p,q)
        # calculate the angle between default normal q and plane normal p
        theta = acosdot3(p,q) / pi * 180
        
        # rotate the plane
        glPushMatrix()
        color = [.1,.1,.1,1.]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        glTranslate(d*p[0], d*p[1], d*p[2])
        glRotate(-theta, c[0], c[1], c[2])
        gluDisk(quad, 0, 40, 40, 1)
        glPopMatrix()

        if not all:
            return

    def _draw_plane_extras(self):
        # Draw a sphere at the origin.
        # Process:
        # Set the color of the material, draw sphere.
        glPushMatrix()
        color = [1.0,0.,0.,1.]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        glutSolidSphere(0.025,20,20)
        glPopMatrix()
        
        # Draw a grid floor.
        glColor3f(1.0,1.0,1.0)
        color = [1.0,1.0,1.0,1.0]
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        glLineWidth(1.5)
        glBegin(GL_LINES)
        grid_space = 1.
        grid_range = 25.
        for i in np.arange(-grid_range,grid_range,grid_space):
            glVertex3f(i,0,grid_range)
            glVertex3f(i,0,-grid_range)
            glVertex3f(grid_range,0,i)
            glVertex3f(-grid_range,0,i)
        glEnd()
        

    # draw_body
    def _draw_body(self, body,color=[0.5,0.5,0.5,1.]):
        """Draw an ODE body.
        """
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        glMaterialfv(GL_FRONT,GL_SPECULAR,color)
        glMaterialfv(GL_FRONT,GL_SHININESS,16) 
        x,y,z = body.getPosition()
        R = body.getRotation()
        rot = makeOpenGLMatrix(R, [x,y,z])
        #rot2 = [R[0], R[3], R[6], 0.,
        #   R[1], R[4], R[7], 0.,
        #   R[2], R[5], R[8], 0.,
        #   x, y, z, 1.0]
        #if rot != rot2:
        #    print(rot)
        #    print(rot2)
        #    print("The two are not equal.")
        glPushMatrix()
        glMultMatrixd(rot)
        if body.shape=="box":
            sx,sy,sz = body.boxsize
            glScalef(sx, sy, sz)
            glutSolidCube(1)
        elif body.shape == "capsule":
            cylHalfHeight = body.length / 2.0
            glBegin(GL_QUAD_STRIP)
            for i in range(0, CAPSULE_SLICES + 1):
                angle = i / float(CAPSULE_SLICES) * 2.0 * pi
                ca = cos(angle)
                sa = sin(angle)
                glNormal3f(ca, sa, 0)
                glVertex3f(body.radius * ca, body.radius * sa, cylHalfHeight)
                glVertex3f(body.radius * ca, body.radius * sa, -cylHalfHeight)
            glEnd()
            glTranslated(0, 0, cylHalfHeight)
            glutSolidSphere(body.radius, CAPSULE_SLICES, CAPSULE_STACKS)
            glTranslated(0, 0, -2.0 * cylHalfHeight)
            glutSolidSphere(body.radius, CAPSULE_SLICES, CAPSULE_STACKS) 
        elif body.shape == "cylinder":
            cylHalfHeight = body.length / 2.0
            quadratic = gluNewQuadric()
            gluCylinder(quadratic, body.radius, body.radius, body.length, CAPSULE_SLICES, CAPSULE_STACKS)      # to draw the lateral parts of the cylinder;
            glTranslated(0,0,cylHalfHeight)
            gluDisk(quadratic, 0., body.radius, CAPSULE_SLICES, CAPSULE_STACKS)
            glTranslated(0,0,-cylHalfHeight)
            gluDisk(quadratic, 0., body.radius, CAPSULE_SLICES, CAPSULE_STACKS)
            # glBegin(GL_QUAD_STRIP)
            # for i in range(0, CAPSULE_SLICES + 1):
            #     angle = i / float(CAPSULE_SLICES) * 2.0 * pi
            #     ca = cos(angle)
            #     sa = sin(angle)
            #     glNormal3f(ca, sa, 0)
            #     glVertex3f(body.radius * ca, body.radius * sa, cylHalfHeight)
            #     glVertex3f(body.radius * ca, body.radius * sa, -cylHalfHeight)
            # glEnd()
            # glTranslated(0,0,cylHalfHeight)
            # gluDisk(quadratic, 0., body.radius, CAPSULE_SLICES, CAPSULE_STACKS)
        elif body.shape == "sphere":
            glutSolidSphere(body.radius, CAPSULE_SLICES, CAPSULE_STACKS)
        glPopMatrix()

    # keyboard callback
    def _keyfunc (self,c, x, y):
        global SlowMotion

        # Set the speed of simulation
        if c >= '0' and c <= '9':
            SlowMotion = 4 * int(c) + 1
        # Pause the simulation
        elif c == 'p' or c == 'P':
            self.paused = not self.paused
            self.logging_on = not self.logging_on
        # Step the simulation
        elif c == 's' or c == 'S':
            self.step = not self.step
        # Exit the simulation
        elif c == 'q' or c == 'Q':
            sys.exit (0)

    # draw callback
    def _drawfunc (self):
        # Draw the scene
        glClearColor(.0,.0,.0,1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT)

        # Projection
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective (45,1.3333,0.2,20)
        
        # Initialize ModelView matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
    
        # View transformation
        gluLookAt (self.eye[0], self.eye[1], self.eye[2], self.center[0], self.center[1], self.center[2], 0, 1, 0)

#        self.prepare_GL()
        self._draw_plane(self.man.floor,1)
        self._draw_plane_extras()
        bodies_drawn = 0
        for k,b in self.man.bodies.iteritems():
            self._draw_body(b)
            bodies_drawn += 1
      
#
        for k,g in self.man.terrain_geoms.iteritems():
            self._draw_body(g, color=g.color)
            bodies_drawn += 1
        glDisable(GL_CULL_FACE)

        glutSwapBuffers ()

        if self.log_frames == 1 and self.logging_on and bodies_drawn > 0:
            self.write_frame_to_file()

    # idle callback
    # main GLUT callback function
    # analagous to main function for graphical displays
    # 
    # Calls the idle_callback function provided by the caller.
    def _idlefunc (self):
        self.idle_callback()
        glutPostRedisplay()

    def write_frame_to_file(self):
        """ Write the current OpenGL frame to file. """
        # Get the image from the OpenGL buffer
        buffer = (GLubyte*(3*self.width*self.height))(0)
        glReadPixels(0,0,self.width,self.height,GL_RGB,GL_UNSIGNED_BYTE,buffer)

        # Use PIL to convert raw RGB buffer and flip right way up.
        image = Image.fromstring(mode="RGB",size=(self.width,self.height),data=buffer)
        image = image.transpose(Image.FLIP_TOP_BOTTOM)

        # Save image to disk.
        image.save(self.filepath+"output_"+str(self.img_num).zfill(5)+".png")
        self.img_num += 1
