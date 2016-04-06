'''
    File to handle visuals for OpenGL when viewing ODE evolved solutions.
'''

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from OpenGL.GL.shaders import *
from OpenGL.GL.framebufferobjects import *

from manager import ODEManager

from vector_ops import *

import numpy as np
import math

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
        self.lookx, self.looky, self.lookz = 0.0,0.0,0.0
        self.override = False
        self.posx,self.posy,self.posz = 0.0,0.0,0.0

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
        if not self.override:
            gluLookAt(x,y,z, self.lookx,self.looky,self.lookz, 0,1,0)
        else:
            gluLookAt(self.lookx+self.distance,y,self.lookz+self.distance, self.lookx,self.looky,self.lookz, 0,1,0)
#            gluLookAt(x,y,z, self.lookx,self.looky,self.lookz, 0,1,0)

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

class Shader():
    def __init__(self):
        self.is_built = False
        self.uniforms = {}

    def build(self):

        self.program = compileProgram(
        compileShader('''
            #version 110

            uniform mat4 camMatrix, shadowMatrix;
            uniform sampler2D texture;

            attribute vec3 position, normal;
            attribute vec2 texcoord;
            attribute float shininess;
            attribute vec4 specular;

            varying vec3 frag_position, frag_normal;
            varying vec2 frag_texcoord;
            varying float frag_shininess;
            varying vec4 frag_specular;

            void main()
            {
                vec4 eye_position = shadowMatrix * vec4(position, 1.0);
                gl_Position = camMatrix * eye_position;
                frag_position = eye_position.xyz;
                frag_normal   = (shadowMatrix * vec4(normal, 0.0)).xyz;
                frag_texcoord = texcoord;
                frag_shininess = shininess;
                frag_specular = specular;
            }
        ''',GL_VERTEX_SHADER),
        compileShader('''
            #version 110

            uniform mat4 camMatrix, shadowMatrix;
            uniform sampler2D texture;

            varying vec3 frag_position, frag_normal;
            varying vec2 frag_texcoord;
            varying float frag_shininess;
            varying vec4 frag_specular;

            const vec3 light_direction = vec3(0.408248, -0.816497, 0.408248);
            const vec4 light_diffuse = vec4(0.8, 0.8, 0.8, 0.0);
            const vec4 light_ambient = vec4(0.2, 0.2, 0.2, 1.0);
            const vec4 light_specular = vec4(1.0, 1.0, 1.0, 1.0);

            void main()
            {
                vec3 mv_light_direction = (shadowMatrix * vec4(light_direction, 0.0)).xyz,
                     normal = normalize(frag_normal),
                     eye = normalize(frag_position),
                     reflection = reflect(mv_light_direction, normal);

                vec4 frag_diffuse = texture2D(texture, frag_texcoord);
                vec4 diffuse_factor
                    = max(-dot(normal, mv_light_direction), 0.0) * light_diffuse;
                vec4 ambient_diffuse_factor
                    = diffuse_factor + light_ambient;
                vec4 specular_factor
                    = max(pow(-dot(reflection, eye), frag_shininess), 0.0)
                        * light_specular;
                
                gl_FragColor = specular_factor * frag_specular
                    + ambient_diffuse_factor * frag_diffuse;
            }
        ''',GL_FRAGMENT_SHADER),)
        self.is_built = True

        self.uniforms['camMatrix'] = glGetUniformLocation(self.program, 'camMatrix')
        self.uniforms['shadowMatrix'] = glGetUniformLocation(self.program, 'shadowMatrix')
        self.uniforms['shadowMap'] = glGetUniformLocation(self.program, 'shadowMap')
        self.uniforms['useShadow'] = glGetUniformLocation(self.program, 'useShadow')
        print self.uniforms

    def use(self):
        if not self.is_built:
            self.build()
        glUseProgram(self.program)

class ODEVisualizer:
    """An OpenGL Wrapper for the ODE Programs"""

    def __init__(self, manager, idle_callback, log_frames=-1, filepath="/Users/JMoore/Desktop/logging_frames/"):
#        self.idle_callback = idle_callback
        self.man = manager
        self.idle_callback = ""    
        self.initialized = False 

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
        glutInit(sys.argv)
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
        glutInitWindowSize(self.width, self.height)
        glutInitWindowPosition(0, 0)
        self.window = glutCreateWindow("ODE Instance")
        self.cam = Camera()
        self.light = Camera()
        self.cam.set_size(self.width, self.height)
        self.cam.distance=8
        self.light.set_size(4000, 4000)
        self.light.distance = 10
        self.light.rotx = 0.
#        self.light.roty = 10.
        self.shader = Shader()

        mat_specular = [ 1.0, 1.0, 1.0, 1.0 ];
        mat_shininess = [ 50.0 ];
        light_position = [ 1.0, 100.0, 100.0, 0.0 ];
        glClearColor (0.0, 0.0, 0.0, 0.0);
        glShadeModel (GL_SMOOTH);

        glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
        glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);

        glutMouseFunc (self.mouse_button)
        glutMotionFunc (self.mouse_move)
        glutReshapeFunc (self.cam.set_size) 
        glutKeyboardFunc (self._keyfunc)
        glutDisplayFunc (self.render)
        glutIdleFunc (self.render)
        glutMainLoop ()

    # set the callback function for the visualizer
    def set_callback(self,idle_callback):
        self.idle_callback = idle_callback

    def setup(self):
        self.initialized = True
        
        glClearColor(0,0,0,1.0);
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)

        self.fbo = glGenFramebuffers(1);
        self.shadowTexture = glGenTextures(1)

        glBindFramebuffer(GL_FRAMEBUFFER, self.fbo)

        w, h = self.light.size

        glActiveTexture(GL_TEXTURE5)
        glBindTexture(GL_TEXTURE_2D, self.shadowTexture)

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );

        glTexImage2D( GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, w, h, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, None)

        glDrawBuffer(GL_NONE)
        glReadBuffer(GL_NONE)

        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, self.fbo, 0)

        FBOstatus = glCheckFramebufferStatus(GL_FRAMEBUFFER)
        if FBOstatus != GL_FRAMEBUFFER_COMPLETE:
            print ("GL_FRAMEBUFFER_COMPLETE_EXT failed, CANNOT use FBO\n");

        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        #glActiveTexture(GL_TEXTURE0)

    def draw(self):

        # Draw the scene
        bodies_drawn = 0
        self._draw_plane(self.man.floor)
        for k,b in self.man.bodies.iteritems():
            self._draw_body(b)
            bodies_drawn += 1
        
        for k,g in self.man.terrain_geoms.iteritems():
            self._draw_body(g,color=[0.,0.2,0.2,1.])
            bodies_drawn += 1

        if self.log_frames == 1 and self.logging_on and bodies_drawn > 0:
            self.write_frame_to_file()

    def apply_camera(self, cam):
        cam.load_matrices()
        model_view = glGetDoublev(GL_MODELVIEW_MATRIX);
        projection = glGetDoublev(GL_PROJECTION_MATRIX);
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glMultMatrixd(projection)
        glMultMatrixd(model_view)
        glUniformMatrix4fv(self.shader.uniforms['camMatrix'], 1, False, glGetFloatv(GL_MODELVIEW_MATRIX))
        glLoadIdentity()

    def shadow_pass(self):
        glUniform1i(self.shader.uniforms['useShadow'], 0)

        glBindFramebuffer(GL_FRAMEBUFFER, self.fbo)
        glClear(GL_DEPTH_BUFFER_BIT)
        glCullFace(GL_FRONT)
        self.apply_camera(self.light)
        self.draw()
        glBindFramebuffer(GL_FRAMEBUFFER, 0)

    def final_pass(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Initialize
        glClearColor(0.8,0.8,0.9,0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        self.light.load_matrices()
        model_view = glGetDoublev(GL_MODELVIEW_MATRIX);
        projection = glGetDoublev(GL_PROJECTION_MATRIX);
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        bias = [ 0.5, 0.0, 0.0, 0.0,
                 0.0, 0.5, 0.0, 0.0,
                 0.0, 0.0, 0.5, 0.0,
                 0.5, 0.5, 0.5, 1.0]
        glLoadMatrixd(bias)
        glMultMatrixd(projection)
        glMultMatrixd(model_view)
        glUniformMatrix4fv(self.shader.uniforms['shadowMatrix'], 1, False, glGetFloatv(GL_MODELVIEW_MATRIX))

        glActiveTexture(GL_TEXTURE5)
        glBindTexture(GL_TEXTURE_2D, self.shadowTexture)
        glUniform1i(self.shader.uniforms['shadowMap'], 5)

        glUniform1i(self.shader.uniforms['useShadow'], 1);

        self.apply_camera(self.cam)
        glLoadIdentity()
        glCullFace(GL_BACK)
        
        self.draw()

    def render(self):
        self.idle_callback()
        if not self.initialized: self.setup()

        self.shader.use()
        self.shadow_pass()
        self.final_pass()
        glutSwapBuffers()

    def mouse_move(self, *args):
        self.cam.on_mouse_move(*args)
        self.light.on_mouse_move(*args)

    def mouse_button(self, b, *args):
        if b==0:
            self.light.on_mouse_button(b, *args)
        else:
            self.cam.on_mouse_button(b, *args)
    
    def set_camera_viewpoint(self,pos):
        """ Set the camera viewpoint to the specified position.

        Args:
            pos: position to look at [x,y,z]
        """
        self.cam.override = True
        self.cam.ex = pos[0]
        self.cam.ey = pos[1]
        self.cam.lookx = pos[0]
        self.cam.looky = pos[1]

    def _draw_plane(self,geom):
        """Draw a grid plane for reference purposes.
        
        Arguments:
        geom - plane object
        """
        color = [0.2,0.2,0.2,1.]
        glColor4f(0.2,0.2,0.2,1.)   
    
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
        glTranslate(d*p[0], d*p[1], d*p[2])
        glRotate(-theta, c[0], c[1], c[2])
        gluDisk(quad, 0, 40, 40, 1)
        glPopMatrix()
        # Draw a grid floor.
        glColor3f(1.0,1.0,1.0)
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
        
        # Draw a sphere at the origin.
        # Process:
        # Set the color of the material, draw sphere.
        glPushMatrix()
        glColor4f(1.0,0.0,0.0,1.0)
        glutSolidSphere(0.025,20,20)
        glPopMatrix()
       
        for i in range(1,5):
            # Draw a second sphere at the positive X direction.
            glPushMatrix()
            glTranslatef(i,0.0,0.0)
            glColor4f(0.0,1.0,0.0,1.0)
            glutSolidSphere(0.025,20,20)
            glPopMatrix()

    # draw_body
    def _draw_body(self, body, color=[0.5,0.5,0.5,1.], shiny=True):
        """Draw an ODE body.
        """
#        color = [0.5,0.5,0.5,1.]
        glColor4f(color[0],color[1],color[2],color[3])
        glMaterialfv(GL_FRONT,GL_DIFFUSE,color)
        glMaterialfv(GL_FRONT,GL_SPECULAR,[1.0,1.0,1.0])
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
