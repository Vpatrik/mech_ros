"""
Display image as an OpenGL object
added 3D models + videos + cube
added probability
added averaging
added GLFW


Resolution 1280x720

Bugs:
    -   alpha channel
    -   Bulgarian version
    -   Video + sound synchronization

ToDo:
    -   3D animation
    -   GPU computing

In Progress:
    -   Video with sound
"""

from OpenGL.GL import *
from OpenGL.GLU import *
import glfw
import cv2
from PIL import Image
import numpy as np
from objloader_V1 import *
from threading import Thread
import threading
import time
from Brush import *

# Constants
WIDTH = 1280
HEIGHT = 720
MARKER_SIZE = 150
INVERSE_MATRIX = np.array([[1.0, 1.0, 1.0, 1.0],
                           [-1.0, -1.0, -1.0, -1.0],
                           [-1.0, -1.0, -1.0, -1.0],
                           [1.0, 1.0, 1.0, 1.0]])

# Marker IDs
STModelID = 0
r2d2ID = 1
bertID = 2
brush_gID = 3
brush_clearID = 4
videoID = 5
cubeID = np.int8([6, 7, 8, 9, 10, 11])
gateModelID = 12


cube_size = 200


# Load camera matrix and distortion coefficients
with np.load('camCal_1280x720_MS.npz') as X:
    mtx, dist, _, _ = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

# Marker parameters
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
detector_params = cv2.aruco.DetectorParameters_create()
detector_params.cornerRefinementMethod = 1
# detector_params.adaptiveThreshWinSizeStep = 5
# detector_params.polygonalApproxAccuracyRate = 0.08
# detector_params.perspectiveRemovePixelPerCell = 10
# detector_params.perspectiveRemoveIgnoredMarginPerCell = 0.2
# detector_params.adaptiveThreshConstant = 5

aspect = (WIDTH * mtx[1, 1]) / (HEIGHT * mtx[0, 0])
fovy = 2 * np.arctan(0.5 * 922 / mtx[1, 1]) * 180 / np.pi
bg_dist = 45


class VidPlayer:

    def __init__(self, video):
        self.video = video
        self.isPlaying = False
        self.cur_frame = np.uint8()

    def start(self):
        Thread(target=self.play, args=(), daemon=True).start()

    def play(self):
        self.isPlaying = True
        nof = self.video.get(cv2.CAP_PROP_FRAME_COUNT)
        fps = self.video.get(cv2.CAP_PROP_FPS)
        frame_counter = 0
        while True:
            if frame_counter == nof:
                self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                self.isPlaying = False
                break

            ret, self.cur_frame = self.video.read()
            frame_counter += 1
            time.sleep(1/(fps*1.5))

    def get_current_frame(self):
        return self.cur_frame


class OpenGLAR3D:
    # constants

    def __init__(self):
        # Initialize stream
        self.cap = cv2.VideoCapture(0)
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.cap.set(cv2.CAP_PROP_FOCUS, 0)

        # initialise texture
        self.texture_background = None

        # Load images
        self.r2d2_pic = cv2.imread('Images/r2d2.png', cv2.IMREAD_UNCHANGED)
        self.brush_g_pic = cv2.imread('Images/brush_g.png', cv2.IMREAD_UNCHANGED)
        self.bert_pic = cv2.imread('Images/einstein.png', cv2.IMREAD_UNCHANGED)

        # Assign video players
        self.polar_vid = cv2.VideoCapture('Videos/polar.MOV')
        self.roboti_vid_player = VidPlayer(self.polar_vid)

        self.draw = True
        self.cube_rendered = False
        self.image = None

        # Marker probability
        self.last_ids_mtx = np.zeros((50, 10), dtype=np.int8)
        self.prob_vect = np.zeros(50, dtype=np.float16)
        self.last_corners = np.zeros((50, 1, 4, 2), dtype=np.float32)
        self.corners_avg_buffer = np.zeros((50, 3, 4, 2), dtype=np.float32)

        self.brush_strokes = np.empty((500, 7), dtype=np.float32)

        self.brush_strokes_counter = 0
        self.brush_rvecs = np.array([[3.14, 0, 0]])
        self.brush_g_last = np.empty((1, 3), dtype=np.float32)
        self.brush_b_last = np.empty((1, 3), dtype=np.float32)

    def _init_gl(self, width, height):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glDepthFunc(GL_LESS)  # specify the value used for depth buffer comparisons
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)

        glEnable(GL_MULTISAMPLE)

        glMatrixMode(GL_PROJECTION)  # specify which matrix is the current matrix
        glLoadIdentity()
        gluPerspective(fovy, aspect, 0.1,
                       500.0 * MARKER_SIZE)  # field of view, aspect ration, distance to near clipping plane, distance to far clipping plane

        # Load models
        self.earth_model = OBJ('Models/Earth1/earth.obj')
        self.ST_model = OBJ('Models/ST/ST.obj')
        self.gate_model = OBJ('Models/Gate/gate_obj.obj')

        # Load brushes
        self.brush_g = Brush(self.brush_g_pic)

        # assign texture
        self.texture_background = glGenTextures(1)  # generate texture names
        self.r2d2_pic_texture = glGenTextures(1)
        self.roboti_vid_texture = glGenTextures(1)
        self.bert_texture = glGenTextures(1)

        _, frame = self.cap.read()
        self.init_background(frame)

    def glfw_main_loop(self):
        while not glfw.window_should_close(self.main_window):
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

            # get image from video stream
            _, frame = self.cap.read()
            self.image = np.fliplr(frame)
            #
            ids, rvecs, tvecs = self.find_marker(self.image)

            self.draw_background(self.image)

            self.draw_ar(ids, rvecs, tvecs)

            glfw.swap_buffers(self.main_window)
            glfw.poll_events()

    def find_marker(self, image):
        rvecs = None
        tvecs = None

        # image = cv2.resize(image, (640, 360), interpolation=cv2.INTER_AREA)
        corners, ids, rejected = cv2.aruco.detectMarkers(image, dictionary, parameters=detector_params)
        # corners = np.dot(corners, 2)

        # <editor-fold desc = "Probability" >
        if not isinstance(ids, type(None)):
            self.last_ids_mtx[ids, 0] = 1
            self.last_corners[ids, 0, :, :] = np.float32(corners)

        self.prob_vect = np.sum(self.last_ids_mtx, 1) / np.size(self.last_ids_mtx, 1)
        self.last_ids_mtx = np.pad(self.last_ids_mtx, ((0, 0), (1, 0)), mode='constant')[:, :-1]

        if np.any(self.prob_vect > 0.3) & np.any(self.prob_vect < 1):
            prob_idx = np.array(np.where(np.logical_and(self.prob_vect > 0.3, self.prob_vect < 1)))[0]
            # print("P1 " + str(prob_idx))
            if not isinstance(ids, type(None)):
                to_delete = []
                # print(np.size(prob_idx, axis=1))
                for i in range(0, np.size(prob_idx)):
                    if prob_idx[i] in ids:
                        to_delete.append(i)
                prob_idx = np.delete(prob_idx, to_delete)
            # print("P2 " + str(prob_idx))
            if isinstance(ids, type(None)):
                ids = np.array([[0]])
                erase = True
            else:
                erase = False
            # print("Mezi1 " + str(ids))
            for i in range(0, np.size(prob_idx)):
                ids = np.append(ids, [[prob_idx[i]]], axis=0)
                # print(np.float32([self.last_corners[prob_idx[i], 0, :, :]]))
                corners.append(np.float32([self.last_corners[prob_idx[i], 0, :, :]]))
            # print("Mezi1 " + str(ids))
            if erase:
                ids = np.delete(ids, ids[0], axis=0)
            if np.size(ids) == 0:
                ids = None
        # </editor-fold >

        # Averaging
        if not isinstance(ids, type(None)):
            self.corners_avg_buffer[ids, 0, :, :] = corners
            corners_avg = np.sum(self.corners_avg_buffer, 1)/3
            valid_avg = np.sum(np.prod(self.corners_avg_buffer, 1), (1, 2))

            for i in range(0, np.size(ids, 0)):
                if valid_avg[ids[i]] != 0:
                    corners[i] = corners_avg[ids[i][0], :, :]

        self.corners_avg_buffer = np.pad(self.corners_avg_buffer, ((0, 0), (1, 0), (0, 0), (0, 0)), mode='constant')[:, :-1, :, :]

        if not isinstance(ids, type(None)) or self.brush_strokes_counter != 0:

            if not isinstance(ids, type(None)):
                rvecs, tvecs, origin = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, mtx, dist)
                rvecs = rvecs[:, 0, :]
                tvecs = tvecs[:, 0, :]
                ids = ids[:, 0]

                for i in range(0, np.size(ids)):
                    if brush_gID == ids[i]:
                        if self.draw:
                            brush_tvecs = tvecs[ids.tolist().index(brush_gID)]
                            dd = np.linalg.norm(self.brush_g_last - brush_tvecs)
                            if dd > 20:
                                self.brush_strokes = np.roll(self.brush_strokes, 1, axis=0)
                                self.brush_strokes[0, 0] = brush_gID
                                self.brush_strokes[0, 1:4] = brush_tvecs
                                self.brush_strokes[0, 4:7] = [3.14, 0.0, 0.0]

                                self.brush_strokes_counter += 1
                                self.brush_g_last = brush_tvecs


                # Add previous brush strokes

                if self.brush_strokes_counter != 0:
                    ids = np.append(ids, self.brush_strokes[0:self.brush_strokes_counter, 0])
                    tvecs = np.append(tvecs, self.brush_strokes[0:self.brush_strokes_counter, 1:4], axis=0)
                    rvecs = np.append(rvecs, self.brush_strokes[0:self.brush_strokes_counter, 4:7], axis=0)

            else:
                ids = np.array([0])
                tvecs = np.array([[0, 0, 0]])
                rvecs = np.array([[0, 0, 0]])

                # DUPLICITNI OPRAVIT
                if self.brush_strokes_counter != 0:
                    ids = np.append(ids, self.brush_strokes[0:self.brush_strokes_counter, 0])
                    tvecs = np.append(tvecs, self.brush_strokes[0:self.brush_strokes_counter, 1:4], axis=0)
                    rvecs = np.append(rvecs, self.brush_strokes[0:self.brush_strokes_counter, 4:7], axis=0)
                ids = np.delete(ids, ids[0], axis=0)
                tvecs = np.delete(tvecs, tvecs[0], axis=0)
                rvecs = np.delete(rvecs, rvecs[0], axis=0)

            # Sorting according to z distanced
            inds = np.flipud(np.argsort(tvecs[:, 2], 0))
            tvecs = tvecs[inds]
            rvecs = rvecs[inds]
            ids = ids[inds]

        return ids, rvecs, tvecs

    def get_view_matrix(self, rvecs, tvecs):
        model_rvecs = rvecs
        model_tvecs = tvecs
        rmtx = cv2.Rodrigues(model_rvecs)[0]

        view_matrix = np.array([[rmtx[0, 0], rmtx[0, 1], rmtx[0, 2], model_tvecs[0]],
                                [rmtx[1, 0], rmtx[1, 1], rmtx[1, 2], model_tvecs[1]],
                                [rmtx[2, 0], rmtx[2, 1], rmtx[2, 2], model_tvecs[2]*0.95],
                                [0.0, 0.0, 0.0, 1.0]])

        view_matrix = np.transpose(view_matrix * INVERSE_MATRIX)
        # print(view_matrix)
        return view_matrix

    def draw_cube_model(self, cube_size, model, id, rvec, tvec, scale, x, y, z, rx, ry, rz):
        if cubeID[0] == id:  # 6
            view_matrix = self.get_view_matrix(rvec, tvec)
            self.draw_model(model, view_matrix, scale, x, y, -cube_size/2 + z, 0, 0, 0)
        elif cubeID[1] == id:  # 7
            view_matrix = self.get_view_matrix(rvec, tvec)
            self.draw_model(model, view_matrix, scale, x, y, -cube_size/2, 0, 90, 0)
        elif cubeID[2] == id:  # 8
            view_matrix = self.get_view_matrix(rvec, tvec)
            self.draw_model(model, view_matrix, scale, x, y, -cube_size/2, 0, 180, 0)
        elif cubeID[3] == id:  # 9
            view_matrix = self.get_view_matrix(rvec, tvec)
            self.draw_model(model, view_matrix, scale, x, y, -cube_size/2, 0, 270, 0)
        elif cubeID[4] == id:  # 10
            view_matrix = self.get_view_matrix(rvec, tvec)
            self.draw_model(model, view_matrix, scale, x, -z, y - cube_size/2, 90, 90, 0)
        # elif cubeID[5] == id:  # 11
        #     view_matrix = self.get_view_matrix(rvec, tvec)
        #     self.draw_model(model, view_matrix, scale, x, z, -y - cube_size/2, -90, 0, 0)

    def draw_ar(self, ids, rvecs, tvecs):
        if not isinstance(ids, type(None)):
            # a = time.time()
            for i in range(0, np.size(ids)):
                if STModelID == ids[i]:
                    view_matrix = self.get_view_matrix(rvecs[i], tvecs[i])
                    self.draw_model(self.ST_model, view_matrix, 500, 0, -500, 0, 0, 0, 0)

                if gateModelID == ids[i]:
                    view_matrix = self.get_view_matrix(rvecs[i], tvecs[i])
                    self.draw_model(self.gate_model, view_matrix, 7, 0, -200, 0, 0, 0, 0)

                elif r2d2ID == ids[i]:
                    view_matrix = self.get_view_matrix(rvecs[i], tvecs[i])
                    self.draw_picture(self.r2d2_pic, view_matrix, 190, 270, 0, 0, 0, self.r2d2_pic_texture)

                elif bertID == ids[i]:
                    view_matrix = self.get_view_matrix(rvecs[i], tvecs[i])
                    self.draw_picture(self.bert_pic, view_matrix, 190, 270, 0, 0, 0, self.r2d2_pic_texture)

                elif brush_gID == ids[i]:
                    view_matrix = self.get_view_matrix(self.brush_rvecs, tvecs[i])
                    self.draw_brush(self.brush_g, view_matrix, 0, 0, 0)

                elif brush_clearID == ids[i]:
                    self.brush_strokes_counter = 0

                elif videoID == ids[i]:
                    if not self.roboti_vid_player.isPlaying:
                        self.roboti_vid_player.start()

                    view_matrix = self.get_view_matrix(rvecs[i], tvecs[i])
                    self.draw_video(self.roboti_vid_player, view_matrix, 300, 250, self.roboti_vid_texture)

                elif np.size(np.intersect1d(cubeID, ids[i])):
                    if not self.cube_rendered:
                        self.draw_cube_model(cube_size, self.earth_model, ids[i], rvecs[i], tvecs[i], 0.8, 0, 0, 0, 0, 0, 0)
                        self.cube_rendered = True

        self.cube_rendered = False

    def draw_video(self, vid_player, view_matrix, width, height, textureID):
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        frame = vid_player.get_current_frame()

        if np.size(frame) == 1:
            return

        # convert image to OpenGL texture format
        frame = Image.fromarray(frame)
        ix = frame.size[0]
        iy = frame.size[1]
        frame = frame.tobytes("raw", "BGRX", 0, -1)

        # create background texture
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D,
                      textureID)  # bind a named texture to a texturing target (target, texture)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                     frame)  # specify a two-dimensional texture image
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                        GL_NEAREST)  # set texture parameters(target, parameter name, parameter value)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

        glPushMatrix()
        glLoadMatrixd(view_matrix)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0)  # set the current texture coordinates
        glVertex3f(-width / 2, -height / 2, 0.0)
        glTexCoord2f(0, 1)
        glVertex3f(-width / 2, height / 2, 0.0)
        glTexCoord2f(1, 1)
        glVertex3f(width / 2, height / 2, 0.0)
        glTexCoord2f(1, 0)
        glVertex3f(width / 2, -height / 2, 0.0)
        glEnd()
        glPopMatrix()
        glDisable(GL_TEXTURE_2D)

    def draw_brush(self, picture, view_matrix, x, y, z):
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        glLoadMatrixd(view_matrix)
        glTranslatef(x, y, z)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_TEXTURE_2D)

        glCallList(picture.gl_list)

        glDisable(GL_BLEND)
        glDisable(GL_TEXTURE_2D)

    def draw_model(self, model, view_matrix, scale, x, y, z, rx, ry, rz):
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        glPushMatrix()
        glLoadMatrixd(view_matrix)

        glTranslatef(x, y, z)
        glScale(scale, scale, scale)  # Scale model

        # Rotate model
        glRotatef(rx, 1, 0, 0)
        glRotatef(ry, 0, 1, 0)
        glRotatef(rz, 0, 0, 1)

        glEnable(GL_TEXTURE_2D)
        glCallList(model.gl_list)
        glDisable(GL_TEXTURE_2D)
        glColor3f(1, 1, 1)
        glPopMatrix()

    def draw_picture(self, picture, view_matrix, width, height, x, y, z, textureID):
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        blend = False
        # convert image to OpenGL texture format
        picture = Image.fromarray(picture)
        ix = picture.size[0]
        iy = picture.size[1]
        if np.shape(picture)[2] == 4:
            picture = picture.tobytes("raw", "BGRA", 0, -1)
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            blend = True
        else:
            picture = picture.tobytes("raw", "BGRX", 0, -1)

        # create background texture
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D,
                      textureID)  # bind a named texture to a texturing target (target, texture)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE,
                     picture)  # specify a two-dimensional texture image
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                        GL_NEAREST)  # set texture parameters(target, parameter name, parameter value)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

        glPushMatrix()
        glLoadMatrixd(view_matrix)
        glTranslatef(x, y, z)

        glBegin(GL_QUADS)
        glTexCoord2f(0, 0)  # set the current texture coordinates
        glVertex3f(-width/2, -height/2, 0.0)
        glTexCoord2f(0, 1)
        glVertex3f(-width/2, height/2, 0.0)
        glTexCoord2f(1, 1)
        glVertex3f(width/2, height/2, 0.0)
        glTexCoord2f(1, 0)
        glVertex3f(width/2, -height/2, 0.0)
        glEnd()

        if blend:
            glDisable(GL_BLEND)

        glDisable(GL_TEXTURE_2D)

        # glLineWidth(2)
        # glBegin(GL_LINES)
        # i = 0
        # for edge in edges:
        #     for vertex in edge:
        #         glColor3fv(colors[i])
        #         glVertex3fv(verticies[vertex])
        #     i += 1
        # glEnd()
        # glColor3fv((1, 1, 1))

        glPopMatrix()

    def draw_background(self, image):
        # create background texture
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D,
                      self.texture_background)  # bind a named texture to a texturing target (target, texture)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, WIDTH, HEIGHT, GL_BGR, GL_UNSIGNED_BYTE,
                        np.flipud(image))  # specify a two-dimensional texture image

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        # Move background
        glTranslatef(0.0, 0.0, -bg_dist * MARKER_SIZE)

        fn = 1.07

        # Draw background
        glBegin(GL_QUADS)
        glTexCoord2f(1, 1)  # set the current texture coordinates
        glVertex3f(0.8 * bg_dist * MARKER_SIZE * fn, 0.45 * bg_dist * MARKER_SIZE * fn, 0.0)
        glTexCoord2f(1, 0)
        glVertex3f(0.8 * bg_dist * MARKER_SIZE * fn, -0.45 * bg_dist * MARKER_SIZE * fn, 0.0)
        glTexCoord2f(0, 0)
        glVertex3f(-0.8 * bg_dist * MARKER_SIZE * fn, -0.45 * bg_dist * MARKER_SIZE * fn, 0.0)
        glTexCoord2f(0, 1)
        glVertex3f(-0.8 * bg_dist * MARKER_SIZE * fn, 0.45 * bg_dist * MARKER_SIZE * fn, 0.0)
        glEnd()
        glDisable(GL_TEXTURE_2D)

    def init_background(self, image):
        # Convert image to OpenGL texture format
        bg_image = Image.fromarray(image)
        ix = bg_image.size[0]
        iy = bg_image.size[1]
        bg_image = bg_image.tobytes("raw", "BGRX", 0, -1)

        # create background texture
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glBindTexture(GL_TEXTURE_2D, 0)

    def window_resize(self, window, width, height):
        aspect = (width * mtx[1, 1]) / (height * mtx[0, 0])
        glViewport(0, 0, width, height)
        gluPerspective(fovy, aspect, 0.1, 500.0 * MARKER_SIZE)

    def main(self):
        # setup and run OpenGL
        glfw.init()
        self.main_window = glfw.create_window(WIDTH, HEIGHT, "AR - GLFW", None, None)
        glfw.set_window_pos(self.main_window, 200, 100)
        glfw.make_context_current(self.main_window)

        glfw.get_primary_monitor()

        # glfw.set_key_callback(self.main_window, self.key_callback)
        glfw.set_window_aspect_ratio(self.main_window, 16, 9)
        glfw.set_window_size_callback(self.main_window, self.window_resize)

        self._init_gl(WIDTH, HEIGHT)
        self.glfw_main_loop()


# run an instance of OpenGL Glyphs
openGLMarker = OpenGLAR3D()
openGLMarker.main()
