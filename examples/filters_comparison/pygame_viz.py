import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *
import zmq
import numpy as np
import json

host = '192.168.0.103'
port = 8358
url = 'tcp://'+host+':'+str(port)
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(url)
socket.setsockopt(zmq.SUBSCRIBE, b'')

verticies = (
    (1, -2, -1), # 0
    (1, 2, -1), # 1
    (-1, 2, -1), # 2
    (-1, -2, -1), # 3
    (1, -2, 1),  # 4
    (1, 2, 1),  # 5
    (-1, -2, 1), # 6
    (-1, 2, 1)  # 7
    )

edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
    )

pitch_surfaces = (
    (0,1,5,4),
    (2,7,6,3)
    )
roll_surfaces = (
    (2,7,5,1),
    (3,6,4,0)
    )
yaw_surfaces = (
    (6,4,5,7),
    (2,3,0,1)
    )


def Cube():
    glBegin(GL_QUADS)
    for surface in roll_surfaces:
        glColor3fv((1,0,0))
        for vertex in surface:
            glVertex3fv(verticies[vertex])

    for surface in pitch_surfaces:
        glColor3fv((0,1,0))
        for vertex in surface:
            glVertex3fv(verticies[vertex])

    glEnd()

    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(verticies[vertex])
    glEnd()


def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)

    glTranslatef(0.0,0.0, -10)
    # glTranslatef(0.0,-10, 0)
    # glRotatef(90, 0, 1, 0)
    curr_roll = 0
    curr_pitch = 0
    curr_yaw = 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        packet = socket.recv_json(0)
        kal = packet['madgwick']
        vals = np.array(json.loads(kal))

        glRotatef(vals[0] - curr_roll, 1, 0, 0)
        curr_roll = vals[0]

        glRotatef(vals[1] - curr_pitch, 0, 1, 0)
        curr_pitch = vals[1]

        glRotatef(vals[2] - curr_yaw, 0, 0, 1)
        curr_yaw = vals[2]


        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        Cube()
        pygame.display.flip()
        # pygame.time.wait(10)


main()