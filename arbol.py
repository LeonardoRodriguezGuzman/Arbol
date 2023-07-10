import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

width = 800
height = 600

pygame.init()
pantalla=pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
glutInit()
gluPerspective(45, (width / height), 0.1, 50.0)
glTranslate(0.0, 0.0, -5)

sun_color = (0.82, 0.87, 0.15)
target_sun_color = (1.0, 0.5, 0.0)
background_color = (0.5, 0.86, 0.92)
target_background_color = (0.1, 0.1, 0.1)

sun_position = [0.0, 0.0, 0.0]
sun_radius = 0.1
sun_distance = 200

def setup_lighting():
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_AMBIENT, (0.5, 0.5, 0.5, 1.0))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
    glLightfv(GL_LIGHT0, GL_POSITION, sun_position)

def draw_sun():
    glEnable(GL_COLOR_MATERIAL)
    glColor3f(*sun_color)
    glutSolidSphere(sun_radius, 50, 50)

def main():
    clock = pygame.time.Clock()

    setup_lighting()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glClearColor(*background_color, 1.0)

        glLoadIdentity()
        draw_sun()

        sun_position[0] += 0.01
        if sun_position[0] > width - sun_distance:
            sun_position[0] = width - sun_distance
        elif sun_position[0] < sun_distance:
            sun_position[0] = sun_distance

        pygame.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    main()
