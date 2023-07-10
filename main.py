import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PIL import Image
import numpy
import math

light_angle = 0.5  # Ángulo de rotación de la luz alrededor del árbol
light_radius = 5.0  # Radio de la órbita de la luz alrededor del árbol

def load_texture(file_path):
    image = Image.open(file_path)
    flipped_image = image.transpose(Image.FLIP_TOP_BOTTOM)
    texture_data = flipped_image.convert("RGBA").tobytes()
    width, height = flipped_image.size

    texture_id = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture_id)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture_data)
    glBindTexture(GL_TEXTURE_2D, 0)

    return texture_id

def load_obj(file_path):
    vertices = []
    normals = []
    texture_coords = []
    faces = []

    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('v '):
                # Coordenadas de vértices
                vertex = line.split()[1:]
                vertex = list(map(float, vertex))
                vertices.append(vertex)
            elif line.startswith('vn '):
                # Coordenadas de normales
                normal = line.split()[1:]
                normal = list(map(float, normal))
                normals.append(normal)
            elif line.startswith('vt '):
                # Coordenadas de texturas
                tex_coord = line.split()[1:]
                tex_coord = list(map(float, tex_coord))
                texture_coords.append(tex_coord)
            elif line.startswith('f '):
                # Caras (conectividad de vértices, normales y texturas)
                face = line.split()[1:]
                face = [vertex.split('/') for vertex in face]
                face = [[int(vertex[0]) - 1, int(vertex[1]) - 1, int(vertex[2]) - 1] for vertex in face]
                faces.append(face)

    return vertices, normals, texture_coords, faces

def render_model(vertices, normals, texture_coords, faces, texture_id):
    glEnable(GL_TEXTURE_2D)
    glBindTexture(GL_TEXTURE_2D, texture_id)
    glBegin(GL_TRIANGLES)
    for face in faces:
        for vertex in face:
            vertex_index, texture_index, normal_index = vertex
            vertex_coords = vertices[vertex_index]
            normal_coords = normals[normal_index]
            tex_coords = texture_coords[texture_index]
            glTexCoord2fv(tex_coords)
            glNormal3fv(normal_coords)
            glVertex3fv(vertex_coords)
    glEnd()
    glDisable(GL_TEXTURE_2D)

def draw_light_sphere():
    global angle 
    angle = 0.5

    glPushMatrix()  # Guarda la matriz de transformación actual

    # Aplica la transformación de rotación
    glRotatef(angle, 0.0, 1.0, 0.0)  # Rota alrededor del eje Y

    # Posiciona la esfera de luz
    light_distance = 5.0  # Distancia de la esfera de luz al origen
    light_angle = angle  # Ángulo de rotación de la esfera de luz
    light_pos = [
        light_distance * math.sin(math.radians(light_angle)),
        0.0,
        light_distance * math.cos(math.radians(light_angle))
    ]
    glTranslatef(light_pos[0], light_pos[1], light_pos[2])  # Traslada la esfera de luz

    # Dibuja la esfera de luz
    glColor3f(1.0, 1.0, 0.0)  # Color amarillo para la esfera de luz
    glutSolidSphere(0.1, 20, 20)  # Cambia los parámetros según el tamaño deseado de la esfera

    glPopMatrix()  # Restaura la matriz de transformación guardada


def render_scene(vertices, normals, texture_coords, faces, texture_id):
    # Borra el búfer de color y el búfer de profundidad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    # Configura la cámara
    gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0)
    light_pos = [light_radius * math.cos(light_angle), 0.0, light_radius * math.sin(light_angle)]
    glLightfv(GL_LIGHT0, GL_POSITION, (light_pos[0], light_pos[1], light_pos[2], 1.0))

    #draw_light_sphere()

    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)

    # Configura las propiedades de la luz
    light_diffuse = [1.0, 1.0, 1.0, 1.0]  # Color difuso de la luz
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse)

    # Renderiza el modelo 3D con la textura
    render_model(vertices, normals, texture_coords, faces, texture_id)

    # Actualiza la ventana
    pygame.display.flip()

def idle_func():
    glutPostRedisplay()


def main():
    # Inicializa Pygame
    pygame.init()
    width, height = 800, 800
    pygame.display.set_mode((width, height), DOUBLEBUF | OPENGL)
    glutInit()
    global light_angle
    global light_radius
    # Configura OpenGL
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (width / height), 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)
    glEnable(GL_DEPTH_TEST)

    # Carga el modelo 3D y la textura
    vertices, normals, texture_coords, faces = load_obj('tree.obj')
    texture_id = load_texture('cortezaTextura.jpg')

    # Bucle principal
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            # Llama a la función de renderizado
        render_scene(vertices, normals, texture_coords, faces, texture_id)
        light_angle += 0.01  # Ajusta la velocidad de rotación según tus necesidades
        glutIdleFunc(idle_func)

        

if __name__ == "__main__":
    main()
