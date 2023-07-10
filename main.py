import pygame
from pygame.locals import *
import numpy as np
import math
import random
import copy
from shading_models import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *

W = 800
H = 600

pygame.init()
pantalla=pygame.display.set_mode((W, H), DOUBLEBUF | OPENGL)
glutInit()
gluPerspective(45, (W / H), 0.1, 50.0)
glTranslate(0.0, 0.0, -5)

display_surface = None #Esta variable se utiliza para almacenar la superficie de visualización en la que se 
#dibujarán los objetos

white = (255.0, 255.0, 255.0) 
green = (0, 255, 0) 
blue = (0, 0, 128) 
black = (0, 0, 0) 
red = (255, 0, 0)
yellow = (252, 203, 0)
col1 = (212, 196, 251)
col2 = (177, 52, 235) #colores en formato RGB 
P = None #variable para guardar opjetos
cam = None #guardar inf acerca de la camara
mouse_pos = None #guardar inf acerca de la posicion del mouse

time = 0 #almacenar el tiempo
delta_time = 1/60 #tiempo transcurrido entre cada cuadro en la animacion

class Model:
#__init__es el constructor de la clase y se ejecuta cuando se crea una instancia de la clase Model
# Toma varios argumentos incluyendo f que es el nombre del archivo OBJ que se va a cargar 
# pos que es una matriz de posición y scale que es un factor de escala
    def __init__(self, f, posMatriz = np.array((0, 0, 0, 1)), scale = 1.0):
        file = open('./'+ f)
        self.name = f # guarda el nombre del archivo
        self.points = [] #almacena los puntos del modelo
        self.normals = [] #almacena las normales
        self.tri_normals = [] #almacena las normales de los trinagulos
        self.triangles = [] #almacena los triangulos
        self.tris = []# lista de triangulos modificados
        self.pos = posMatriz #almacenaa la posicion
        self.scale = scale  #almacena el factor de escala
        for l in file:
            l = l.split(' ')
    
            if l[0] == 'v':
           
                self.points.append(posMatriz + np.asarray([self.scale*float(l[i]) for i in range(1,4)]+[1]))
            elif l[0] == 'vn':
                self.normals.append(np.asarray([self.scale*float(l[i]) for i in range(1,4)]))
            elif l[0] == 'f':
                if '/' in l[1]:
          
                    self.triangles.append([int(l[i].split('/')[0]) for i in range(1, 4)])
                    self.tri_normals.append([int(l[i].split('/')[2]) for i in range(1, 4)])
                else:
                    self.triangles.append([int(l[i]) for i in range(1,4)])
        for t in self.triangles:
            self.tris.append([self.points[p-1] for p in t] + [p-1 for p in t])
        for i in range(len(self.triangles)):
            if self.name ==  ('cube.obj'):
                break
            t = self.tri_normals[i]
            n = np.array((0.0, 0, 0))
            for d in t:
                n+=self.normals[d-1]/3
            self.tris[i].append(normalized(n))
            #se realizan algunas operaciones adicionales en las listas de triángulos y normales
            # Los triángulos se definen en una lista de vértices 
            # y se agregan índices de vértices y normales a cada triángulo. Las normales se promedian para cada triángulo y se normalizan
    
    def rotate_Y_axis(self, theta, other):
        theta = math.radians(theta)
        for p in self.points:
            x, z = p[0], p[2]
            p[0] = math.cos(theta)*x - math.sin(theta)*z
            p[2] = math.sin(theta)*x + math.cos(theta)*z
        
        p = other.pos
        x, z = p[0], p[2]
        p[0] = math.cos(theta)*x - math.sin(theta)*z
        p[2] = math.sin(theta)*x + math.cos(theta)*z
        #se utiliza para rotar el modelo alrededor del eje Y toma dos argumentos: theta el ángulo de rotación en grados y other
 #-------------------------------------------------       
class Light:
    #-- es la luz en la escena 3D. Tiene una posición  y un color, que se almacenan como matrices numpy.
    def __init__(self, pos, color = np.asarray(white)):
        self.pos = pos
        self.color = color

class Camera:
    #-- Esta clase representa una cámara en la escena 3D. Tiene una posición (eye),
    #-- Una dirección de vista (front), vectores de orientación (right, up) y una matriz de vista (view_mat).
    up_world = np.asarray((0, 1, 0))
    def __init__(self, eye = [0, 0, 0], is_perspective = True):
        self.eye = eye
        self.front = None
        self.right = None
        self.up = None
        self.is_perspective = is_perspective
        self.view_mat = None
        self.yaw = -90
        self.pitch = 90
        self.sens_x = 0.2
        self.sens_y = 0.2
    
    def lookat(self, eye, at, up = (0, 1, 0)):
        #--Establece la posición de la cámara
        self.eye = eye
        #--Calcula el vector de dirección de vista restando el punto de vista del punto de destino  y normalizando el resultado.
        self.front = normalized(eye-at)
        #--Calcula el vector hacia la derecha  utilizando el producto cruz entre el vector de dirección de vista  y el vector hacia arriba del mundo
        self.right = normalized(cross(Camera.up_world, self.front))
        #--Calcula el vector hacia arriba utilizando el producto cruz entre el vector de dirección de vista y el vector hasia la derecha.
        self.up =  cross(self.front, self.right)
        #--Crea la matriz de vista utilizando los vectores de dirección y la posición de la cámara.
        self.view_mat = np.asarray(
            #--La matriz de vista se crea combinando los vectores y la posición en una matriz 4x4. La columna adicional [0, 0, 0, 1] se agrega para completar la matriz.
            [
                [self.right[0], self.right[1], self.right[2], - dot(self.eye, self.right)],
                [self.up[0], self.up[1], self.up[2], - dot(self.eye, self.up)],
                [self.front[0], self.front[1], self.front[2], - dot(self.eye, self.front)],
                [0, 0, 0, 1]
            ]
        )
    
    def update(self, delta_mouse):
        #--Actualiza el ángulo de rotación horizontal sumándole el desplazamiento horizontal del raton  multiplicado por el factor de sensibilidad horizontal.
        #--Esto controla el movimiento de la cámara de izquierda a derecha.
        self.yaw += self.sens_x*delta_mouse[0]
        #--Esto controla el movimiento de la cámara hacia arriba y hacia abajo.
        self.pitch -= self.sens_y*delta_mouse[1]
        #-- Esto evita que la cámara gire demasiado hacia arriba o hacia abajo. Se asegura que el angulo de inclinacion este entre 0 y 180
        self.pitch = max(0, min(180, self.pitch))
        #--Utilizamos funciones trigonométricas para obtener las componentes x, y, y z del vector.
        new_point = np.array((
            #--Calcula la componente x del vector
            math.sin(math.radians(self.pitch))*math.cos(math.radians(self.yaw)),
            #--Calcula la componente y del vector
            math.cos(math.radians(self.pitch)),
            #--Calcula la componente z del vector
            math.sin(math.radians(self.pitch))*math.sin(math.radians(self.yaw))
        ))
        #--Llama al método lookat() para actualizar la orientación de la cámara, utilizando la posición actual y la nueva dirección de vista.
        self.lookat(self.eye, self.eye + new_point)
    
    
    def reset(self):
        #--Esto indica que la cámara mirará hacia la izquierda.
        self.yaw = -90
        #--La cámara estará orientada hacia abajo.
        self.pitch = 90
        #--Coloca la cámara en el origen del sistema de coordenadas y la desplaza 10 unidades hacia adelante a lo largo del eje Z.
        self.eye = np.array((0.0, 0, 10))
        #--orientamos la dirección de vista de la camra hacia el origen
        self.lookat(self.eye, np.array((0, 0, 0)))


def rgb(a, b, c):
    #--Esta línea crea un vector numpy  que toma como argumento una tupla que contiene los valores de los componentes rgb del color.
    return np.asarray((a, b, c), dtype=int)  

#--Crea un vector numpy llamado pink que representa el color rosa
pink = rgb(234, 185, 201)
#--Crea un vector numpy llamado sky que representa el color del cielo
sky = rgb(64, 0, 130)

def magnitude(v):
    #--Esta línea calcula la magnitud del vector v utilizando la fórmula de la distancia euclidiana en tres dimensiones
    return math.sqrt(v[0]*v[0] +v[1]*v[1] +v[2]*v[2])

def cross(a, b):
    #--Esta línea calcula las componentes X, Y y Z del producto cruz de los vectores a y b utilizando la fórmula del producto cruz. 
    c = [a[1]*b[2] - a[2]*b[1],
         a[2]*b[0] - a[0]*b[2],
         a[0]*b[1] - a[1]*b[0]]
    #--Vector numpy a partir de las componentes calculadas (c) y lo devuelve como resultado.
    return np.array(c)

#--------------------------------------------
def perspective(n, f, FOV, a):
    #--Esta línea calcula el valor de c utilizando la fórmula matemática 1/tan(FOV/2) donde FOV es el campo de visión en grados. 
    c = 1/math.tan(math.radians(FOV/2))
    #--Esta línea devuelve una matriz numpy que representa la matriz de proyección en perspectiva.
    return np.asarray(
         #--genera una matriz de proyección en perspectiva utilizando los parámetros n, f, FOV y a
        [
            [c, 0, 0, 0],
            [0, c*a, 0, 0],
            [0, 0, -(f+n)/(f-n), -2*f*n/(f-n)],
            [0, 0, -1, 0]
        ]
    )
#*--- indicamos que las variables P y cam se utilizarán como variables globales dentro de la función.
def init(cam_pos, object_pos):
    #--las variables P y cam se utilizarán como variables globales dentro de la función.
    global P, cam
    #--llama a la función perspective() para generar una matriz de proyección en perspectiva utilizando los valores proporcionados.
    P = perspective(1, 500, 60, W/H)
    #--creamos una instancia de la clase Camera y la asigna a la variable global cam.
    cam = Camera()
    #--Esta línea llama al método lookat() de la instancia de Camera para establecer la posición y orientación de la cámara. 
    cam.lookat(cam_pos, object_pos)

def test_triangle():
    e = 15
    t = time
     #--Los cálculos de las coordenadas x, y y z se basan en funciones trigonométricas y multiplicaciones por e.
    points = [
        [-e*math.cos(t), -e, -e*math.sin(t)], [e*math.cos(t), -e, e*math.sin(t)], [-e*math.cos(t), e, -e*math.sin(t)],
        [-e*math.cos(t), e, -e*math.sin(t)], [e*math.cos(t), -e, e*math.sin(t)], [e*math.cos(t), e, e*math.sin(t)]
    
        ]
    #--crea una lista vacía llamada new_points que se utilizará para almacenar los puntos transformados.
    new_points = []
    #--itera sobre cada punto en la lista points.
    for p in points:
        #--Esta línea realiza una multiplicación de matrices para transformar el punto utilizando la matriz de vista de la cámara 
        p = np.matmul(cam.view_mat, np.asarray(p+[1]))
        #-- Esta línea realiza otra multiplicación de matrices para aplicar la matriz de proyección en perspectiva al punto transformado anteriormente.
        p = np.matmul(P, p)
        #-- Esta línea divide las coordenadas del punto por su componente w (última componente) para normalizar las coordenadas homogéneas.
        p/=p[-1]
        new_points.append(p)
#-- Esta línea llama a una función draw_triangle() para dibujar un triángulo utilizando los primeros tres puntos de la lista. Se especifica el color blanco para el triángulo.    
    draw_triangle(new_points[:3], color = white)
    #--Esta línea llama a la función para dibujar otro triángulo utilizando los siguientes tres puntos de la lista.
    draw_triangle(new_points[3:6], color = col1)



def draw_triangle(points, color = black):
    #--Dibuja un triángulo en la superficie de visualización
    pygame.draw.polygon(display_surface, color, points)
    #--Esta línea dibuja los contornos del triángulo con un color negro. El argumento adicional 1 indica que el contorno debe tener un grosor de 1 píxel.
    pygame.draw.polygon(display_surface, black, points, 1)


def draw_model(model, light, is_light = False):
#--Rotamos el modelo alrededor del eje Y si el parámetro is_light es True. 
#-- La rotación se realiza utilizando el método rotate_Y_axis() del objeto model y se basa en el tiempo y la información de luz.    
    if is_light:
        model.rotate_Y_axis(25 * delta_time*1.5, light)
    #--copia profunda de la lista de triángulos (tris) del modelo.
    tris = copy.deepcopy(model.tris)
    #--Estas líneas transforman los vértices de cada triángulo en la lista tris utilizando la matriz de vista de la cámara (cam.view_mat).
    #--Se realiza una multiplicación de matrices utilizando la función para aplicar la transformación.
    for t in tris:
        for i in range(3):
            t[i] = np.matmul(cam.view_mat,  t[i])
        #--ordena los triángulos en la lista tris según su profundidad. 
    tris.sort(key = lambda x:x[0][2] + x[1][2] + x[2][2])
    for j in range(len(tris)):
        t = []
    #--Extrae los índices de los vértices del triángulo original. Estos índices se utilizan para acceder a los vértices en el objeto 
        t1 = tris[j][3:]
        draw = True
        #--Bucle en rango de 3 que itera sobre los vertices del triangulo
        for i in range(3):
            #--Transformamos los vertices del triangulo y realizamos una multiplicacion de matrices
             a = np.matmul(P, tris[j][i])
            #--aqui normalizamos las coordenadas divideiendolas por su componente
             a /= a[-1]
             #--comprobamos su las coordenadas normalizadaas del verticen están dentro del rango de vizualizacion(en x, y)
             if min(a)<-1 or max(a)>1:
                 draw = False
                 break
            #--Agregamos el vertice transformado a t, mapeando las coordenadas para ajustarlas al tamaño de la ventana
             t.append([a[0]*W//2 + W//2, -a[1]*H//2 + H//2])
        if not draw:
            continue
        #--Aqui determinamos el color del triangulo si hay una fuente de luz
        if not is_light:
            area = 1
            normal = tris[j][6]
        #--Si hay una fuente de luz, se utiliza un cálculo de iluminación difusa para determinar el color del triángulo.       
            if area>0.61:
                diff = rgb(127, 255, 54)/255
        #-- Si no hay una fuente de luz, se utiliza un color predeterminado.
            else:
                diff = rgb(87, 56, 40)/255
        c = []
        if is_light:
            c = (255, 255, 255)
        else:
            
            c = diff*255.0*max(0.1, dot(normalized(light.pos-model.points[t1[0]][:3]), normal))
        #--Si se ha determinado un color, se llama a la función  para dibujar el triángulo
        if len(c)>0:
            draw_triangle(t, c)
        else:
        #--Si no, dibujamos el triangulo de un color predeterminado
            draw_triangle(t, diff*255)

def draw_edge(p1, p2, color = black):
    #--Esta línea dibuja un segmento de línea entre los puntos p1 y p2 en la superficie de visualización. 
    pygame.draw.line(display_surface, color, p1, p2, width=2)
#--controlar el movimiento y la orientación de la cámara en función de las teclas presionadas y el movimiento del ratón.
def cam_controller(vel, new_mouse):
    #--Esta línea obtiene el estado actual de todas las teclas presionadas utilizando la función 
    keys = pygame.key.get_pressed()
    #--Si estamos presionando la letra D entonces la camara se mueve a la derecha
    if keys[pygame.K_d]:
        cam.eye += vel*delta_time*cam.right
    #--Si estamos presionamos la letra A la camara se mueve a la izquierda    
    if keys[pygame.K_a]:
        cam.eye -= vel*delta_time*cam.right
    #--Si presionamos la letra W la camara se mueve al frente
    if keys[pygame.K_w]:
        cam.eye -= vel*delta_time*cam.front
    #--Si presionamos la letra S la camara va hacia atras
    if keys[pygame.K_s]:
        cam.eye += vel*delta_time*cam.front
    #--Si la tecla R esta presionada se restablece la posición y orientación de la camara
    if keys[pygame.K_r]:
        cam.reset()
    #--Esta línea actualiza la orientación de la cámara en función del movimiento del ratón.
    cam.update((new_mouse[0]-mouse_pos[0], -new_mouse[1]+mouse_pos[1]))
#--Representación y actualización continua de la escena en la pantalla utilizando la biblioteca 
def render():
    #--Con esta funcion ocultamos el cursor del raton en la ventana
    global time, delta_time, mouse_pos
    
    pygame.mouse.set_visible(False)
    run = True
    display_surface.fill(rgb(65, 60, 105))
    #--Un objeto PixelArray permite un acceso más rápido a los píxeles individuales de la superficie.
    pxarray = pygame.PixelArray (display_surface)
    #--En esta parte configuramos la fuente de luz, modelo y camara
    light_pos = (8.0, 4.0, 0, 1.0)
    light = Light(np.array(light_pos[:3]))
    model = Model('tree.obj', scale = 3)
    model_light = Model('cube.obj', posMatriz=light_pos, scale = 0.5)
    
    vel = 10

    while run:
    #--Obtenemos la posicion del raton y la guardamos en mouse_pos
        mouse_pos = pygame.mouse.get_pos()
        #-- itera sobre todos los eventos generados
        for event in pygame.event.get():
            #--Si la tecla presionada es Esc entonces se cierra
            if event.type == pygame.QUIT:
                run = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    run = False
        #--Esta línea obtiene el tiempo en milisegundos desde que comenzó la aplicación.
        t = pygame.time.get_ticks()
        #--Se realiza una serie de operaciones para determinar qué modelo se dibuja primero según la distancia entre la cámara y la posición del modelo y la posición de la luz. 
        display_surface.fill(rgb(30, 30, 30))
    
        d_m, d_l = np.linalg.norm(cam.eye-model.pos[:3]-np.array((0, 1, 0))), np.linalg.norm(cam.eye-light.pos[:3])
       #Llamamos la funcion draw_model para dibujar los modelos correspondientes
        if d_m<=d_l:
            draw_model(model_light, light, is_light=True)
            draw_model(model, light, is_light=False)
        else:
            draw_model(model, light, is_light= False)
            draw_model(model_light, light, is_light=True)
        cam_controller(vel, pygame.mouse.get_pos())
        #--Esta línea dibuja un pequeño círculo blanco en el centro de la ventana de visualización
        pygame.draw.circle(display_surface, white, (W//2, H//2), 2)
        pygame.display.update()
        #--Esta línea imprime en la consola la cantidad de fotogramas por segundo estimada.
        delta_time = (pygame.time.get_ticks() - t)/1000
        
        if delta_time == 0:
            delta_time = 0.001
        print('FPS:'+ str(1/delta_time))
    #--Esta línea actualiza el tiempo total de la aplicación agregando el tiempo transcurrido en cada iteración del bucle.
        time += delta_time

    pygame.quit()

if __name__ == "__main__":
    pygame.init()
    display_surface = pygame.display.set_mode((W, H ))
    display_surface = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    init(np.array((0, 0, 10.0)), np.asarray((0, 0, 0)))
    render()
