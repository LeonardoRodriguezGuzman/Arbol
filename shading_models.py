import numpy as np

def normalized(v):
    return v/np.linalg.norm(v)

def dot(a,b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def clamp(x, a, b):
    return max(a, min(x, b))

def reflect(r, n):
    return normalized(r + (1+dot(r, n))*n)

def get_light_color_bp(eye, light, vertPos, normal, diffuseColor):
    lightPos = light.pos
    lightColor = light.color
    lightPower = 0.1
    ambientColor = diffuseColor*0.05
    specColor = np.asarray((1.0, 1.0, 1.0))
    shininess = 20
    screenGamma = 2.2

    lightDir = lightPos - vertPos
    distance = np.linalg.norm(lightDir)
#    distance = 1
    distance = distance * distance
    lightDir = normalized(lightDir)
    
    lambertian = max( dot(lightDir, normal), 0.0)
#    print(lambertian, np.linalg.norm(lightDir), np.linalg.norm(normal))
    specular = 0.0
    if lambertian > 0.0:
        viewDir = normalized(eye-vertPos)
        halfDir = normalized(lightDir + viewDir)
        specAngle = max( dot(halfDir, normal), 0.0)
        specular = pow(specAngle, shininess)

    colorLinear = ambientColor + diffuseColor * lambertian * lightColor * lightPower / distance + specColor * specular * lightColor * lightPower / distance
#    colorLinear = colorLinear**(screenGamma)
#    return colorLinear
    colorLinear *= 255
    return (min(255, colorLinear[0]), min(255, colorLinear[1]), min(255, colorLinear[2]))

def get_light_color_gooch(eye, light, vertPos, normal, c_surface):
    
    c_cool = np.array((0, 0, 0.55)) + 0.25*c_surface
    c_warm = np.array((0.3, 0.3, 0)) + 0.25*c_surface
#    c_highlight = np.array((1.0, 1, 1))
    lightPos = light.pos
    lightDir = normalized(lightPos - vertPos)
    viewDir = normalized(eye-vertPos)

    dot_ = dot(normal, -lightDir)
    # t = (dot_+1.0)/2
    # r = 2*dot_*normal - lightDir
    # s = clamp(-(100*dot(r, viewDir) - 97), 0.0, 1.0)
#    colorLinear = s*c_highlight + (1-s) *(t*c_warm + (1-t)*c_cool)
    colorLinear = (1.0+dot_)/2.0*c_cool + (1.0-(1.0+dot_)/2.0)*c_warm
    colorLinear *= 255
#    print(colorLinear)
    return (min(255, colorLinear[0]), min(255, colorLinear[1]), min(255, colorLinear[2]))

def get_light_color_cel(eye, light, vertPos, normal, diffuseColor):
    lightPos = light.pos
    lightColor = light.color
    lightPower = 0.1
    ambientColor = diffuseColor*0.05
    specColor = np.asarray((1.0, 1.0, 1.0))
    shininess = 20
    screenGamma = 2.2

    lightDir = lightPos - vertPos
    distance = np.linalg.norm(lightDir)
#    distance = 1
    attenuation = 1/distance
    lightDir = normalized(lightDir)
    viewDir = normalized(eye-vertPos)

    lambertian = max( dot(lightDir, normal), 0.0)
#    print(lambertian, np.linalg.norm(lightDir), np.linalg.norm(normal))
    specular = 0.0
    if lambertian > 0.0:
        viewDir = normalized(eye-vertPos)
        halfDir = normalized(lightDir + viewDir)
        specAngle = max( dot(halfDir, normal), 0.0)
        specular = pow(specAngle, shininess)

#    colorLinear = ambientColor + diffuseColor * lambertian * lightColor * lightPower / distance
    colorLinear = diffuseColor
    specColor = specColor * specular * lightColor * lightPower / distance
    if dot(normal, lightDir) > 0.0:
        if attenuation*pow(max(0.0, dot(reflect(-lightDir, normal), viewDir)), shininess)>0.5:
            colorLinear = 0.8*np.array(light.color)/255*specColor + (0.2)*colorLinear
    
    colorLinear *= 255
#    print(colorLinear)
    return (min(255, colorLinear[0]), min(255, colorLinear[1]), min(255, colorLinear[2]))



