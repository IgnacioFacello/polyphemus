import numpy as np
import math

#Ingresar angulos de rotación
anz = float(input("Ingrese angulo alfa"))
any = float(input("Ingrese angulo beta"))

#angulo que quiero rotar
anx = float(input("Ingrese angulo fi, angulo a rotar"))
paso_g = float(input("Ingrese paso de micro-rotacion: "))

#Paso grados radianes
alfa = np.radians(anz)
beta = np.radians(any)
fi = np.radians(anx)
paso = np.radians(paso_g)

#componentes del eje de rotación
nx = float(np.sin(beta)*np.cos(alfa))
ny = float(np.sin(beta)*np.sin(alfa))
nz = float(np.cos(beta))

print("\nEje de rotacion:")
print("nx =", nx)
print("ny =", ny)
print("nz =", nz)

#Ingresamos los valores de el vector que queremos rotar
x = float(input("Ingrese x: "))
y = float(input("Ingrese y: "))
z = float(input("Ingrese z: "))

V = np.array([
    x,
    y,
    z
])

#Matriz para rotación total 
M1 = np.array([
    [
        nx**2 + (1 - nx**2)*np.cos(fi),
        nx*ny*(1 - np.cos(fi)) + nz*np.sin(fi),
        nx*nz*(1 - np.cos(fi)) - ny*np.sin(fi)
    ],

    [
        nx*ny*(1 - np.cos(fi)) - nz*np.sin(fi),
        ny**2 + (1 - ny**2)*np.cos(fi),
        ny*nz*(1 - np.cos(fi)) + nx*np.sin(fi)
    ],

    [
        nx*nz*(1 - np.cos(fi)) + ny*np.sin(fi),
        ny*nz*(1 - np.cos(fi)) - nx*np.sin(fi),
        nz**2 + (1 - nz**2)*np.cos(fi)
    ]
])


#Matriz de rotación simplificada MICRO ROTACION 
M_micro = np.array([
    [1,        nz*paso,  -ny*paso],
    [-nz*paso, 1,         nx*paso],
    [ny*paso, -nx*paso,   1]
])

#Calculamos la cantidad de microrotaciones que vamos a tener que hacer
cantidad = int(round(anx/ paso_g))

V_micro = V.copy()

for i in range(cantidad):
    V_micro = M_micro @ V_micro

#Imprimimos matriz de microrotacion 
print("\nMatriz de micro-rotacion:")
print(M_micro)

#Rotamos el vector
V_rotado_exacto = M1 @ V

diferencia = V_micro - V_rotado_exacto


#imprimimos para comparar
print("\n--------------------------------")
print("RESULTADOS")
print("--------------------------------")

print("\nVector inicial:")
print(V)

print("\nRotacion exacta de", anx, "grados:")
print(V_rotado_exacto)

print("\nRotacion mediante", cantidad, "micro-rotaciones de",
      paso_g, "grados:")
print(V_micro)

print("\nDiferencia:")
print(diferencia)
