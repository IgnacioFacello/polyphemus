import numpy as np
np.set_printoptions(precision=4, suppress=True) #con esto hacemos que muestre hasta 4 decimales, es solo para que sea mas limpio
import math

#Ingresar angulos de rotación
anz = float(input("Ingrese angulo alfa"))
any = float(input("Ingrese angulo beta"))
anx = float(input("Ingrese angulo gamma"))

#Paso grados radianes
alfa = np.radians(anz)
beta = np.radians(any)
gamma = np.radians(anx)

#Ingresa Vector
x = float(input(" Ingrese x "))
y = float(input(" Ingrese y "))
z = float(input(" ingrese z "))

V = np.array([
    x,
    y,
    z
])
# Primera rotacion eje Z (ALFA)
M1 = np.array([
    [ np.cos(alfa),  np.sin(alfa), 0],
    [-np.sin(alfa),  np.cos(alfa), 0],
    [0,              0,             1]
])

# Segunda rotacion eje Y (BETA)
M2 = np.array([
    [np.cos(beta), 0, -np.sin(beta)],
    [0,            1,  0],
    [np.sin(beta), 0,  np.cos(beta)]
])

# Tercera rotacion eje X (GAMMA)
M3 = np.array([
    [1, 0,              0],
    [0, np.cos(gamma),  np.sin(gamma)],
    [0, -np.sin(gamma), np.cos(gamma)]
])
#Con las matrices de rotacion ya armadas --> @ hace la multiplicacion matricial de los vectores
V1 = M1 @ V
V2 = M2 @ V1
Vb = M3 @ V2

M_total = M3 @ M2 @ M1


# RESULTADOS


print("\nVector inicial:")
print(V)

print("\nDespues de rotar alfa alrededor de Z:")
print(V1)

print("\nDespues de rotar beta alrededor de Y:")
print(V2)

print("\nDespues de rotar gamma alrededor de X:")
print(Vb)

print("\nMatriz total Euler tipo 2 (Z-Y-X):")
print(M_total)

print("\nVector final:")
print(Vb)
