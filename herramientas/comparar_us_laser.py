#!/usr/bin/env python3
"""
Comparativa en vivo: ultrasonido (HC-SR04) vs laser (VL53L0X).

Lee del puerto serie las lineas "CSV,..." que imprime medicion_task (main.c),
las guarda en un archivo y las grafica en vivo:

  1. Distancia de los dos sensores en el tiempo (mm)
  2. Diferencia ultrasonido - laser (mm)
  3. Estadisticas de la ventana: media, desvio, % de mediciones validas y tiempos

Uso:
  python3 herramientas/comparar_us_laser.py                   # en vivo, /dev/ttyUSB0
  python3 herramientas/comparar_us_laser.py -p /dev/ttyACM0   # otro puerto
  python3 herramientas/comparar_us_laser.py -a mediciones.csv # ver un archivo ya guardado

OJO: el puerto no se puede compartir. Cerrar 'idf.py monitor' antes de correr esto.
Requiere: pyserial, matplotlib, numpy.
"""
import argparse
import csv
import datetime
import math
import threading
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

COLUMNAS = ["t_ms", "us_mm", "us_estado", "us_eco_us", "laser_mm", "laser_estado", "laser_dur_us"]

# ---- Colores (paleta categorica validada: CVD y contraste OK) ----
COLOR_US = "#2a78d6"      # ultrasonido: azul
COLOR_LASER = "#eb6834"   # laser: naranja
SUPERFICIE = "#fcfcfb"
TINTA = "#0b0b0b"
TINTA_2 = "#52514e"
TINTA_MUTED = "#898781"
GRILLA = "#e1e0d9"
EJE = "#c3c2b7"


def parsear(linea):
    """'CSV,1520,503.4,OK,...' -> dict, o None si no es una medicion."""
    linea = linea.strip()
    if not linea.startswith("CSV,"):
        return None
    campos = linea[4:].split(",")
    if len(campos) != len(COLUMNAS) or campos[0] == "t_ms":   # encabezado
        return None
    try:
        m = dict(zip(COLUMNAS, campos))
        m["t_s"] = int(m["t_ms"]) / 1000.0
        m["us_mm"] = float(m["us_mm"]) if m["us_estado"] == "OK" else math.nan
        m["laser_mm"] = float(m["laser_mm"]) if m["laser_estado"] == "OK" else math.nan
        m["us_eco_us"] = int(m["us_eco_us"])
        m["laser_dur_us"] = int(m["laser_dur_us"])
        return m
    except ValueError:
        return None


class Datos:
    """Ultimas N mediciones, compartidas entre el hilo que lee y el que grafica."""

    def __init__(self, n):
        self.buf = deque(maxlen=n)
        self.lock = threading.Lock()
        self.total = 0

    def agregar(self, m):
        with self.lock:
            self.buf.append(m)
            self.total += 1

    def copia(self):
        with self.lock:
            return list(self.buf), self.total


def leer_serie(puerto, baud, datos, archivo_salida):
    import serial

    with serial.Serial(puerto, baud, timeout=1) as ser, open(archivo_salida, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNAS)
        print(f"Leyendo {puerto} @ {baud}. Guardando en {archivo_salida}")
        while True:
            linea = ser.readline().decode("utf-8", errors="replace")
            m = parsear(linea)
            if m is None:
                continue
            w.writerow([linea.strip()[4:].split(",")[i] for i in range(len(COLUMNAS))])
            f.flush()
            datos.agregar(m)


def leer_archivo(ruta, datos):
    with open(ruta) as f:
        for fila in csv.reader(f):
            m = parsear("CSV," + ",".join(fila))
            if m:
                datos.agregar(m)


def estilo_eje(ax, titulo):
    ax.set_facecolor(SUPERFICIE)
    ax.set_title(titulo, loc="left", color=TINTA, fontsize=11, fontweight="bold", pad=24)
    ax.grid(axis="y", color=GRILLA, linewidth=0.8)
    ax.set_axisbelow(True)
    for lado in ("top", "right", "left"):
        ax.spines[lado].set_visible(False)
    ax.spines["bottom"].set_color(EJE)
    ax.tick_params(colors=TINTA_MUTED, labelsize=9, length=0)


def stats(v):
    v = np.asarray(v, dtype=float)
    ok = v[~np.isnan(v)]
    if len(ok) == 0:
        return math.nan, math.nan, 0.0
    return ok.mean(), ok.std(), 100.0 * len(ok) / len(v)


def main():
    ap = argparse.ArgumentParser(description="Comparativa ultrasonido vs laser")
    ap.add_argument("-p", "--puerto", default="/dev/ttyUSB0")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-a", "--archivo", help="graficar un CSV ya guardado (sin puerto)")
    ap.add_argument("-v", "--ventana", type=float, default=20.0, help="segundos visibles en vivo")
    args = ap.parse_args()

    en_vivo = args.archivo is None
    datos = Datos(n=int(args.ventana * 10) if en_vivo else 10**7)   # 10 Hz

    if en_vivo:
        salida = datetime.datetime.now().strftime("mediciones_%Y%m%d_%H%M%S.csv")
        threading.Thread(target=leer_serie, args=(args.puerto, args.baud, datos, salida),
                         daemon=True).start()
    else:
        leer_archivo(args.archivo, datos)

    plt.rcParams["font.family"] = "sans-serif"
    fig = plt.figure(figsize=(11, 7.5), facecolor=SUPERFICIE)
    gs = fig.add_gridspec(3, 1, height_ratios=[3, 1.6, 0.9], hspace=0.45)
    ax_d = fig.add_subplot(gs[0])
    ax_e = fig.add_subplot(gs[1], sharex=ax_d)
    ax_s = fig.add_subplot(gs[2])
    ax_s.axis("off")

    estilo_eje(ax_d, "Distancia medida (mm)")
    estilo_eje(ax_e, "Diferencia ultrasonido − láser (mm)")
    ax_e.set_xlabel("tiempo (s)", color=TINTA_MUTED, fontsize=9)

    (l_us,) = ax_d.plot([], [], color=COLOR_US, lw=2, label="Ultrasonido (HC-SR04)")
    (l_la,) = ax_d.plot([], [], color=COLOR_LASER, lw=2, label="Láser (VL53L0X)")
    et_us = ax_d.text(0, 0, "", color=TINTA, fontsize=9, va="center")
    et_la = ax_d.text(0, 0, "", color=TINTA, fontsize=9, va="center")
    ax_d.legend(loc="lower left", frameon=False, ncol=2, fontsize=9,
                bbox_to_anchor=(0, 1.0), borderaxespad=0.2, labelcolor=TINTA_2)
    ax_e.axhline(0, color=EJE, lw=1)
    (l_dif,) = ax_e.plot([], [], color=TINTA_2, lw=1.5)

    texto_stats = ax_s.text(0, 0.5, "", family="monospace", fontsize=9.5, color=TINTA, va="center")

    def actualizar(_):
        filas, total = datos.copia()
        if not filas:
            texto_stats.set_text("Esperando mediciones... (¿está cerrado idf.py monitor?)")
            return
        t = np.array([m["t_s"] for m in filas])
        us = np.array([m["us_mm"] for m in filas])
        la = np.array([m["laser_mm"] for m in filas])
        dif = us - la

        l_us.set_data(t, us)
        l_la.set_data(t, la)
        l_dif.set_data(t, dif)

        # Ejes: el x sigue a los datos; el y se ajusta a lo valido con margen
        ax_d.set_xlim(t[0], max(t[-1], t[0] + 1))
        validos = np.concatenate([us[~np.isnan(us)], la[~np.isnan(la)]])
        if len(validos):
            lo, hi = validos.min(), validos.max()
            m = max(10.0, 0.1 * (hi - lo))
            ax_d.set_ylim(max(0, lo - m), hi + m)
        dv = dif[~np.isnan(dif)]
        if len(dv):
            a = max(5.0, np.abs(dv).max() * 1.2)
            ax_e.set_ylim(-a, a)

        # Etiquetas directas con el ultimo valor valido al final de cada linea.
        # Si quedan muy cerca, se separan (el de mayor valor arriba) para que no se pisen.
        ultimos = []
        for etiqueta, serie, nombre in ((et_us, us, "US"), (et_la, la, "Láser")):
            ok = np.where(~np.isnan(serie))[0]
            ultimos.append(serie[ok[-1]] if len(ok) else None)
            etiqueta.set_text(f"  {nombre} {ultimos[-1]:.0f} mm" if len(ok) else "")
        y0, y1 = ax_d.get_ylim()
        sep = 0.06 * (y1 - y0)
        pos = [u for u in ultimos]
        if None not in pos and abs(pos[0] - pos[1]) < sep:
            medio = (pos[0] + pos[1]) / 2
            arriba, abajo = (0, 1) if pos[0] >= pos[1] else (1, 0)
            pos[arriba], pos[abajo] = medio + sep / 2, medio - sep / 2
        for etiqueta, y in zip((et_us, et_la), pos):
            if y is not None:
                etiqueta.set_position((t[-1], y))

        mu_us, sd_us, ok_us = stats(us)
        mu_la, sd_la, ok_la = stats(la)
        mu_d, sd_d, _ = stats(dif)
        eco = np.array([m["us_eco_us"] for m in filas if m["us_estado"] == "OK"])
        dur = np.array([m["laser_dur_us"] for m in filas])
        eco_txt = f"{eco.mean() / 1000:5.2f} ms" if len(eco) else "   -   "
        texto_stats.set_text(
            f"{'':14}{'media':>10}{'desvío':>10}{'válidas':>10}{'tiempo':>12}\n"
            f"{'Ultrasonido':14}{mu_us:>8.1f}mm{sd_us:>8.1f}mm{ok_us:>9.0f}%{eco_txt:>12}  (eco)\n"
            f"{'Láser':14}{mu_la:>8.1f}mm{sd_la:>8.1f}mm{ok_la:>9.0f}%{dur.mean() / 1000:>9.2f} ms  (medición)\n"
            f"{'US − láser':14}{mu_d:>8.1f}mm{sd_d:>8.1f}mm"
            f"{'':10}   {len(filas)} muestras en ventana · {total} en total"
        )

    if en_vivo:
        anim = FuncAnimation(fig, actualizar, interval=200, cache_frame_data=False)  # noqa: F841
    else:
        actualizar(None)
    fig.canvas.manager.set_window_title("Ultrasonido vs Láser")
    plt.show()


if __name__ == "__main__":
    main()
