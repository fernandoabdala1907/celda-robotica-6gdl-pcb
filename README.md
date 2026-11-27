# Celda Robótica de 6 GDL para Manipulación y Posicionamiento de Tarjetas Electrónicas (PCB)

**Proyecto Final — Maestría en Automatización y Robótica (Universidad Tecmilenio)**  
* **Sustentante:** Fernando Abdalá Gómez  
* **Docente:** Ingrith Yuritsa Páez Pidiache  
* **Fecha de entrega:** 4 de diciembre de 2026  
* **Evidencia audiovisual (evolución en 3 etapas):** [https://youtu.be/MoI7Jq5fUXg](https://youtu.be/MoI7Jq5fUXg)

---

## Descripción General del Proyecto

Este repositorio contiene el código fuente completo (~10,000 líneas) desarrollado para el diseño, simulación, control e integración física experimental de una celda robótica antropomórfica de **6 grados de libertad (6R, arquitectura AR4-MK2)** dedicada a automatizar la detección, sujeción, traslado y posicionamiento de tarjetas de circuito impreso (PCB) desde una banda transportadora lineal hacia una estación receptora de prueba funcional.

La arquitectura opera mediante **control distribuido jerárquico en tres niveles**:
1. **Nivel Supervisor y Visión Artificial (PC Host — `AR4.py`):** Interfaz hombre-máquina (HMI), calibración espacial, procesamiento de video cenital con cámara HD Emeet C960 (detección de centroide $X, Y$ y orientación $\theta$), resolución cinemática de 6 GDL y envío de trayectorias por puerto serial.
2. **Nivel de Control Articular Determinista (Teensy 32 bits):** Generación de pulsos `Step/Dir` con perfiles trapezoidales hacia los seis drivers Makerbase (`J1–J6`), modulación PWM para la servopinza paralela y lectura de finales de carrera mecánicos (`LS1–LS6`).
3. **Nivel de Control de Periféricos de Celda (Arduino Mega 2560 — `bandasensores.ino`):** Lectura de sensores infrarrojos `E18-D80NK` y conmutación optoacoplada mediante módulo de 16 relés para controlar el motorreductor de la banda transportadora negra mate ($12\text{ VCD}$) y los frenos electromagnéticos (`J2/J3`), respaldado por Paro de Emergencia Clase 0 ($< 100\text{ ms}$).

---

## Estructura del Repositorio y Descripción de Archivos

| Archivo | Lenguaje / Entorno | Función Técnica en el Proyecto |
| :--- | :--- | :--- |
| **`AR4.py`** | Python 3 (OpenCV / NumPy / PySerial) | **Programa principal de supervisión y control en la PC Host.** Gestiona la interfaz gráfica del manipulador de 6 GDL, ejecuta la visión artificial cenital para localizar pose $(X, Y, \theta)$ sobre la banda negra antirreflejo, resuelve la cinemática directa e inversa y coordina la secuencia automática *Pick-and-Place* con los microcontroladores. |
| **`bandasensores.ino`** | C++ (Arduino IDE) | **Firmware de control de periféricos de celda (Arduino Mega 2560).** Lee los sensores ópticos infrarrojos `E18-D80NK` en entrada y zona de inspección, detiene la banda transportadora para captura estática y conmuta el módulo de 16 relés de potencia ($12\text{ VCD}$ / $24\text{ VCD}$). |
| **`proyectofinal.m`** | MATLAB | **Simulación integral 3D de 6 GDL (AR4-MK2).** Implementa los parámetros Denavit-Hartenberg exactos del robot, valida el punto de prueba cruzado con RoboDK y Excel ($P = [-105.722, 183.115, 303.897]\text{ mm}$) y grafica la trayectoria cúbica 3D con las 6 articulaciones etiquetadas. |
| **`Cinematicaproyecto.m`** | MATLAB | **Validación analítica de Cinemática Directa e Inversa (6 GDL).** Calcula las matrices de transformación homogénea individuales ($A_1 \dots A_6$), el centro de muñeca esférica y la descomposición de orientación ZYX verificando error nulo ($0.000\text{ mm}$). |
| **`Cinematicainversaproyecto.m`** | MATLAB | **Planificación de trayectorias polinomiales cúbicas (6 GDL).** Genera los perfiles articulares suaves $q_1(t) \dots q_6(t)$ con velocidades inicial y final nulas entre la posición de toma en banda negra y la estación receptora en un ciclo nominal de $2.4\text{ s}$. |
| **`Filtroproyectofinal.m`** | MATLAB | **Acondicionamiento de señales y control PID discreto.** Modela el filtrado digital en lazo mediante filtro **Notch IIR ($60\text{ Hz}$)** y pasa bajas **Butterworth ($f_c = 80\text{ Hz}$)** acoplado a un controlador PID discreto con saturación ($\pm 8\text{ N}\cdot\text{m}$) y *anti-windup* en la articulación crítica del hombro (`J2`). |
| **`Potenciaproyecto.m`** | MATLAB | **Dimensionamiento eléctrico, PWM y balance de potencia.** Calcula la tensión media, corrientes nominal/arranque, tiempos $T_{\text{on}}/T_{\text{off}}$ de conmutación PWM a $20\text{ kHz}$ del motorreductor de banda ($12\text{ VCD}$) y el consumo energético total de la celda ($24\text{ V} / 12\text{ V} / 5\text{ V}$). |

---

## Parámetros Cinemáticos Denavit-Hartenberg (AR4-MK2)

| Eje | $\theta_{\text{offset}}$ | $d\text{ [mm]}$ | $a\text{ [mm]}$ | $\alpha\text{ [rad]}$ | Rango Operativo |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **J1 (Cintura)** | $0$ | $169.77$ | $64.20$ | $-\pi/2$ | $-170^\circ \text{ a } +170^\circ$ |
| **J2 (Hombro)** | $-\pi/2$ | $0.00$ | $305.00$ | $0$ | $-42^\circ \text{ a } +90^\circ$ |
| **J3 (Codo)** | $\pi$ | $0.00$ | $0.00$ | $\pi/2$ | $-89^\circ \text{ a } +52^\circ$ |
| **J4 (Antebrazo)** | $0$ | $222.63$ | $0.00$ | $-\pi/2$ | $-165^\circ \text{ a } +165^\circ$ |
| **J5 (Muñeca Pitch)** | $0$ | $0.00$ | $0.00$ | $\pi/2$ | $-105^\circ \text{ a } +105^\circ$ |
| **J6 (Brida Yaw)** | $0$ | $36.25$ | $0.00$ | $0$ | $-155^\circ \text{ a } +155^\circ$ |

---

## Requisitos de Ejecución

* **Python 3.9+:** `opencv-python`, `numpy`, `pyserial`, `tkinter` (para ejecutar `AR4.py`).
* **Arduino IDE 2.x:** Placas Arduino Mega 2560 y Teensyduino (para compilar y cargar `bandasensores.ino`).
* **MATLAB R2022b o superior (u Online):** Signal Processing Toolbox (para ejecutar los scripts `.m`).
