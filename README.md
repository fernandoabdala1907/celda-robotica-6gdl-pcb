# Celda Robótica de 6 GDL para Transferencia y Posicionamiento de PCB

**Proyecto Final — Maestría en Automatización y Robótica (Universidad Tecmilenio)**  
* **Autor:** Fernando Abdalá Gómez  
* **Docente:** Ingrith Yuritsa Páez Pidiache  
* **Demostración en video:** https://youtu.be/MoI7Jq5fUXg

## Estructura del Código Fuente
* `/Firmware_Control`: Código en C++ para control determinista de movimiento (Teensy) y periféricos de celda (Arduino Mega 2560).
* `/Vision_Artificial`: Scripts de detección cenital, cálculo de centroide (X, Y) y orientación angular.
* `/Simulacion_MATLAB`: Modelado cinemático directo/inverso (D-H) y planificación de trayectorias 6 GDL.
* `/Rutinas_AR4`: Programas operativos de secuencia Pick-and-Place (`proy1.ar4` / `tesis1.ar4`).
