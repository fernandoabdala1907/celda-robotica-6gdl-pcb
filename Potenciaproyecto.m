%% DIMENSIONAMIENTO ELÉCTRICO, PWM Y FUENTES - CELDA ROBÓTICA 6 GDL (PCB)
% Proyecto Final - Maestría en Automatización y Robótica (Tecmilenio)
% Autor: Fernando Abdalá Gómez
% Subsistema: Banda transportadora lineal (Motorreductor 12 VCD) y bus de celda

clc; clear; close all;

%% 1. DATOS DEL MOTORREDUCTOR DE BANDA TRANSPORTADORA Y BUS DE CELDA
Vdc_banda  = 12.0;    % Tensión nominal del motorreductor de banda [VCD]
Inom_banda = 1.2;     % Corriente nominal en operación continua [A]
Iarr_banda = 2.5;     % Corriente pico de arranque / bloqueo [A]
fpwm       = 20000;   % Frecuencia de conmutación PWM (20 kHz, fuera de rango audible) [Hz]
D_inicial  = 0.75;    % Ciclo de trabajo calibrado para avance estable en inspección (75%)
margen     = 0.25;    % Margen de seguridad de diseño (+25%)

%% 2. CÁLCULOS ELÉCTRICOS Y PWM DE LA BANDA TRANSPORTADORA
Pnom_banda = Vdc_banda * Inom_banda;              % Potencia nominal [W]
Parr_banda = Vdc_banda * Iarr_banda;              % Potencia pico de arranque [W]
Ifuente_12V = Inom_banda * (1 + margen);          % Corriente mínima recomendada fuente 12 V [A]

T_pwm = 1 / fpwm;                                 % Período PWM [s]
Ton   = D_inicial * T_pwm;                        % Tiempo en alto (ON) [s]
Toff  = (1 - D_inicial) * T_pwm;                  % Tiempo en bajo (OFF) [s]

D = 0:0.1:1;                                      % Vector de ciclos de trabajo (0% a 100%)
Vpromedio_D = D * Vdc_banda;                      % Tensión media aplicada al motor [V]
Vprom_op    = D_inicial * Vdc_banda;              % Tensión media en punto de operación (75%) [V]

%% 3. BALANCE DE POTENCIA INTEGRAL DE LA CELDA (24 V / 12 V / 5 V)
P_motores_24V = 24.0 * 4.8;   % 6 motores a pasos NEMA 17/23 + frenos J2/J3 (~115.2 W nom.)
P_banda_12V   = Pnom_banda;   % Motorreductor de banda transportadora (14.4 W nom.)
P_control_5V  = 5.0 * 1.5;    % Teensy + Arduino Mega + relés + sensores IR + servo (7.5 W nom.)
P_total_celda = P_motores_24V + P_banda_12V + P_control_5V;

%% 4. REPORTE EN CONSOLA
fprintf('\n==============================================================\n');
fprintf(' DIMENSIONAMIENTO ELÉCTRICO Y PWM - CELDA ROBÓTICA 6 GDL (PCB)\n');
fprintf('==============================================================\n');
fprintf('Tensión nominal banda:        %.2f VCD\n', Vdc_banda);
fprintf('Corriente nominal / arranque: %.2f A / %.2f A\n', Inom_banda, Iarr_banda);
fprintf('Tensión promedio (D = 75%%):   %.2f V\n', Vprom_op);
fprintf('Potencia nominal banda:       %.2f W (Arranque: %.2f W)\n', Pnom_banda, Parr_banda);
fprintf('Corriente diseño fuente 12V:  %.2f A (con margen +25%%)\n', Ifuente_12V);
fprintf('Potencia total celda (nom.):  %.2f W (24V + 12V + 5V)\n', P_total_celda);
fprintf('\n--- PARÁMETROS PWM (20 kHz) ---\n');
fprintf('Período PWM: %.2f us | Tiempo ON: %.2f us | Tiempo OFF: %.2f us\n', ...
    T_pwm*1e6, Ton*1e6, Toff*1e6);

%% 5. GRÁFICAS DE VALIDACIÓN
figure('Name', 'Dimensionamiento PWM y Potencia - Celda PCB', 'Color', 'w', 'Position', [100 100 1050 380]);

subplot(1,3,1);
plot(D * 100, Vpromedio_D, '-o', 'Color', [0 0.48 0.53], 'LineWidth', 2); hold on;
plot(D_inicial*100, Vprom_op, 'rs', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
grid on; xlabel('Ciclo de trabajo D [%]'); ylabel('Tensión promedio [V]');
title('Curva V_{media} vs. Ciclo PWM (12 VCD)');
legend('Linealidad PWM', 'Operación (75% -> 9.0 V)', 'Location', 'northwest');

subplot(1,3,2);
t = linspace(0, 3*T_pwm, 1000);
senal_PWM = double(mod(t, T_pwm) < Ton);
plot(t * 1e6, senal_PWM, 'Color', [0.11 0.21 0.36], 'LineWidth', 2);
grid on; xlabel('Tiempo [\mus]'); ylabel('Estado lógico');
title('Señal PWM a 20 kHz (D = 75%)'); ylim([-0.2 1.2]);

subplot(1,3,3);
b = bar([Pnom_banda, Parr_banda, P_total_celda], 'FaceColor', 'flat');
b.CData(1,:) = [0 0.48 0.53];
b.CData(2,:) = [0.85 0.47 0.02];
b.CData(3,:) = [0.11 0.21 0.36];
grid on; set(gca, 'XTickLabel', {'Banda Nom.', 'Banda Arr.', 'Celda Total'});
ylabel('Potencia [W]'); title('Balance de Potencia del Sistema');