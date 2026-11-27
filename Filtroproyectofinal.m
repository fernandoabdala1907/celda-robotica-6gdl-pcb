%% ACONDICIONAMIENTO DE SEÑALES (NOTCH 60 Hz + BUTTERWORTH) Y CONTROL PID (EJE J2)
% Proyecto Final - Maestría en Automatización y Robótica (Tecmilenio)
% Autor: Fernando Abdalá Gómez

clear; clc; close all;
rng(42); % Semilla reproducible

%% 1. PARÁMETROS DE MUESTREO Y MODELO DINÁMICO DEL HOMBRO (J2)
fs = 1000;          % Frecuencia de muestreo del lazo digital [Hz]
Ts = 1 / fs;        % Período de muestreo (1 ms) [s]
T_sim = 3.0;        % Duración de simulación [s]
t = 0:Ts:T_sim;
N = numel(t);
fNyquist = fs / 2;  % 500 Hz

% Parámetros físicos equivalentes de la articulación J2 (NEMA 23 + caja planetaria)
J_eq = 0.01;        % Inercia equivalente reflejada [kg*m^2]
B_v  = 0.08;        % Amortiguamiento viscoso [N*m*s/rad]
Kp = 25; Ki = 12; Kd = 2;   % Ganancias PID (estables por Routh-Hurwitz)
uMin = -8.0; uMax = 8.0;    % Límites de saturación de par [N*m]

%% 2. TRAYECTORIA CÚBICA DE REFERENCIA (0° -> 90° EN Tf = 2.0 s)
Tf = 2.0; q0 = 0; qf = pi/2;
a2 = 3*(qf - q0)/(Tf^2);
a3 = -2*(qf - q0)/(Tf^3);
qd = zeros(1, N);
for k = 1:N
    if t(k) <= Tf
        qd(k) = a2*t(k)^2 + a3*t(k)^3;
    else
        qd(k) = qf;
    end
end

%% 3. DISEÑO DE FILTROS DIGITALES (NOTCH 60 Hz + BUTTERWORTH PASA BAJAS)
% Filtro IIR Notch para eliminar interferencia de red eléctrica (60 Hz)
f0 = 60; Q = 25;
w0 = 2*pi*f0/fs;
r_n = 1 - pi*(f0/Q)/fs;
bn = [1, -2*cos(w0), 1];
an = [1, -2*r_n*cos(w0), r_n^2];
bn = bn * sum(an) / sum(bn); % Ganancia unitaria en DC

% Filtro Butterworth pasa bajas de 2do orden (fc = 80 Hz) para ruido PWM/cuantización
fc = 80;
Wn = fc / fNyquist;
[bb, ab] = butter(2, Wn, 'low');

%% 4. SIMULACIÓN EN LAZO CERRADO: MEDICIÓN RUIDOSA -> FILTRADO -> PID + ANTI-WINDUP
q_real  = zeros(1, N);
qp_real = zeros(1, N);
q_meas  = zeros(1, N);
q_notch = zeros(1, N);
q_filt  = zeros(1, N);
u_sat   = zeros(1, N);
e_track = zeros(1, N);

I_term = 0; e_prev = 0; D_filt = 0;
alpha_d = Ts / (0.008 + Ts); % Filtro derivativo para evitar amplificar ruido

for k = 1:N-1
    % Simulación de ruido aleatorio PWM + interferencia de red de 60 Hz
    ruido_total = 0.008*randn() + 0.015*sin(2*pi*60*t(k));
    q_meas(k) = q_real(k) + ruido_total;
    
    % Aplicación causal en tiempo real: Notch 60 Hz -> Butterworth 80 Hz
    if k >= 3
        q_notch(k) = bn(1)*q_meas(k) + bn(2)*q_meas(k-1) + bn(3)*q_meas(k-2) ...
                   - an(2)*q_notch(k-1) - an(3)*q_notch(k-2);
        q_filt(k)  = bb(1)*q_notch(k) + bb(2)*q_notch(k-1) + bb(3)*q_notch(k-2) ...
                   - ab(2)*q_filt(k-1) - ab(3)*q_filt(k-2);
    else
        q_notch(k) = q_meas(k);
        q_filt(k)  = q_meas(k);
    end
    
    % Controlador PID discreto con anti-windup
    e = qd(k) - q_filt(k);
    e_track(k) = qd(k) - q_real(k);
    D_raw = (e - e_prev) / Ts;
    D_filt = (1 - alpha_d)*D_filt + alpha_d*D_raw;
    I_tent = I_term + Ts * e;
    
    u_unsat = Kp*e + Ki*I_tent + Kd*D_filt;
    u_c = min(max(u_unsat, uMin), uMax);
    
    if u_c == u_unsat || sign(e) ~= sign(u_unsat)
        I_term = I_tent; % Integración condicional anti-windup
    end
    u_sat(k) = u_c;
    e_prev = e;
    
    % Dinámica física de la articulación J2
    qpp = (u_c - B_v*qp_real(k)) / J_eq;
    qp_real(k+1) = qp_real(k) + Ts * qpp;
    q_real(k+1)  = q_real(k)  + Ts * qp_real(k);
end
q_meas(end) = q_real(end); q_filt(end) = q_real(end); u_sat(end) = u_sat(end-1);

%% 5. INDICADORES DE DESEMPEÑO (IAE, ISE, ERROR FINAL)
IAE = sum(abs(e_track)) * Ts;
ISE = sum(e_track.^2) * Ts;
fprintf('==============================================================\n');
fprintf(' RESULTADOS DE FILTRADO Y CONTROL PID (EJE J2 - CELDA PCB)\n');
fprintf('==============================================================\n');
fprintf('IAE = %.5f rad*s | ISE = %.6f rad^2*s | Error final = %.5f rad\n', ...
        IAE, ISE, abs(e_track(end-1)));

%% 6. GRÁFICAS DE RESPUESTA EN FRECUENCIA Y SEGUIMIENTO
figure('Name', 'Filtrado y Control PID - Celda Robótica PCB', 'Color', 'w', 'Position', [80 80 1100 620]);

subplot(2,2,1);
idx_z = (t >= 0.8) & (t <= 1.15);
plot(t(idx_z), q_meas(idx_z), 'Color', [0.6 0.65 0.7], 'DisplayName', 'Cruda (Ruido + 60 Hz)'); hold on;
plot(t(idx_z), q_filt(idx_z), 'Color', [0 0.48 0.53], 'LineWidth', 1.8, 'DisplayName', 'Filtrada (Notch + BW)');
plot(t(idx_z), qd(idx_z), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Referencia q_d');
grid on; xlabel('Tiempo [s]'); ylabel('Posición [rad]');
title('Acondicionamiento de Señal en Lazo'); legend('Location', 'northwest');

subplot(2,2,2);
[Hb, f_vec] = freqz(bb, ab, 2048, fs);
[Hn, ~]     = freqz(bn, an, 2048, fs);
plot(f_vec, 20*log10(abs(Hb.*Hn)), 'Color', [0.11 0.21 0.36], 'LineWidth', 1.5);
xlim([0 200]); ylim([-50 5]); grid on;
xlabel('Frecuencia [Hz]'); ylabel('Magnitud [dB]');
title('Respuesta en Frecuencia: Notch 60 Hz + Butterworth 80 Hz');

subplot(2,2,3);
plot(t, rad2deg(qd), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Referencia q_d(t)'); hold on;
plot(t, rad2deg(q_real), 'Color', [0 0.48 0.53], 'LineWidth', 1.8, 'DisplayName', 'Respuesta Real q(t)');
grid on; xlabel('Tiempo [s]'); ylabel('Ángulo [°]');
title('Seguimiento PID + Anti-Windup en Eje J2 (0° -> 90°)'); legend('Location', 'southeast');

subplot(2,2,4);
plot(t, u_sat, 'Color', [0.11 0.21 0.36], 'LineWidth', 1.3, 'DisplayName', 'Par u(t)'); hold on;
yline(uMax, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Saturación (±8 N·m)');
yline(uMin, 'r--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
grid on; xlabel('Tiempo [s]'); ylabel('Par [N·m]'); ylim([-9.5 9.5]);
title('Esfuerzo de Control u(t) Acotado'); legend('Location', 'northeast');