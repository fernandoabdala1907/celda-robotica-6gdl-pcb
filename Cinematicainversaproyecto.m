%% PLANIFICACIÓN DE TRAYECTORIA CÚBICA 6 GDL (BANDA -> ESTACIÓN PCB)
% Proyecto Final - Maestría en Automatización y Robótica (Tecmilenio)
% Autor: Fernando Abdalá Gómez

clear; clc; close all;

%% 1. PARÁMETROS D-H DEL MANIPULADOR 6 GDL (AR4-MK2) [mm y rad]
dh_base = [  0,      169.77,   64.20, -pi/2;   % J1
    -pi/2,      0.00,  305.00,  0.00;   % J2
    pi,        0.00,    0.00,  pi/2;   % J3
    0,      222.63,    0.00, -pi/2;   % J4
    0,        0.00,    0.00,  pi/2;   % J5
    0,       36.25,    0.00,  0.00 ]; % J6

tf = 2.4;      % Tiempo nominal de transferencia Pick-and-Place [s]
dt = 0.03;     % Paso de simulación [s]
t  = 0:dt:tf;
N  = length(t);

%% 2. CONFIGURACIONES ARTICULARES DE TOMA (BANDA) Y DEPÓSITO (ESTACIÓN)
% Obtenidas de las rutinas de celda (AR4.py / tesis1.ar4)
q_i = deg2rad([-12.0,  28.5,  38.0,  0.0,  23.5, -12.0]); % Toma sobre banda negra
q_f = deg2rad([-81.5,  32.0,  34.0,  0.0,  24.0,  15.0]); % Depósito en bandeja receptora

%% 3. GENERACIÓN DE TRAYECTORIA POLINOMIAL CÚBICA SUAVE (q_dot(0)=q_dot(tf)=0)
q_t  = zeros(6, N);
dq_t = zeros(6, N);

for j = 1:6
    a0 = q_i(j);
    a1 = 0;
    a2 =  3 * (q_f(j) - q_i(j)) / (tf^2);
    a3 = -2 * (q_f(j) - q_i(j)) / (tf^3);

    q_t(j, :)  = a0 + a1.*t + a2.*(t.^2) + a3.*(t.^3);
    dq_t(j, :) = a1 + 2*a2.*t + 3*a3.*(t.^2);
end

%% 4. CÁLCULO DE TRAYECTORIA CARTESIANA DEL EFECTOR FINAL (FK)
pos_tcp = zeros(3, N);
pts_all = zeros(3, 7, N);

for k = 1:N
    T = eye(4);
    for i = 1:6
        th = q_t(i, k) + dh_base(i, 1);
        d  = dh_base(i, 2);
        a  = dh_base(i, 3);
        al = dh_base(i, 4);
        A_i = [cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th);
            sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th);
            0,        sin(al),          cos(al),         d;
            0,        0,                0,               1];
        T = T * A_i;
        pts_all(:, i+1, k) = T(1:3, 4);
    end
    pos_tcp(:, k) = T(1:3, 4);
end

fprintf('Inicio TCP (Banda):    [%.2f, %.2f, %.2f] mm\n', pos_tcp(:,1));
fprintf('Fin TCP (Estación):    [%.2f, %.2f, %.2f] mm\n', pos_tcp(:,end));

%% 5. VISUALIZACIÓN 3D Y PERFILES ARTICULARES
figure('Name', 'Planificación de Trayectoria 6 GDL - Celda PCB', 'Color', 'w', 'Position', [100 100 1100 500]);

subplot(1,2,1);
hold on; grid on; axis equal; view(130, 28);
plot3(pos_tcp(1,1), pos_tcp(2,1), pos_tcp(3,1), 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g', 'DisplayName', 'Toma Banda');
plot3(pos_tcp(1,end), pos_tcp(2,end), pos_tcp(3,end), 'rs', 'MarkerSize', 9, 'MarkerFaceColor', 'r', 'DisplayName', 'Estación PCB');
plot3(pos_tcp(1,:), pos_tcp(2,:), pos_tcp(3,:), 'r--', 'LineWidth', 1.8, 'DisplayName', 'Trayectoria TCP');
h_arm = plot3(pts_all(1,:,1), pts_all(2,:,1), pts_all(3,:,1), '-o', 'Color', [0.11 0.21 0.36], 'LineWidth', 3.5, 'DisplayName', 'Manipulador 6R');
xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('Trayectoria Pick-and-Place (2.4 s)');
xlim([-400 500]); ylim([-650 200]); zlim([0 700]);
legend('Location', 'northeast');

subplot(1,2,2);
plot(t, rad2deg(q_t'), 'LineWidth', 1.6); grid on;
xlabel('Tiempo [s]'); ylabel('Ángulo Articular [°]');
title('Perfiles Articulares Cúbicos q_1(t) ... q_6(t)');
legend('J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'Location', 'best');

for k = 1:2:N
    set(h_arm, 'XData', pts_all(1,:,k), 'YData', pts_all(2,:,k), 'ZData', pts_all(3,:,k));
    drawnow;
end