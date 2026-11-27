%% CINEMÁTICA DIRECTA E INVERSA 6 GDL (AR4-MK2) - CELDA ROBÓTICA PCB
% Proyecto Final - Maestría en Automatización y Robótica (Tecmilenio)
% Autor: Fernando Abdalá Gómez

clear; clc; close all;

%% 1. PARÁMETROS DENAVIT-HARTENBERG ESTÁNDAR DEL MANIPULADOR 6R [mm y rad]
% Columnas: [theta_offset, d, a, alpha]
dh_base = [  0,      169.77,   64.20, -pi/2;   % J1: Cintura (Base)
    -pi/2,      0.00,  305.00,  0.00;   % J2: Hombro
    pi,        0.00,    0.00,  pi/2;   % J3: Codo
    0,      222.63,    0.00, -pi/2;   % J4: Rotación Antebrazo
    0,        0.00,    0.00,  pi/2;   % J5: Cabeceo Muñeca
    0,       36.25,    0.00,  0.00 ]; % J6: Giro Brida (TCP)

% Función de Matriz Homogénea D-H estándar
DH = @(theta, d, a, alpha) [
    cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
    sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
    0,           sin(alpha),            cos(alpha),            d;
    0,           0,                     0,                     1
    ];

%% 2. CINEMÁTICA DIRECTA (FK) - VALIDACIÓN CRUZADA ROBODK / EXCEL
q_deg = [120, 0, 40, 0, 90, 0];
q_rad = deg2rad(q_deg);

T = eye(4);
pts = zeros(3, 7); % Origen base + 6 articulaciones
for i = 1:6
    A_i = DH(q_rad(i) + dh_base(i,1), dh_base(i,2), dh_base(i,3), dh_base(i,4));
    T = T * A_i;
    pts(:, i+1) = T(1:3, 4);
end

pos_tcp = T(1:3, 4);
R_tcp   = T(1:3, 1:3);

fprintf('==============================================================\n');
fprintf(' VALIDACIÓN CINEMÁTICA 6 GDL (AR4-MK2) - CELDA ROBÓTICA PCB\n');
fprintf('==============================================================\n');
fprintf('Configuración Articular: [%.1f°, %.1f°, %.1f°, %.1f°, %.1f°, %.1f°]\n', q_deg);
fprintf('Coordenadas TCP (FK):    X = %.3f mm, Y = %.3f mm, Z = %.3f mm\n', ...
    pos_tcp(1), pos_tcp(2), pos_tcp(3));

%% 3. CINEMÁTICA INVERSA DESACOPLADA (IK: CENTRO DE MUÑECA + EULER ZYX)
d1 = dh_base(1,2); a1 = dh_base(1,3); a2 = dh_base(2,3);
d4 = dh_base(4,2); d6 = dh_base(6,2);

% Centro de muñeca esférica P_w = P_tcp - d6 * z6
P_w = pos_tcp - d6 * R_tcp(:,3);
q1_ik = atan2(P_w(2), P_w(1));

r_w = hypot(P_w(1), P_w(2)) - a1;
s_w = P_w(3) - d1;
D_ik = (r_w^2 + s_w^2 - a2^2 - d4^2) / (2 * a2 * d4);
% Solución codo arriba compatible con convención AR4
beta = atan2(sqrt(1 - D_ik^2), D_ik);
q3_ik = beta - pi/2;
q2_ik = pi/2 - (atan2(s_w, r_w) + atan2(d4*sin(beta), a2 + d4*cos(beta)));

% Orientación de muñeca R_3^6 = (R_0^3)^T * R_tcp
T03 = eye(4);
q_reg = [q1_ik, q2_ik, q3_ik];
for i = 1:3
    T03 = T03 * DH(q_reg(i) + dh_base(i,1), dh_base(i,2), dh_base(i,3), dh_base(i,4));
end
R36 = T03(1:3,1:3)' * R_tcp;
q5_ik = atan2(hypot(R36(1,3), R36(2,3)), R36(3,3));
q4_ik = atan2(R36(2,3), R36(1,3));
q6_ik = atan2(R36(3,2), -R36(3,1));

q_ik_deg = rad2deg([q1_ik, q2_ik, q3_ik, q4_ik, q5_ik, q6_ik]);
fprintf('Ángulos Recuperados (IK): [%.2f°, %.2f°, %.2f°, %.2f°, %.2f°, %.2f°]\n\n', q_ik_deg);

%% 4. VISUALIZACIÓN ESPACIAL 3D DEL MANIPULADOR 6 GDL
figure('Name', 'Cinemática 6 GDL - Celda PCB', 'Color', 'w', 'Position', [100 100 750 580]);
plot3(pts(1,:), pts(2,:), pts(3,:), '-o', 'Color', [0.11 0.21 0.36], ...
    'LineWidth', 4, 'MarkerSize', 7, 'MarkerFaceColor', [0.85 0.47 0.02]);
hold on; grid on; axis equal; view(135, 25);

% Ejes locales del TCP (X rojo, Y verde, Z azul)
quiver3(pos_tcp(1), pos_tcp(2), pos_tcp(3), R_tcp(1,1), R_tcp(2,1), R_tcp(3,1), 80, 'r', 'LineWidth', 2);
quiver3(pos_tcp(1), pos_tcp(2), pos_tcp(3), R_tcp(1,2), R_tcp(2,2), R_tcp(3,2), 80, 'g', 'LineWidth', 2);
quiver3(pos_tcp(1), pos_tcp(2), pos_tcp(3), R_tcp(1,3), R_tcp(2,3), R_tcp(3,3), 80, 'b', 'LineWidth', 2);

xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
title('Manipulador 6 GDL (AR4-MK2) - Punto de Validación D-H');
xlim([-500 500]); ylim([-500 500]); zlim([0 750]);