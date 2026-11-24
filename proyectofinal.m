%% SIMULACIÓN CINEMÁTICA 6 GDL (AR4-MK2) - ETIQUETAS J1 A J6 SEPARADAS
clear; clc; close all;

% 1. Parámetros Denavit-Hartenberg estándar [theta_offset, d, a, alpha] (mm y rad)
dh_base = [ 0,      169.77,  64.20, -pi/2;   % J1: Cintura (Base)
           -pi/2,     0.00, 305.00,  0.00;   % J2: Hombro
            pi,       0.00,   0.00,  pi/2;   % J3: Codo
            0,      222.63,   0.00, -pi/2;   % J4: Rotación Antebrazo
            0,        0.00,   0.00,  pi/2;   % J5: Cabeceo Muñeca
            0,       36.25,   0.00,  0.00 ]; % J6: Giro Brida (TCP)

% 2. Validación del punto de prueba (idéntico a RoboDK y Excel)
q_prueba = deg2rad([120, 0, 40, 0, 90, 0]);
[pts_prueba, ejes_z_p, T_final] = cinematica_directa_6gdl(q_prueba, dh_base);
pos_tcp = T_final(1:3, 4);

fprintf('--- VALIDACIÓN CINEMÁTICA DIRECTA (6 GDL) ---\n');
fprintf('Coordenadas TCP: X = %.3f mm, Y = %.3f mm, Z = %.3f mm\n', ...
        pos_tcp(1), pos_tcp(2), pos_tcp(3));

% 3. Trayectoria Articular Conservadora (PTP) moviendo los 6 GDL
q_toma     = deg2rad([  0, -15,  35,  15,  70, -20]); % Toma sobre banda
q_seguro   = deg2rad([ 45, -30,  20,  30,  80,  10]); % Punto de vía elevado
q_deposito = deg2rad([110, -10,  40, -25,  60,  45]); % Depósito en estación

N = 60;
q_traj = [interp_cubica(q_toma, q_seguro, N/2); interp_cubica(q_seguro, q_deposito, N/2)];

ruta_tcp = zeros(N, 3);
for k = 1:N
    [~, ~, Tk] = cinematica_directa_6gdl(q_traj(k,:), dh_base);
    ruta_tcp(k,:) = Tk(1:3, 4)';
end

% 4. Gráfica 3D con los 6 GDL y etiquetas separadas con líneas guía
figure('Color', 'w', 'Position', [60, 60, 1180, 560]);

% Subgráfica 1: Validación Punto de Prueba
subplot(1, 2, 1);
dibujar_robot_6gdl(pts_prueba, ejes_z_p, T_final, 'Validación 6 GDL: Punto Prueba [120°,0°,40°,0°,90°,0°]');

% Subgráfica 2: Trayectoria Pick-and-Place 6 GDL
subplot(1, 2, 2);
[pts_dep, ejes_z_d, T_dep] = cinematica_directa_6gdl(q_deposito, dh_base);
dibujar_robot_6gdl(pts_dep, ejes_z_d, T_dep, 'Trayectoria 6 GDL: Banda -> Estación (2.0 s)');
hold on;
plot3(ruta_tcp(:,1), ruta_tcp(:,2), ruta_tcp(:,3), 'r-', 'LineWidth', 2.5);
plot3(ruta_tcp(1,1), ruta_tcp(1,2), ruta_tcp(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(ruta_tcp(end,1), ruta_tcp(end,2), ruta_tcp(end,3), 'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

%% --- FUNCIONES AUXILIARES ---
function [puntos, ejes_z, T] = cinematica_directa_6gdl(q, dh_base)
    T = eye(4);
    puntos = zeros(7, 3);
    ejes_z = zeros(6, 3);
    for i = 1:6
        ejes_z(i, :) = T(1:3, 3)';
        theta = q(i) + dh_base(i, 1);
        d     = dh_base(i, 2);
        a     = dh_base(i, 3);
        alpha = dh_base(i, 4);
        
        Ai = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
              sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,           sin(alpha),             cos(alpha),            d;
              0,           0,                      0,                     1];
        T = T * Ai;
        puntos(i+1, :) = T(1:3, 4)';
    end
end

function q_mat = interp_cubica(q0, qf, pasos)
    s = linspace(0, 1, pasos)';
    perfil = 3*(s.^2) - 2*(s.^3);
    q_mat = q0 + perfil .* (qf - q0);
end

function dibujar_robot_6gdl(pts, ejes_z, T_tcp, titulo)
    % Dibujar eslabones físicos
    plot3(pts(:,1), pts(:,2), pts(:,3), '-', 'Color', [0.1 0.25 0.45], 'LineWidth', 4);
    hold on;
    
    % Desplazamientos bien separados para cada etiqueta [dx, dy, dz] en mm
    offsets_txt = [  55,   0,  -20;   % J1 (Base)
                     65,  20,    0;   % J2 (Hombro)
                    -85, -35,   55;   % J3 (Codo - separado hacia arriba/izquierda)
                     85, -45,   45;   % J4 (Antebrazo - separado hacia la derecha)
                    -95,  45,  -45;   % J5 (Muñeca - separado hacia abajo/izquierda)
                     85,  55,  -45 ]; % J6 (Brida/TCP - separado hacia abajo/derecha)
                 
    descripciones = {'J1 (Base)', 'J2 (Hombro)', 'J3 (Codo)', ...
                     'J4 (Antebrazo)', 'J5 (Muñeca)', 'J6 (Brida)'};
                 
    for j = 1:6
        p_j = pts(j, :);
        z_j = ejes_z(j, :) * 38;
        
        % Nodo de articulación
        plot3(p_j(1), p_j(2), p_j(3), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', [0.95 0.75 0.1]);
        
        % Línea punteada magenta del eje de rotación
        plot3([p_j(1)-z_j(1) p_j(1)+z_j(1)], [p_j(2)-z_j(2) p_j(2)+z_j(2)], ...
              [p_j(3)-z_j(3) p_j(3)+z_j(3)], 'm--', 'LineWidth', 1.3);
          
        % Posición de la etiqueta con línea guía fina para evitar amontonamiento
        p_txt = p_j + offsets_txt(j, :);
        plot3([p_j(1) p_txt(1)], [p_j(2) p_txt(2)], [p_j(3) p_txt(3)], ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1);
        text(p_txt(1), p_txt(2), p_txt(3), descripciones{j}, ...
             'FontWeight', 'bold', 'FontSize', 8.5, 'Color', [0.35 0 0.45], ...
             'BackgroundColor', [1 1 1 0.75], 'Margin', 1);
    end
    
    % Ejes coordenados del efector final (TCP)
    L_eje = 65;
    p_tcp = T_tcp(1:3, 4);
    quiver3(p_tcp(1), p_tcp(2), p_tcp(3), T_tcp(1,1)*L_eje, T_tcp(2,1)*L_eje, T_tcp(3,1)*L_eje, 'r', 'LineWidth', 2);
    quiver3(p_tcp(1), p_tcp(2), p_tcp(3), T_tcp(1,2)*L_eje, T_tcp(2,2)*L_eje, T_tcp(3,2)*L_eje, 'g', 'LineWidth', 2);
    quiver3(p_tcp(1), p_tcp(2), p_tcp(3), T_tcp(1,3)*L_eje, T_tcp(2,3)*L_eje, T_tcp(3,3)*L_eje, 'b', 'LineWidth', 2);
    
    grid on; axis equal;
    xlim([-500 600]); ylim([-500 600]); zlim([0 780]);
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title(titulo);
    view(135, 25);
end