%% graficar_resultados.m
% Este script grafica los resultados de la simulación
% Ejecutar DESPUÉS de correr la simulación de Simulink

% ========== VERIFICAR QUE EXISTEN LOS DATOS ==========
if ~exist('out', 'var')
    error('No se encontró la variable "out". Ejecuta primero la simulación de Simulink.');
end

if ~exist('t', 'var') || ~exist('qd_deseado', 'var')
    error('No se encontraron las variables del setup. Ejecuta primero "setup_robot.m".');
end

% ========== EXTRAER DATOS DE SIMULINK ==========
Q_nominal = out.Q_outNP;      % Robot NO perturbado
rposNP = out.RealPosNP.Data;

Q_perturbado = out.Q_outP;    % Robot perturbado
rposP = out.RealPosP.Data;

Pos = Posicion_Cartesiana;

% Verificar longitudes
if size(Q_nominal, 1) ~= length(t)
    warning('El número de pasos del robot nominal no coincide con el vector de tiempo "t".');
end
if size(Q_perturbado, 1) ~= length(t)
    warning('El número de pasos del robot perturbado no coincide con el vector de tiempo "t".');
end

% ========== GRÁFICO 1: TRAYECTORIAS CARTESIANAS ==========
figure('Name', 'Trayectorias Cartesianas', 'NumberTitle', 'off');
subplot(3,1,1);
plot(t, [X, Y], 'LineWidth', 1.5);
title('Posición Polinómica (X, Y)');
legend('X', 'Y');
grid on;

subplot(3,1,2);
plot(t, [X_d, Y_d], 'LineWidth', 1.5);
title('Velocidad Polinómica (X_d, Y_d)');
legend('X_d', 'Y_d');
grid on;

subplot(3,1,3);
plot(t, [X_dd, Y_dd], 'LineWidth', 1.5);
title('Aceleración Polinómica (X_dd, Y_dd)');
legend('X_dd', 'Y_dd');
xlabel('Tiempo (s)');
grid on;

% ========== GRÁFICO 2: COMPARACIÓN NOMINAL vs PERTURBADO ==========
figure('Name', 'Angulos Joints', 'NumberTitle', 'off');

subplot(2,1,1);
plot(t, qd_deseado(:, 1), 'k--', ...
     t, Q_nominal(:, 1), 'b', ...
     t, Q_perturbado(:, 1), 'r', 'LineWidth', 1.5);
title('Articulación 1 (q1): Deseado vs Sin Perturbar vs Perturbado');
ylabel('Ángulo (rad)');
legend('q1 Deseado', 'q1 Sin Perturbar', 'q1 Perturbado', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, qd_deseado(:, 2), 'k--', ...
     t, Q_nominal(:, 2), 'b', ...
     t, Q_perturbado(:, 2), 'r', 'LineWidth', 1.5);
title('Articulación 2 (q2): Deseado vs Sin Perturbar vs Perturbado');
xlabel('Tiempo (s)');
ylabel('Ángulo (rad)');
legend('q2 Deseado', 'q2 Sin Perturbar', 'q2 Perturbado', 'Location', 'best');
grid on;

% ========== GRÁFICO 3: POSICIÓN DESEADA VS REAL EN TIEMPO ==========
figure('Name', 'Posición Deseada vs Real (Espacio Cartesiano)', 'NumberTitle', 'off');

subplot(2,1,1);
plot(t, Pos(:,2), 'k--', ...
     t, rposNP(:, 1), 'b', ...
     t, rposP(:, 1), 'r', 'LineWidth', 1.5);
title('Coordenada X del Efector Final');
ylabel('X (m)');
legend('Deseada', 'Real (Sin Perturbar)', 'Real (Perturbado)', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t, Pos(:,3), 'k--', ...
     t, rposNP(:, 2), 'b', ...
     t, rposP(:, 2), 'r', 'LineWidth', 1.5);
title('Coordenada Y del Efector Final');
xlabel('Tiempo (s)');
ylabel('Y (m)');
legend('Deseada', 'Real (Sin Perturbar)', 'Real (Perturbado)', 'Location', 'best');
grid on;

% ========== GRÁFICO 4: TRAYECTORIA EN EL PLANO XY ==========
figure('Name', 'Trayectoria del Efector Final (Plano XY)', 'NumberTitle', 'off');
hold on;

plot(Pos(:,2), Pos(:,3), 'k--', ...      % Trayectoria deseada
     rposNP(:,1), rposNP(:,2), 'b', ...  % Trayectoria real (nominal)
     rposP(:,1), rposP(:,2), 'r', 'LineWidth', 1.5);    % Trayectoria real (perturbado)

title('Trayectoria del Efector Final en Espacio Cartesiano');
xlabel('X (m)');
ylabel('Y (m)');
legend('Deseada', 'Real (Nominal)', 'Real (Perturbado)', 'Location', 'best');
axis equal;     % Mantiene la escala 1:1 para X e Y
grid on;
hold off;

% ========== ANIMACIÓN 3D: ROBOT NOMINAL ==========
% Definir los puntos de la pared
punto1_pared = [2; 0; -1];
punto2_pared = [0; 2; -1];
altura_pared = 1.5;

% Crear vértices del rectángulo de la pared
vertices_pared = [
    punto1_pared(1), punto1_pared(2), punto1_pared(3);
    punto2_pared(1), punto2_pared(2), punto2_pared(3);
    punto2_pared(1), punto2_pared(2), altura_pared;
    punto1_pared(1), punto1_pared(2), altura_pared
];

% ===== ANIMACIÓN ROBOT NOMINAL =====
figure('Name', 'Animación Robot NOMINAL con Pared', 'NumberTitle', 'off');
hold on;

% Dibujar pared
patch('Vertices', vertices_pared, 'Faces', [1 2 3 4], ...
      'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.5, ...
      'EdgeColor', 'k', 'LineWidth', 2);

text(punto1_pared(1), punto1_pared(2), 0, ' P1(2,0)', ...
     'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');
text(punto2_pared(1), punto2_pared(2), 0, ' P2(0,2)', ...
     'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');

% Animar robot nominal
rob.plot(Q_nominal, 'trail', 'b-', 'workspace', [-1 3 -1 3 -1 2]);
title('Robot NOMINAL (Sin Perturbaciones)');
grid on;
hold off;

% ===== ANIMACIÓN ROBOT PERTURBADO =====
figure('Name', 'Animación Robot PERTURBADO con Pared', 'NumberTitle', 'off');
hold on;

% Dibujar pared
patch('Vertices', vertices_pared, 'Faces', [1 2 3 4], ...
      'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.5, ...
      'EdgeColor', 'k', 'LineWidth', 2);

text(punto1_pared(1), punto1_pared(2), 0, ' P1(2,0)', ...
     'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');
text(punto2_pared(1), punto2_pared(2), 0, ' P2(0,2)', ...
     'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');

% Animar robot perturbado
rob2.plot(Q_perturbado, 'trail', 'r-', 'workspace', [-1 3 -1 3 -1 2]);
title('Robot PERTURBADO (80%)');
grid on;
hold off;