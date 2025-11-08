%% GRAFICACIÓN DE RESULTADOS
% Archivo: graficar_resultados.m
% Ejecutar DESPUÉS de correr el modelo Simulink

%% VERIFICAR QUE EXISTEN LOS DATOS
if ~exist('out', 'var')
    error('No se encontró la variable "out". Ejecuta primero el modelo Simulink.');
end

fprintf('=== GENERANDO GRÁFICOS ===\n');

%% EXTRACCIÓN DE DATOS - CASO NO PERTURBADO (NP)
if isstruct(out.Q_outNP) || isa(out.Q_outNP, 'timeseries')
    q_NP = out.Q_outNP.Data;  % [N x 2]
    t_NP = out.Q_outNP.Time;
else
    q_NP = out.Q_outNP;
    t_NP = t;
end

if isstruct(out.ForceNP) || isa(out.ForceNP, 'timeseries')
    force_NP = out.ForceNP.Data;
    t_force_NP = out.ForceNP.Time;
else
    force_NP = out.ForceNP;
    t_force_NP = t;
end

%% EXTRACCIÓN DE DATOS - CASO PERTURBADO (P)
if isstruct(out.Q_outP) || isa(out.Q_outP, 'timeseries')
    q_P = out.Q_outP.Data;  % [N x 2]
    t_P = out.Q_outP.Time;
else
    q_P = out.Q_outP;
    t_P = t;
end

if isstruct(out.ForceP) || isa(out.ForceP, 'timeseries')
    force_P = out.ForceP.Data;
    t_force_P = out.ForceP.Time;
else
    force_P = out.ForceP;
    t_force_P = t;
end

%% CALCULAR POSICIONES CARTESIANAS
N_NP = size(q_NP, 1);
N_P = size(q_P, 1);

fprintf('Calculando posiciones cartesianas...\n');

% Posiciones cartesianas - No Perturbado
pos_cart_NP = zeros(N_NP, 3);
for i = 1:N_NP
    T = rob.fkine(q_NP(i,:));
    pos_cart_NP(i,:) = T.t';
end

% Posiciones cartesianas - Perturbado
pos_cart_P = zeros(N_P, 3);
for i = 1:N_P
    T = rob2.fkine(q_P(i,:));
    pos_cart_P(i,:) = T.t';
end

%% === GRÁFICO 4a: ÁNGULOS DE JOINTS vs TIEMPO ===
fprintf('Generando gráfico 4a: Ángulos de joints...\n');

figure('Name', 'Ángulos de Joints', 'NumberTitle', 'off', 'Position', [100 100 1200 500]);

% No Perturbado
subplot(1,2,1);
plot(t_NP, rad2deg(q_NP(:,1)), 'b-', 'LineWidth', 1.5); hold on;
plot(t_NP, rad2deg(q_NP(:,2)), 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Ángulo (grados)', 'FontSize', 12);
title('Robot NO Perturbado', 'FontSize', 14, 'FontWeight', 'bold');
legend('Joint 1 (q_1)', 'Joint 2 (q_2)', 'Location', 'best');
set(gca, 'FontSize', 11);

% Perturbado
subplot(1,2,2);
plot(t_P, rad2deg(q_P(:,1)), 'b-', 'LineWidth', 1.5); hold on;
plot(t_P, rad2deg(q_P(:,2)), 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Ángulo (grados)', 'FontSize', 12);
title('Robot PERTURBADO (80%)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Joint 1 (q_1)', 'Joint 2 (q_2)', 'Location', 'best');
set(gca, 'FontSize', 11);

%% === GRÁFICO 4b: POSICIÓN CARTESIANA vs TIEMPO ===
fprintf('Generando gráfico 4b: Posición cartesiana vs tiempo...\n');

figure('Name', 'Posición Cartesiana vs Tiempo', 'NumberTitle', 'off', 'Position', [100 100 1400 900]);

% --- FILA 2: Robot NO Perturbado ---

subplot(2,1,1);
plot(t, X, 'b--', 'LineWidth', 1, 'DisplayName', 'X deseado'); hold on;
plot(t_NP, pos_cart_NP(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'X real');
plot(t, Y, 'r--', 'LineWidth', 1, 'DisplayName', 'Y deseado');
plot(t_NP, pos_cart_NP(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Y real');
grid on;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('Posición (m)', 'FontSize', 11);
title('NO Perturbado - Comparación X,Y', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best');
set(gca, 'FontSize', 10);

% --- FILA 3: Robot PERTURBADO ---

subplot(2,1,2);
plot(t, X, 'b--', 'LineWidth', 1, 'DisplayName', 'X deseado'); hold on;
plot(t_P, pos_cart_P(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'X real');
plot(t, Y, 'r--', 'LineWidth', 1, 'DisplayName', 'Y deseado');
plot(t_P, pos_cart_P(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Y real');
grid on;
xlabel('Tiempo (s)', 'FontSize', 11);
ylabel('Posición (m)', 'FontSize', 11);
title('PERTURBADO - Comparación X,Y', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best');
set(gca, 'FontSize', 10);

%% === GRÁFICO 4c: VISTA AÉREA XY ===
fprintf('Generando gráfico 4c: Vista aérea (plano XY)...\n');

figure('Name', 'Vista Aérea XY', 'NumberTitle', 'off', 'Position', [100 100 1200 500]);

% No Perturbado
subplot(1,2,1);
plot(pos_cart_NP(:,1), pos_cart_NP(:,2), 'b-', 'LineWidth', 2); hold on;
plot(pos_cart_NP(1,1), pos_cart_NP(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot(pos_cart_NP(end,1), pos_cart_NP(end,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 2);

% Dibujar línea de la pared
plot([2, 0], [0, 2], 'k--', 'LineWidth', 2);
text(2, 0, ' P1(2,0)', 'FontSize', 10, 'FontWeight', 'bold');
text(0, 2, ' P2(0,2)', 'FontSize', 10, 'FontWeight', 'bold');

grid on;
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
title('Robot NO Perturbado - Vista Superior', 'FontSize', 14, 'FontWeight', 'bold');
legend('Trayectoria', 'Inicio', 'Final', 'Pared', 'Location', 'best');
axis equal;
set(gca, 'FontSize', 11);

% Perturbado
subplot(1,2,2);
plot(pos_cart_P(:,1), pos_cart_P(:,2), 'b-', 'LineWidth', 2); hold on;
plot(pos_cart_P(1,1), pos_cart_P(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'LineWidth', 2);
plot(pos_cart_P(end,1), pos_cart_P(end,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'LineWidth', 2);

% Dibujar línea de la pared
plot([2, 0], [0, 2], 'k--', 'LineWidth', 2);
text(2, 0, ' P1(2,0)', 'FontSize', 10, 'FontWeight', 'bold');
text(0, 2, ' P2(0,2)', 'FontSize', 10, 'FontWeight', 'bold');

grid on;
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
title('Robot PERTURBADO (80%) - Vista Superior', 'FontSize', 14, 'FontWeight', 'bold');
legend('Trayectoria', 'Inicio', 'Final', 'Pared', 'Location', 'best');
axis equal;
set(gca, 'FontSize', 11);

%% === GRÁFICO 4e: FUERZA DESEADA vs EJERCIDA ===
fprintf('Generando gráfico 4e: Fuerzas deseada vs ejercida...\n');

figure('Name', 'Fuerza Deseada vs Ejercida', 'NumberTitle', 'off', 'Position', [100 100 1200 500]);

% No Perturbado
subplot(1,2,1);
plot(t_force_NP, 10*ones(size(t_force_NP)), 'k--', 'LineWidth', 2, 'DisplayName', 'F deseada'); hold on;

% Graficar fuerzas ejercidas
if size(force_NP, 2) >= 2
    plot(t_force_NP, force_NP(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'F_x ejercida');
    plot(t_force_NP, force_NP(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'F_y ejercida');
else
    plot(t_force_NP, force_NP(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'F ejercida');
end

grid on;
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Fuerza (N)', 'FontSize', 12);
title('Robot NO Perturbado', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best');
set(gca, 'FontSize', 11);

% Perturbado
subplot(1,2,2);
plot(t_force_P, 10*ones(size(t_force_P)), 'k--', 'LineWidth', 2, 'DisplayName', 'F deseada'); hold on;

% Graficar fuerzas ejercidas
if size(force_P, 2) >= 2
    plot(t_force_P, force_P(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'F_x ejercida');
    plot(t_force_P, force_P(:,2), 'r-', 'LineWidth', 1.5, 'DisplayName', 'F_y ejercida');
else
    plot(t_force_P, force_P(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'F ejercida');
end

grid on;
xlabel('Tiempo (s)', 'FontSize', 12);
ylabel('Fuerza (N)', 'FontSize', 12);
title('Robot PERTURBADO (80%)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best');
set(gca, 'FontSize', 11);

%% DEFINICIÓN DE LA PARED
% Puntos de la pared
punto1_pared = [2; 0; -1];
punto2_pared = [0; 2; -1];
altura_pared = 4;

% Crear vértices del rectángulo de la pared
vertices_pared = [
    punto1_pared(1), punto1_pared(2), punto1_pared(3);
    punto2_pared(1), punto2_pared(2), punto2_pared(3);
    punto2_pared(1), punto2_pared(2), 2;
    punto1_pared(1), punto1_pared(2), 2;
];

%% === SUBMUESTREO PARA ANIMACIÓN RÁPIDA ===
% Reducir el número de puntos para animación (cada 50 puntos)
% Ajusta este valor: mayor número = animación más rápida
factor_submuestreo = 50;  % Cambia este valor para controlar velocidad

indices_NP = 1:factor_submuestreo:N_NP;
if indices_NP(end) ~= N_NP
    indices_NP = [indices_NP, N_NP];  % Incluir último punto
end

indices_P = 1:factor_submuestreo:N_P;
if indices_P(end) ~= N_P
    indices_P = [indices_P, N_P];  % Incluir último punto
end

q_NP_animacion = q_NP(indices_NP, :);
q_P_animacion = q_P(indices_P, :);

fprintf('Reduciendo datos para animación:\n');
fprintf('  - Datos originales NP: %d puntos\n', N_NP);
fprintf('  - Datos animación NP: %d puntos (%.1f%% de los datos)\n', ...
          length(indices_NP), 100*length(indices_NP)/N_NP);
fprintf('  - Datos originales P: %d puntos\n', N_P);
fprintf('  - Datos animación P: %d puntos (%.1f%% de los datos)\n', ...
          length(indices_P), 100*length(indices_P)/N_P);

%% === GRÁFICO 4d: ANIMACIÓN DEL MOVIMIENTO ===
fprintf('Generando gráfico 4d: Animación del manipulador (mucho más rápida)...\n');

% Crear figura para animación - No Perturbado
figure('Name', 'Animación Robot NO PERTURBADO', 'NumberTitle', 'off');
hold on;

% Dibujar pared
patch('Vertices', vertices_pared, 'Faces', [1 2 3 4], ...
      'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.5, ...
      'EdgeColor', 'k', 'LineWidth', 1);

%plot3([2, 0], [0, 2], [0, 0], 'k-', 'LineWidth', 0);
text(2, 0, 0, ' P1(2,0)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');
text(0, 2, 0, ' P2(0,2)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');

% Animar robot nominal con datos submuestreados
rob.plot(q_NP_animacion, 'trail', 'b-', 'workspace', [-1 3 -1 3 -1 2]);
title('Robot NO PERTURBADO', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

fprintf('Animación 1/2 completada.\n');

% Crear figura para animación - Perturbado
figure('Name', 'Animación Robot PERTURBADO', 'NumberTitle', 'off');
hold on;

% Dibujar pared
patch('Vertices', vertices_pared, 'Faces', [1 2 3 4], ...
      'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.5, ...
      'EdgeColor', 'k', 'LineWidth', 1);

%plot3([2, 0], [0, 2], [0, 0], 'k-', 'LineWidth', 0);
text(2, 0, 0, ' P1(2,0)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');
text(0, 2, 0, ' P2(0,2)', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');

% Animar robot perturbado con datos submuestreados
rob2.plot(q_P_animacion, 'trail', 'r-', 'workspace', [-1 3 -1 3 -1 2]);
title('Robot PERTURBADO (80%)', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

fprintf('Animación 2/2 completada.\n');

%% RESUMEN
fprintf('\n=== ✓ GRÁFICOS COMPLETADOS ===\n');
fprintf('Se generaron los siguientes gráficos:\n');
fprintf('  • 4a: Ángulos de joints vs tiempo\n');
fprintf('  • 4b: Posición cartesiana vs tiempo\n');
fprintf('  • 4c: Vista aérea (plano XY)\n');
fprintf('  • 4e: Fuerza deseada vs ejercida\n');
fprintf('  • 4d: Animación del manipulador (ACELERADA)\n');
fprintf('\nCada gráfico muestra comparación entre robot nominal y perturbado.\n');
fprintf('\nNOTA: La animación usa submuestreo (factor=%d) para mayor velocidad.\n', factor_submuestreo);
fprintf('      Ajusta "factor_submuestreo" para controlar la velocidad.\n');