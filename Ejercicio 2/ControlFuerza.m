%% DEFINICIÓN DEL ROBOT Y CONSTANTES GLOBALES

% Limpiar Workspace y figuras
%clear all; close all; clc;
global rob fd Kenv n kpf kvf d_pared
n = [1/sqrt(2); 1/sqrt(2)];

% --- A. DEFINICIÓN DE LINKS Y ROBOT SERIALLINK ---
L(1) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0); 
L(2) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0); 

% Parámetros Dinámicos (m=1kg, fricción de Coulomb=1, r=[a, 0, 0])
L(1).m = 1; L(1).r = [L(1).a, 0, 0]; L(1).I = zeros(3,3); L(1).Tc = 1;
L(2).m = 1; L(2).r = [L(2).a, 0, 0]; L(2).I = zeros(3,3); L(2).Tc = 1;

g = [0 0 -10];
rob = SerialLink(L, 'name', 'Rob_fuerza', 'gravity', g);


%% CONDICIONES INICIALES

t1 = pi - (asin((2-sqrt(2))/sqrt(2))) - pi/4
t2 = pi/4 - t1


% Posición articular inicial (radianes)
q_inicial = [t1; t2];

% Velocidades articulares iniciales (en reposo)
qd_inicial = [0; 0];

% Aceleraciones articulares iniciales
qdd_inicial = [0; 0];

% Verificar la posición cartesiana inicial del efector
T_inicial = rob.fkine(q_inicial);
pos_inicial = T_inicial.t;

fprintf('=== CONDICIONES INICIALES ===\n');
fprintf('Posición articular inicial (rad):\n');
fprintf('  q1 = %.4f rad (%.2f°)\n', q_inicial(1), rad2deg(q_inicial(1)));
fprintf('  q2 = %.4f rad (%.2f°)\n', q_inicial(2), rad2deg(q_inicial(2)));
fprintf('\nPosición cartesiana inicial del efector:\n');
fprintf('  x = %.4f m\n', pos_inicial(1));
fprintf('  y = %.4f m\n', pos_inicial(2));
fprintf('  z = %.4f m\n', pos_inicial(3));

%% TRAYECTORIA DE ACERCAMIENTO SEGURA

t = [0:0.001:5]'; % Tiempo de simulación propuesto
if isrow(t); t = t'; end
N = length(t);

num_columnas = length(t);

% Fuerza deseada constante en el tiempo
fila_10 = (10 * ones(1, num_columnas))'; % [10, 10, 10, ...]
fila_0 = (zeros(1, num_columnas))';      % [0, 0, 0, ...]

Fd = [t, fila_10, fila_0];

%% VERIFICACIÓN FINAL
fprintf('\n=== LISTO PARA SIMULINK ===\n');
fprintf('Ejecuta el modelo de Simulink ahora.\n');


if isstruct(out.Q_out_f)
    q_data = out.Q_out_f.Data;  % [N x 2]
else
    q_data = out.Q_out_f;
end

% Definir los puntos de la pared
punto1_pared = [2; 0; 0];
punto2_pared = [0; 2; 0];
altura_pared = 1.5;

% Crear vértices del rectángulo de la pared
vertices_pared = [
    punto1_pared(1), punto1_pared(2), 0;
    punto2_pared(1), punto2_pared(2), 0;
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

plot3([punto1_pared(1), punto2_pared(1)], ...
      [punto1_pared(2), punto2_pared(2)], ...
      [0, 0], 'k-', 'LineWidth', 3);

text(punto1_pared(1), punto1_pared(2), 0, ' P1(2,0)', ...
     'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');
text(punto2_pared(1), punto2_pared(2), 0, ' P2(0,2)', ...
     'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');

% Animar robot nominal
%rob.plot(, 'trail', 'b-', 'workspace', [-1 3 -1 3 -1 2]);
%title('Robot NOMINAL (Sin Perturbaciones)');
%grid on;
%hold off;
