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

% Posición articular inicial (radianes)
q_inicial = [1.93; -1.143];

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

% Calcular distancia a la pared
d_inicial = d_pared - (n(1)*pos_inicial(1) + n(2)*pos_inicial(2));
fprintf('\nDistancia inicial a la pared: %.4f m\n', d_inicial);

if d_inicial < 0
    fprintf('  ⚠️  El robot YA está en contacto (penetración: %.4f m)\n', -d_inicial);
elseif d_inicial < 0.1
    fprintf('  ⚠️  El robot está MUY cerca de la pared\n');
else
    fprintf('  ✓ El robot está antes de la pared\n');
end

%% TRAYECTORIA DE ACERCAMIENTO SEGURA

t = [0:0.02:15]'; % Tiempo de simulación propuesto
if isrow(t); t = t'; end
N = length(t);

% Puntos Cartesianos para la Trayectoria
P_start = pos_inicial(1:2); % Partir desde la posición inicial real
P_contacto = [1; 1];
P_penetracion = P_contacto + 0.5 * n(1:2); % Intenta penetrar 0.5m más allá

% Matrices de Transformación
T1 = transl([P_start; 0]') * trotz(0); 
T2 = transl([P_penetracion; 0]') * trotz(pi/4); 

% Generación de la Trayectoria (incluye X, Xd, Xdd)
[s, sd, sdd] = tpoly(0, 1, t, 0, 0); % Escalado polinómico
Tcart = ctraj(T1, T2, s); 

% Extracción de Posición, Velocidad y Aceleración Cartesianas
X = squeeze(Tcart(1, 4, :));
Y = squeeze(Tcart(2, 4, :));

P_delta = P_penetracion - P_start; % Solo X e Y
Vel_cart = P_delta * sd';
Acc_cart = P_delta * sdd';

% Formato para el bloque From Workspace de Simulink ([Tiempo, X, Y])
Posicion_Cartesiana = [t, X, Y];
Velocidad_Cartesiana = [t, Vel_cart(1,:)', Vel_cart(2,:)']; 
Aceleracion_Cartesiana = [t, Acc_cart(1,:)', Acc_cart(2,:)']; 

X_d = Vel_cart(1, :)';
Y_d = Vel_cart(2, :)';

X_dd = Acc_cart(1, :)';
Y_dd = Acc_cart(2, :)';

num_columnas = length(t);

% Fuerza deseada constante en el tiempo
fila_10 = (10 * ones(1, num_columnas))'; % [10, 10, 10, ...]
fila_0 = (zeros(1, num_columnas))';      % [0, 0, 0, ...]

Fd = [t, fila_10, fila_0];

%% VERIFICACIÓN FINAL

fprintf('\n=== VERIFICACIÓN DE DIMENSIONES ===\n');
fprintf('Posicion_Cartesiana: [%d x %d]\n', size(Posicion_Cartesiana));
fprintf('Velocidad_Cartesiana: [%d x %d]\n', size(Velocidad_Cartesiana));
fprintf('Aceleracion_Cartesiana: [%d x %d]\n', size(Aceleracion_Cartesiana));
fprintf('Fd: [%d x %d]\n', size(Fd));
fprintf('q_inicial: [%d x %d]\n', size(q_inicial));
fprintf('qd_inicial: [%d x %d]\n', size(qd_inicial));
fprintf('qdd_inicial: [%d x %d]\n', size(qdd_inicial));

fprintf('\n=== LISTO PARA SIMULINK ===\n');
fprintf('Ejecuta el modelo de Simulink ahora.\n');


if isstruct(out.Q_out_f)
    q_data = out.Q_out_f.Data;  % [N x 2]
else
    q_data = out.Q_out_f;
end

% Graficar y animar
figure;
rob.plot(q_data, 'trail', 'r-');