
%% --------------Control Hibrido----------------------------

clear all;
close all;
clc;


%Variables Control de Fuerza:
global rob fd Kenv n kpf kvf d_pared rob2
n = [1/sqrt(2); 1/sqrt(2)];



% ========== DEFINICIÓN DEL ROBOT ==========
% Definicion de Links
L(1) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0);
L(2) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0);

L(1).m = 1;                     % Masa (1 kg)
L(1).r = [L(1).a, 0, 0];        % Centro de masa en la punta (x=a)
L(1).I = zeros(3,3);            % Inercia de una masa puntual es CERO

L(2).m = 1;
L(2).r = [L(2).a, 0, 0];        % Centro de masa en la punta (x=a)
L(2).I = zeros(3,3);            % Inercia de una masa puntual es CERO

L(1).Tc = 1;                    % Fricción de Coulomb
L(2).Tc = 1;                    % Fricción de Coulomb

g = [0 0 -10]; 




% Crear robot nominal y perturbado
rob = SerialLink(L, 'name', '2R Robot', 'gravity', g); 
rob2 = rob.perturb(0.8);  % 80% de perturbación

%% ========== DEFINICIÓN DE TRAYECTORIA ==========
T1 = transl(1.5, 0.5, 0) * trotz(-pi/2);  % Posición cartesiana inicial 
T2 = transl(0.5, 1.5, 0) * trotz(pi/2);    % Posición cartesiana final
t = [0:0.001:5]';                       % Tiempo de trayectoria propuesto

[s, sd, sdd] = tpoly(0, 1, t, 0, 0);

s(s < 0) = 0;
s(s > 1) = 1;

Tcart = ctraj(T1, T2, s);

X = squeeze(Tcart(1, 4, :));
Y = squeeze(Tcart(2, 4, :));

P1_vec = T1(1:3, 4); % [1; -1; 0]
P2_vec = T2(1:3, 4); % [1; 1; 0]
P_delta = P2_vec - P1_vec; % [0; 2; 0]

Vel_cart = P_delta * sd'; % Resultado es [3xN]
Acc_cart = P_delta * sdd'; % Resultado es [3xN]

X_d = Vel_cart(1, :)';
Y_d = Vel_cart(2, :)';

X_dd = Acc_cart(1, :)';
Y_dd = Acc_cart(2, :)';

if isrow(t)
    t = t';
end

Posicion_Cartesiana = [t, X, Y];
Velocidad_Cartesiana = [t, X_d, Y_d];
Aceleracion_Cartesiana = [t, X_dd, Y_dd];

%% ========== DEFINICIÓN DE FUERZAS ==========

% Punto objetivo
p = [1.5, 0.5];

q_nominal = [0, 0]; % Conjetura inicial de ángulos (q1=0, q2=0)

% Calcular la cinemática inversa (solo posición)
T = transl(p(1), p(2), 0);

mascara = [1 1 0 0 0 0];

% Resolver para q1 y q2
q_inicial = rob.ikine(T, 'q0', q_nominal, 'ru', 'mask', mascara);

% Mostrar resultado
disp('Ángulos (rad):');
disp(q_inicial);

num_columnas = length(t);

% Fuerza deseada constante en el tiempo
fila_10 = (10 * ones(1, num_columnas))'; % Fuerza de 10N
fila_0 = (zeros(1, num_columnas))';      % Sin componente en otra dirección

Fd = [t, fila_10, fila_0];

fprintf('Fuerza deseada: 10N constante\n');
