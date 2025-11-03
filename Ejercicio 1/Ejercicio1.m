%% setup_robot.m
% Este script configura el robot y calcula las trayectorias
% Ejecutar ANTES de correr la simulación de Simulink

clear all;
close all;
clc;

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

global rob
global rob2

% Crear robot nominal y perturbado
rob = SerialLink(L, 'name', '2R Robot', 'gravity', g); 
rob2 = rob.perturb(0.8);  % 80% de perturbación

% ========== DEFINICIÓN DE TRAYECTORIA ==========
T1 = transl(1, -1, 0) * trotz(-pi/2);  % Posición cartesiana inicial 
T2 = transl(1, 1, 0) * trotz(pi/2);    % Posición cartesiana final
t = [0:0.02:5]';                       % Tiempo de trayectoria propuesto

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

% ========== CINEMÁTICA INVERSA ==========
q_nominal = [0, 0]; % Conjetura inicial de ángulos (q1=0, q2=0)
N = length(t); 
qd_deseado = zeros(N, rob.n); % rob.n = 2 DOF

% Definición de la Máscara para un Robot 2R plano:
% [X, Y, Z, Roll, Pitch, Yaw]. Solo nos importan X e Y.
mascara = [1 1 0 0 0 0]; 

for i = 1:N
    T_i = Tcart(:, :, i);
    
    q_i = rob.ikine(T_i, 'q0', q_nominal, 'ru', 'mask', mascara);
    
    if ~isempty(q_i) 
        qd_deseado(i, :) = q_i;
        q_nominal = q_i;
    else
        disp(['Advertencia: ikine no encontró solución en el paso ', num2str(i)]);
        qd_deseado(i, :) = qd_deseado(max(1, i-1), :); 
    end
end

disp('=================================================');
disp('Setup completado. Variables disponibles:');
disp('  - rob, rob2: Robots nominal y perturbado');
disp('  - t: Vector de tiempo');
disp('  - qd_deseado: Ángulos deseados [N x 2]');
disp('  - Posicion_Cartesiana, Velocidad_Cartesiana, Aceleracion_Cartesiana');
disp('=================================================');
disp('Ahora puedes ejecutar la simulación de Simulink.');