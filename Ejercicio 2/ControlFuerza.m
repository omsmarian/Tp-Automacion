%% SETUP - DEFINICIÓN DEL ROBOT Y CONDICIONES INICIALES
% Archivo: setup_control_fuerza.m
% Ejecutar ANTES de correr el modelo Simulink

%% LIMPIAR WORKSPACE
clear all; close all; clc;

%% VARIABLES GLOBALES
global rob fd Kenv n kpf kvf d_pared rob2

% Vector normal a la pared
n = [1/sqrt(2); 1/sqrt(2)];

%% DEFINICIÓN DEL ROBOT NOMINAL
fprintf('=== CONFIGURACIÓN DEL ROBOT ===\n');

% Definición de Links
L(1) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0); 
L(2) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0); 

% Parámetros Dinámicos (m=1kg, fricción de Coulomb=1, r=[a, 0, 0])
L(1).m = 1; 
L(1).r = [L(1).a, 0, 0]; 
L(1).I = zeros(3,3); 
L(1).Tc = 1;

L(2).m = 1; 
L(2).r = [L(2).a, 0, 0]; 
L(2).I = zeros(3,3); 
L(2).Tc = 1;

% Gravedad
g = [0 0 -10];

% Robot nominal
rob = SerialLink(L, 'name', 'Rob_fuerza', 'gravity', g);
fprintf('Robot nominal creado.\n');

% Robot perturbado (80%%)
rob2 = rob.perturb(0.8);
fprintf('Robot perturbado creado (80%% de perturbación).\n');

%% CONDICIONES INICIALES
fprintf('\n=== CONDICIONES INICIALES ===\n');

% Cálculo de ángulos iniciales
t1 = pi - (asin((2-sqrt(2))/sqrt(2))) - pi/4;
t2 = pi/4 - t1;

% Posición articular inicial (radianes)
q_inicial = [t1; t2];

% Velocidades articulares iniciales (en reposo)
qd_inicial = [0; 0];

% Aceleraciones articulares iniciales
qdd_inicial = [0; 0];

% Verificar la posición cartesiana inicial del efector
T_inicial = rob.fkine(q_inicial);
pos_inicial = T_inicial.t;

fprintf('Posición articular inicial (rad):\n');
fprintf('  q1 = %.4f rad (%.2f°)\n', q_inicial(1), rad2deg(q_inicial(1)));
fprintf('  q2 = %.4f rad (%.2f°)\n', q_inicial(2), rad2deg(q_inicial(2)));
fprintf('\nPosición cartesiana inicial del efector:\n');
fprintf('  x = %.4f m\n', pos_inicial(1));
fprintf('  y = %.4f m\n', pos_inicial(2));
fprintf('  z = %.4f m\n', pos_inicial(3));

%% TRAYECTORIA Y FUERZA DESEADA
fprintf('\n=== CONFIGURACIÓN DE SIMULACIÓN ===\n');

% Tiempo de simulación
t = [0:0.001:3]'; 
if isrow(t); t = t'; end
N = length(t);
num_columnas = length(t);

fprintf('Tiempo de simulación: 0 a 5 segundos\n');
fprintf('Número de muestras: %d\n', N);

% Fuerza deseada constante en el tiempo
fila_10 = (10 * ones(1, num_columnas))'; % Fuerza de 10N
fila_0 = (zeros(1, num_columnas))';      % Sin componente en otra dirección

Fd = [t, fila_10, fila_0];

fprintf('Fuerza deseada: 10N constante\n');

%% DEFINICIÓN DE LA PARED
% Puntos de la pared
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

fprintf('\n=== PARED CONFIGURADA ===\n');
fprintf('Punto 1: (%.1f, %.1f, %.1f)\n', punto1_pared);
fprintf('Punto 2: (%.1f, %.1f, %.1f)\n', punto2_pared);

%% LISTO PARA SIMULINK
fprintf('\n=== ✓ SETUP COMPLETO ===\n');
fprintf('Ejecuta el modelo de Simulink ahora.\n');
fprintf('Después ejecuta: graficar_resultados\n');