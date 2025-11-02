% Definicion de Links
L(1) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0); %Define el primer brazo como R
L(2) = Link('revolute', 'd', 0, 'a', 1, 'alpha', 0); %d y a son los parámetros dh por eso es d= 0 y a=1 (L1=L2=1)

L(1).m = 1;                     % Masa (1 kg)
L(1).r = [L(1).a, 0, 0];        % Centro de masa en la punta (x=a)
L(1).I = zeros(3,3);            % Inercia de una masa puntual es CERO

L(2).m = 1;
L(2).r = [L(2).a, 0, 0];        % Centro de masa en la punta (x=a)
L(2).I = zeros(3,3);            % Inercia de una masa puntual es CERO

L(1).Tc = 1;                    %La fricción de Coulomb es 1 por consigna
L(2).Tc = 1;                    %La fricción de Coulomb es 1 por consigna

g = [0 0 -10]; 

global rob
%Crea el objeto robot final, un SerialLink, utilizando la lista de eslabones $L$, le da un nombre y le asigna el vector de gravedad g
rob = SerialLink(L, 'name', 'Mi Robot 2R', 'gravity', g); 


T1 = transl(1, -1, 0) * trotz(-pi/2)  % Posición cartesiana inicial 
T2 = transl(1, 1, 0) * trotz(pi/2) % Posición cartesiana final
t = [0:0.02:5]';                  % Tiempo de trayectoria propuesto

[s, sd, sdd] = tpoly(0, 1, t, 0, 0);

s(s < 0) = 0;
s(s > 1) = 1;

Tcart = ctraj(T1, T2, s);
%ctraj (Cartesian Trajectory) genera una trayectoria de poses (matrices de transformación) que va de $T_1$ a $T_2$.
%Utiliza el vector de escala $s$ para interpolar linealmente las posiciones (traslación) y rotacionalmente (usando el logaritmo de la matriz de rotación) las orientaciones.
%Tcart es un arreglo 3D de 4 x 4 x N, donde N es la longitud de t.

X = squeeze(Tcart(1, 4, :));
Y = squeeze(Tcart(2, 4, :));
%Extrae los componentes de posición del arreglo de poses Tcart. squeeze elimina las dimensiones unitarias, dejando $X$ y $Y$ como vectores columna.

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

figure;
subplot(3,1,1);
plot(t, [X, Y]);
title('Posición Polinómica (X, Y)');
legend('X', 'Y');

subplot(3,1,2);
plot(t, [X_d, Y_d]);
title('Velocidad Polinómica (X_d, Y_d)');
legend('X_d', 'Y_d');

subplot(3,1,3);
plot(t, [X_dd, Y_dd]);
title('Aceleración Polinómica (X_dd, Y_dd)');
legend('X_dd', 'Y_dd');
xlabel('Tiempo (s)');






% --- 1. CÁLCULO DE ÁNGULOS DE ARTICULACIÓN DESEADOS (q_deseado) ---
% Se usa Cinemática Inversa (ikine) para obtener la referencia q_d
q_nominal = [0, 0]; % Conjetura inicial de ángulos (q1=0, q2=0)
N = length(t); 
qd_deseado = zeros(N, rob.n); % rob.n = 2 DOF

% Definición de la Máscara para un Robot 2R plano:
% [X, Y, Z, Roll, Pitch, Yaw]. Solo nos importan X e Y.
mascara = [1 1 0 0 0 0]; 

for i = 1:N
    T_i = Tcart(:, :, i);
    
    % LÍNEA CORREGIDA: Se incluye la máscara
    q_i = rob.ikine(T_i, 'q0', q_nominal, 'ru', 'mask', mascara); 
    
    if ~isempty(q_i) 
        qd_deseado(i, :) = q_i;
        q_nominal = q_i;
    else
        % Manejo de error si ikine no encuentra solución
        disp(['Advertencia: ikine no encontró solución en el paso ', num2str(i)]);
        qd_deseado(i, :) = qd_deseado(max(1, i-1), :); 
    end
end

% --- 2. GENERACIÓN DEL GRÁFICO 4.a ---

% !!! ADVERTENCIA: Se ASUME que la simulación de Simulink ya se ejecutó !!!
% La salida de posición de articulación real debe estar guardada en el Workspace.
% Por ejemplo, si usas un bloque 'To Workspace' llamado 'Q_out':
% Q_real = Q_out.Data; 

Q_real = out.Q_out; 

% 2. Verificar longitudes (opcional, pero útil para depurar)
if size(Q_real, 1) ~= length(t)
    disp('Advertencia: El número de pasos en la simulación no coincide con el vector de tiempo "t".');
    % Si los datos de Simulink incluyen el tiempo, podrías usar: t_real = Q_out.Time;
end

figure;
plot(t, qd_deseado(:, 1), 'b--', t, Q_real(:, 1), 'b', ...
     t, qd_deseado(:, 2), 'r--', t, Q_real(:, 2), 'r');
title('4a. Ángulos de Articulación (Joints): Deseado vs. Real');
xlabel('Tiempo (s)');
ylabel('Ángulo (rad)');
legend('q1 Deseado', 'q1 Real', 'q2 Deseado', 'q2 Real', 'Location', 'best');
grid on;



if isstruct(out.Q_out)
    q_data = out.Q_out.Data;  % [N x 2]
else
    q_data = out.Q_out;
end

% ===== CONFIGURACIÓN DE LA PARED =====
% Definir los puntos de la pared
punto1_pared = [2; 0; 0];  % Primer punto de la pared
punto2_pared = [0; 2; 0];  % Segundo punto de la pared

% Graficar y animar con la pared
figure;

% PRIMERO: Crear la figura y configurar hold on
ax = gca;
hold on;

% SEGUNDO: Dibujar la pared ANTES de la animación
% Definir altura de la pared (en Z)
altura_pared = 1.5;

% Crear vértices del rectángulo de la pared
vertices_pared = [
    punto1_pared(1), punto1_pared(2), 0;              % Punto 1 abajo
    punto2_pared(1), punto2_pared(2), 0;              % Punto 2 abajo
    punto2_pared(1), punto2_pared(2), altura_pared;   % Punto 2 arriba
    punto1_pared(1), punto1_pared(2), altura_pared    % Punto 1 arriba
];

% Dibujar el polígono de la pared usando patch
patch('Vertices', vertices_pared, 'Faces', [1 2 3 4], ...
      'FaceColor', [0.8 0.8 0.8], ...  % Color gris claro
      'FaceAlpha', 0.5, ...             % Transparencia
      'EdgeColor', 'k', ...             % Borde negro
      'LineWidth', 2);

% Línea en el plano XY para mayor claridad
plot3([punto1_pared(1), punto2_pared(1)], ...
      [punto1_pared(2), punto2_pared(2)], ...
      [0, 0], 'k-', 'LineWidth', 3);

% Etiquetas para los puntos de la pared
text(punto1_pared(1), punto1_pared(2), 0, ' P1(2,0)', ...
     'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');
text(punto2_pared(1), punto2_pared(2), 0, ' P2(0,2)', ...
     'FontSize', 10, 'FontWeight', 'bold', 'Color', 'blue');

% TERCERO: Ahora sí, animar el robot
rob.plot(q_data, 'trail', 'r-', 'workspace', [-1 3 -1 3 -1 2]);

title('Animación del Robot 2R con Pared');
grid on;
hold off;