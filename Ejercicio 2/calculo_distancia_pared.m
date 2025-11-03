function [d, delta] = calculo_distancia_pared(pos)
    % CALCULO_DISTANCIA_PARED Calcula la distancia del efector final a la pared
    %
    % La pared está definida por: x + y = 2
    % Forma normal: (1/sqrt(2))*x + (1/sqrt(2))*y = 2/sqrt(2) = sqrt(2)
    %
    % Inputs:
    %   pos - Posición del efector final [x; y] o [x, y]
    %
    % Outputs:
    %   d     - Distancia perpendicular a la pared (negativa = penetración)
    %   delta - Magnitud de penetración (0 si no hay contacto, |d| si hay)
    %
    % Ecuación de distancia punto-recta:
    %   d = (x + y - 2) / sqrt(2)
    %   d = n' * [x; y] - d_pared
    %   donde n = [1/sqrt(2); 1/sqrt(2)] y d_pared = sqrt(2)
    
    % Extraer coordenadas
    x = pos(1);
    y = pos(2);
    
    % Vector normal a la pared (unitario)
    n = [1/sqrt(2); 1/sqrt(2)];
    
    % Distancia del origen a la pared
    d_pared = 2 / sqrt(2); % = sqrt(2)
    
    % Calcular distancia perpendicular (signada)
    % d > 0: efector está antes de la pared (no hay contacto)
    % d = 0: efector está exactamente en la pared
    % d < 0: efector penetró la pared
    d = d_pared - (n(1)*x + n(2)*y);
    
    % Calcular delta (magnitud de penetración)
    % delta = 0 si d >= 0 (no hay contacto)
    % delta = |d| si d < 0 (hay penetración)
    if d >= 0
        delta = 0;  % No hay contacto
    else
        delta = -d; % Penetración (valor positivo)
    end
    
    % Nota: Algunos textos usan la convención opuesta donde:
    % d < 0 significa "antes de la pared"
    % Si tu libro usa esa convención, invierte el signo:
    % d = (n(1)*x + n(2)*y) - d_pared;
end