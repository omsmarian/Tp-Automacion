function G = gravload_wrapper(q)
% Wrapper para rob.gravload(q)
    persistent rob_local
    if isempty(rob_local)
        % Si no existe aún, recupera el robot del workspace base
        rob_local = evalin('base', 'rob');
    end
    G = rob_local.gravload(q);
end
