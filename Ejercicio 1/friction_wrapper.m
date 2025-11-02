function F = friction_wrapper(qp)
% Wrapper para rob.friction(qp)
    persistent rob_local
    if isempty(rob_local)
        rob_local = evalin('base', 'rob');
    end
    F = rob_local.friction(qp);
end
