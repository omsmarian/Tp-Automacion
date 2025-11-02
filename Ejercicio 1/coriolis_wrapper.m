function C = coriolis_wrapper(q, qp)
% Wrapper para rob.coriolis(q, qp)
    persistent rob_local
    if isempty(rob_local)
        rob_local = evalin('base', 'rob');
    end
    C = rob_local.coriolis(q, qp);
end
