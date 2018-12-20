function [Fy] = coupled(C,slip_angle,Fz,mu,Fx)

% Used to calculate Fy for the coupled tire model

zeta = sqrt(mu^2 * Fz^2 - Fx^2)/mu/Fz;
slip_limit = atan(3*zeta*mu*Fz/C);

if(abs(slip_angle) <= slip_limit)
    Fy = -C*tan(slip_angle) + C^2*abs(tan(slip_angle))*tan(slip_angle)/3/zeta/mu/Fz...
        - C^3*(tan(slip_angle))^3/27*zeta^-2*mu^-2*Fz^-2;
elseif(abs(slip_angle) > slip_limit)
    Fy = -zeta * mu * Fz * sign(slip_angle);
end
    
end