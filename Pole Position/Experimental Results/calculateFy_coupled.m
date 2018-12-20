function Fy = calculateFy_coupled(alpha_sl, C_alpha, alpha, u, u_s, Fz, Fx) 
gamma = sqrt((u*Fz)^2-Fx)/(u*Fz);
if abs(alpha)<= alpha_sl 
    Fy = -C_alpha*tand(alpha)+ (((C_alpha^2)/(3*gamma*u*Fz))*abs(tand(alpha))*tand(alpha))-(((C_alpha^3)/(27*u^2*Fz^2*gamma^2))*tand(alpha)^3);
elseif abs(alpha)> alpha_sl
    Fy = -gamma*u_s*Fz*sign(alpha);
end

end