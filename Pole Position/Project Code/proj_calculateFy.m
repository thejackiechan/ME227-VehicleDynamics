function Fy = calculateFy(alpha_sl, C_alpha, alpha, u, u_s, Fz) 

if abs(alpha)< alpha_sl 
    Fy = -C_alpha*tand(alpha)+ (((C_alpha^2)/(3*u*Fz))*(2-(u_s/u))*abs(tand(alpha))*tand(alpha))-(((C_alpha^3)/(9*u^2*Fz^2))*tand(alpha)^3*(1-(2*u_s/(3*u))));
elseif abs(alpha)>= alpha_sl
    Fy = -u_s*Fz*sign(alpha);
end

end

