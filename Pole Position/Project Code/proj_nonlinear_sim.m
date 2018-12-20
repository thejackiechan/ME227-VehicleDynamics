function [ t_s, ux_mps, uy_mps, r_radps, s_m, e_m, delta_psi_rad, Fx_total, delta_steer ] = proj_nonlinear_sim(C_alphaf, C_alphar, u_f,...
    u_fs, u_r, u_rs, m,Iz, L, s, k, percent_front, percent_rear,...
    Kla, xla, k_vel, e0, ux0, vx_prof, ax_prof, k_accel, f_rr, C_DA)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%  proj_nonlinear_sim(C_alphaf, C_alphar, u_f, u_fs, u_r, u_rs, m,Iz, L, s,
% ...k, Ux_des, percent_front, percent_rear, Kla, xla, k_vel, e0, ux0);
%Wf = mgb/L, Wr = mga/L -> find a and b:
g = 9.8;
a = percent_rear*L;
b = percent_front*L;
% integrate s and k with given initial conditions [psi0, E0, N0] to get path
path = generate_path(s,k,[0;0;0]);
Fz_front = m*g*percent_front;
Fz_rear = m*g*percent_rear;
alpha_sl_front = abs(atand(3*u_f*Fz_front/C_alphaf));
alpha_sl_rear = abs(atand(3*u_r*Fz_rear/C_alphar));


% simulation time
dT = 0.005;
t_final = 10;
t_s = 0:dT:t_final;
N = length(t_s);


% allocate space for simulation data
ux_mps = zeros(N,1);
uy_mps = zeros(N,1);
r_radps = zeros(N,1);
s_m = zeros(N,1);
e_m = zeros(N,1);
delta_psi_rad = zeros(N,1);
Fx_total = zeros(N,1);
delta_steer = zeros(N,1);
ux_dot_mps = zeros(N,1);

% set initial conditions
ux_mps(1) = ux0;
uy_mps(1) = 0;
r_radps(1)  = 0;
s_m(1) = 0;
e_m(1) = e0;
delta_psi_rad(1) = 0;
ux_dot_mps(1) = 0;





%Nonlinear model


% simulation loop
for idx = 1:N
    % look up K
    k = interp1(path.s_m, path.k_1pm, s_m(idx));
    
    
    % current states
    ux = ux_mps(idx);
    if(ux < 0.5) % magic minimum speed
        ux = 0.5;
    end

    uy = uy_mps(idx);
    r = r_radps(idx);
    %s = s_m(idx);
    e = e_m(idx);
    delta_psi = delta_psi_rad(idx);
    ux_dot = ux_dot_mps(idx);
    
    %best fit from Problem 1
    C_alphar_linear = 203000;
    C_alphaf_linear = 188000;
    delta_psi_ss = k*((m*a*ux^2/(L*(C_alphar_linear)))-b); 
    p = (Kla*xla*delta_psi_ss/C_alphaf_linear);
    q = k*(L - ux^2*(m*(a*C_alphaf_linear-b*C_alphar_linear)/(L*C_alphaf_linear*C_alphar_linear)));
    delta_ff = p + q;
    %% Steer control
    delta_steer(idx) = (-Kla*(e+xla*delta_psi)/C_alphaf_linear)+ delta_ff;
    alpha_f = rad2deg(atan(((uy + a*r)/ux)) - delta_steer(idx));
    alpha_r = rad2deg(atan((uy - b*r)/ux));
    F_yf = proj_calculateFy(alpha_sl_front, C_alphaf, alpha_f, u_f, u_fs, Fz_front);
    F_yr = proj_calculateFy(alpha_sl_rear, C_alphar, alpha_r, u_r, u_rs, Fz_rear);
    
    % drag force
    air_density = 1.225;
    F_d = 0.5 * air_density * ux^2 * C_DA;
    accel_drag = F_d/m;
    
    % rolling resistance 
    F_rr = f_rr * m * g;
    a_rr = F_rr/m;
    
    %control input
    Fg = 0; %not given grade
    %% Longitudinal Speed (throttle) control
    Fx_total(idx) = k_vel*(vx_prof(idx) - ux)  - F_d - F_rr + Fg + m*ax_prof(idx);
    F_xf = Fx_total(idx) * 0.6;
    F_xr = Fx_total(idx) * 0.4;
    
    % equations of motion
    ux_dot = ((F_xr + F_xf*cos(delta_steer(idx)) - F_yf*sin(delta_steer(idx)))/m) + ...
        r*uy - accel_drag - a_rr;
    uy_dot = ((F_yf*cos(delta_steer(idx)) + F_yr +F_xf*sin(delta_steer(idx)))/m) - r*ux;
    r_dot = (a*F_yf*cos(delta_steer(idx)) + a*F_xf*sin(delta_steer(idx)) -  b*F_yr)/Iz;
    
    s_dot = (1/(1-e*k))*(ux*cos(delta_psi) - uy*sin(delta_psi));
    e_dot = uy*cos(delta_psi) + ux*sin(delta_psi);
    delta_psi_dot = r - k*s_dot;
    
   

    % only update next state if we are not at end of simulation
    if idx < N
        % euler integration
        ux_mps(idx+1) = ux_mps(idx) + ux_dot*dT;
        uy_mps(idx+1) = uy_mps(idx) + uy_dot*dT;
        r_radps(idx+1) = r_radps(idx) + r_dot*dT;
        s_m(idx+1) = s_m(idx) + s_dot*dT;
        e_m(idx+1) = e_m(idx) + e_dot*dT;
        delta_psi_rad(idx+1) = delta_psi_rad(idx) + delta_psi_dot*dT;
    end

end

