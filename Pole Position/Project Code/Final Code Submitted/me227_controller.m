function [ delta_rad, Fx_N ] = me227_controller( s_m, e_m, deltaPsi_rad, Ux_mps, Uy_mps, r_radps, modeSelector, path )
% me227_controller( s_m, e_m, deltaPsi_rad, Ux_mps, Uy_mps, r_radps, modeSelector, path )
%   Include your names in the comments at the top of the function.  The
%   state and controls follow the notation in class and have the units
%   appended.  The parameter path matches what you have in simulation i.e.
%   it has the following fields s_m, k_1pm, psi_rad, posE_m, and posN_m.

%#codegen % this directive is needed to run your code on the car

%% Controller Constants
% External Forces
g = 9.8;
f_rr = 0.0157;      % rolling resistance
C_DA = 0.594;       % drag coefficient
air_density = 1.225;

% Car Dimensions
m = 1659; % kg 
Iz = 2447; % kg m2
L = 2.468; % m
percent_front = .577; % weight
percent_rear = .423;
a                   = percent_rear*L;
b                   = percent_front*L;

% Tire Properties
C_alphaf = 275000; %N/rad
C_alphar = 265000; %N/rad
u_f = 0.97;
u_fs = 0.97;
u_r = 1.03;
u_rs = 1.03;

% Control Properties for Delta
C_alphar_linear = 203000;
C_alphaf_linear = 188000;
kla             = 7000;
xla             = 25;

k_understeer    = (m*(b*C_alphar_linear-a*C_alphaf_linear)/(L*C_alphaf_linear*C_alphar_linear));

% Control Gains for Fx_N
Fx_desired = 0.3*m*g; 
ux_diff_des = 1;
k_vel = Fx_desired/ux_diff_des; % k_vel (!)
%% K acceleration 

% Kinematic Labels
ux          = Ux_mps;
uy          = Uy_mps;
r           = r_radps;
delta_psi   = deltaPsi_rad;
e           = e_m;
dT          = 0.005;

% Persistent Variable: path index s_idx
persistent e_sum
if isempty(e_sum)
    e_sum = 0;
end
e_sum = e_sum + e*dT;

persistent e_dot
if isempty(e_dot)
    e_dot = 0;
end

persistent last_k
if isempty(last_k)
    last_k = 0;
end

persistent last_e
if isempty(last_e)
    last_e = 0;
end

% Get vehicle velocity and acceleration profiles tracking the path
%[vx_prof, ax_prof] = tracking_controller_test(path);
%[vx_prof, ax_prof] = tracking(path); % Path for tracking oval

if modeSelector == 1 % Control of delta w/ ff, Fx_N with PD
    %% 1 Feed Forward Steer Control
    
    % 1a get curvature  of path
    k_curv = interp1(path.s_m, path.k_1pm, s_m);
    ux_des = interp1(path.s_m, path.ux_des,s_m);
    ax_des = interp1(path.s_m, path.ax_des,s_m);
    
    % 1b define steady state yaw error
    delta_psi_ss = k_curv*((m*a*ux^2/(L*(C_alphar_linear)))-b); 
    
    % 1c define delta feed forward
    p = (kla*xla*delta_psi_ss/C_alphaf_linear);
    q = k_curv*(L + ux^2*k_understeer);
    delta_ff = p + q;

    % 1d define lookahead control input delta with feed forward
    ela         = (e+xla*delta_psi);
    delta_la    = (-kla*ela/C_alphaf_linear);
    
    % 1e define final delta based on individual delta controls
    delta_rad   = delta_la + delta_ff;
    
    %% 2 PD Longitudinal Fx_N Control
    % 2a drag force
    F_d = 0.5 * air_density * ux^2 * C_DA;
    
    % 2b rolling resistance 
    F_rr = f_rr * m * g;
    
    % 2c control input
    Fg = 0; %not given grade

    % 2d calculate control throttle
    Fx_N = k_vel*(ux_des - ux)  + F_d + F_rr + Fg + m*ax_des;

    
else % run your second steering controller
     %% 2 PID Steer Control
    kla             = 7000;
    kd           = .08; %.04 when ki = 0
    ki           = 0.003;
    
    % 1a get curvature  of path
    k_curv = interp1(path.s_m, path.k_1pm, s_m);
    ux_des = interp1(path.s_m, path.ux_des,s_m);
    ax_des = interp1(path.s_m, path.ax_des,s_m);
    
    % 1b define steady state yaw error
    delta_psi_ss = k_curv*((m*a*ux^2/(L*(C_alphar_linear)))-b); 
    
    % 1c define delta feed forward
    p = (kla*xla*delta_psi_ss/C_alphaf_linear);
    q = k_curv*(L + ux^2*k_understeer);
    delta_ff = p + q;

%     % 1d define a nonlinear, proportial derivative control element
%     e_tol_upperlimit = 4.5e-06; 
%     e_tol_lowerlimit = -4.5e-06;
%     abs(e) - abs(last_e);
%     if ( (abs(e) - abs(last_e)) > e_tol_upperlimit)
%         kd = 200;
%     end
%     if ( (abs(e) - abs(last_e)) < e_tol_lowerlimit)
%         kd = 0;
%     end
    e_dot = (e - last_e)/dT;
    delta_der = - kd*e_dot;

    % 1e define lookahead control input delta with feed forward
    ela         = (e+xla*delta_psi);
    delta_la    = (-kla*ela/C_alphaf_linear);
    
    % 1f define a integral control on
    delta_int   = -ki*e_sum;
    
    % 1g define final delta based on individual delta controls
    if(k_curv <= 0.01 && last_k >0.01)
        e_sum = 0;
    end
    
    delta_rad   = delta_la + delta_ff + delta_der + delta_int;
    
   
    last_e = e;

    %% 2 PD Longitudinal Fx_N Control
    % 2a drag force
    F_d = 0.5 * air_density * ux^2 * C_DA;
    
    % 2b rolling resistance 
    F_rr = f_rr * m * g;
    
    % 2c control input
    Fg = 0; %not given grade

    % 2d calculate control throttle
    Fx_N = k_vel*(ux_des - ux)  + F_d + F_rr + Fg + m*ax_des;

    last_k = k_curv;
end

end
