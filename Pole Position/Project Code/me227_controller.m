function [ delta_rad, Fx_N ] = me227_controller( s_m, e_m, deltaPsi_rad, Ux_mps, Uy_mps, r_radps, modeSelector, path )
% ME227_CONTROLLER Summary of this function goes here
%   Include your names in the comments at the top of the function.  The
%   state and controls follow the notation in class and have the units
%   appended.  The parameter path matches what you have in simulation i.e.
%   it has the following fields s_m, k_1pm, psi_rad, posE_m, and posN_m.

%#codegen % this directive is needed to run your code on the car

% Define our Car parameters

% Front wheel
% uf = 0.97;
% ufs = uf;
% 
% ca_r = 265000;
% ur = 1.03;
% urs = ur;
% 
% m       = 1648; %kg
% Iz      = 2235; %kg m^2
% L       = 2.468;
% wfp     = 57.7; % percent weight front
% wrp     = 42.3; % percent weight rear
% 
% s0      = 0;
% e0      = 1;
% dpsi0   = 0;
% ux0     = 13;
% % Create a car state and car properties 
% car_prop  = [m, Iz, L, wfp, wrp, ca_f, ca_r,uf, ur];
% car_state = [s0 e0 dpsi0 ux0]; 

C_alphaf = 275000; %N/rad
C_alphar = 265000; %N/rad
u_f = 0.97;
u_fs = 0.97;
u_r = 1.03;
u_rs = 1.03;
m = 1659; % kg 
Iz = 2447; % kg m2
L = 2.468; % m
percent_front = .577;
percent_rear = .423;

f_rr = 0.0157;
C_DA = 0.594;

%(1)
g = 9.8;
Fx_total = 0.3*m*g;
ux_diff = 1;
k_vel = Fx_total/ux_diff;

k_accel = 0.01 * k_vel; % change this later

Kla = 3500;
xla = 15;
Ux_des = 15;

e0 = 1;
ux0 = 0;

%path info
% constant radius  path
% s = [0      350];
% k = [1/40   1/40];

% Straight path
s = [0      300];
k =  [0    0];


N = length(Ux_mps);
dT = 0.005; %sec
T_total = N*dT;
t_s = 0:dT:T_total;

vx_prof  = zeros(N,1);
vy_prof  = zeros(N,1);
ax_prof  = zeros(N,1);
ay_prof  = zeros(N,1);

if modeSelector == 1 % run your first steering control scheme
    % Call to proj_nonlinear_sim()
    % proj_nonlinear_sim(C_alphaf, C_alphar, u_f, u_fs, u_r, u_rs, m,Iz, L, s,
  ...k, Ux_des, percent_front, percent_rear, Kla, xla, k_drive, e0, ux0);
     
    % Create our velocity profile, acceleration profile
    % from 0 to 3 seconds
        ax_des = 3;
        vx_des = 9;
        decel_time = 7;
        
    for idx = 1:N
        if (t_s(idx) < 3) 
            
            % constant acceleration of 3 m/sec^2
            ax_prof(idx) = ax_des;
            vx_prof(idx) = ax_des*t_s(idx);
            
        elseif (t_s(idx) < decel_time)
           
            % constant speed of 9 m/sec
            ax_prof(idx) = 0;
            vx_prof(idx) = vx_des;
            
        else 
            % constant deceleration of -3m/sec^2
            ax_prof(idx) = -ax_des;
            vx_prof(idx) = vx_des - ax_des*(t_s(idx) - decel_time); 
                
        end
    end
    
    %We get back our states 
    [ t_s, ux_mps, uy_mps, r_radps, s_m, e_m, delta_psi_rad, Fx_N, delta_rad ] = proj_nonlinear_sim(C_alphaf, C_alphar, u_f,...
    u_fs, u_r, u_rs, m,Iz, L, s, k, percent_front, percent_rear,...
    Kla, xla, k_vel, e0, ux0, vx_prof, ax_prof, k_accel, f_rr, C_DA)
else % run your second steering controller
end
figure
plot(t_s,e_m);
title('Lateral Error');

figure
plot(t_s,abs(ux_mps-vx_prof));
title('longitudinal speed error');


end

