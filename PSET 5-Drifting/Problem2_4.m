% Problem 2 Part 4
clear variables; close all; clc

C_front = 60000;
mu = 1.1;
C_rear = 160000;
m = 1450;
Iz = 2300;
L = 2.4;
a = 0.67 * L;
b = 0.33 * L;
g = 9.81;
Ux_steady = 8;
yaw = 1.1858;
lateral_vel = -4.3412;

% Making vehicle struct
veh.m = m;
veh.Iz = Iz;
veh.L = L;
veh.C_front = C_front;
veh.C_rear = C_rear;
veh.a = a;
veh.b = b;

delta_ff = degtorad(-10);
Kx = 2000;
Ux_des = 8;
k_r = 1;
k_y = -0.5;
Uy_des = lateral_vel;
r_des = yaw;
Fz_front = b*m*g/L;
Fz_rear = a*m*g/L;

% simulation time
t_final = 9;
dT = 0.008;
t_s = 0:dT:t_final;
N = length(t_s);

% allocate space for simulation data
Uy_m        = zeros(N,1);
r_m         = zeros(N,1);
Ux_m        = zeros(N,1);
delta_m     = zeros(N,1);

% set initial conditions
Ux_m(1) = 8;
Uy_m(1) = lateral_vel;
r_m(1) = yaw;

% simulation loop
for idx = 1:N
    
    % current states
    Uy = Uy_m(idx);
    r = r_m(idx);
    Ux = Ux_m(idx);
    
    % state equations
    
    Fx_front = 0;
    delta = k_r*(r_des - r) + k_y * (Uy_des - Uy) + delta_ff;
    Fx_ff = m*yaw*Ux_steady*sin(delta_ff)/cos(delta_ff)/(1+a/b) - m*yaw*lateral_vel;
    Fx_rear = Kx*(Ux_des - Ux) + Fx_ff;
    slip_front = atan2((Uy + a*r),Ux) - delta;
    slip_rear = atan2((Uy - b*r),Ux);
    
    % Calculate lateral forces
    Fy_front = coupled(C_front,slip_front,Fz_front,mu,Fx_front);
    Fy_rear = coupled(C_rear,slip_rear,Fz_rear,mu,Fx_rear);
    
    % State equations
    
    Ux_dot = (Fx_rear + Fx_front*cos(delta) - Fy_front*sin(delta))/m + r*Uy;
    Uy_dot = (Fy_front*cos(delta) + Fy_rear + Fx_front*sin(delta))/m - r*Ux;
    r_dot = (a*Fy_front*cos(delta) + a*Fx_front*sin(delta) - b*Fy_rear)/Iz;
    
    
    % only update next state if we are not at end of simulation
    if idx < N
        % euler integration
        Uy_m(idx+1) = Uy_m(idx) + Uy_dot*dT;
        Ux_m(idx+1) = Ux_m(idx) + Ux_dot*dT;
        r_m(idx+1) = r_m(idx) + r_dot*dT;
    end  
end

plot(t_s,r_m);
hold on
plot(t_s,Ux_m);
plot(t_s,Uy_m);
xlabel('Time (s)');
ylabel('Rate (rad/s or m/s)');
title('Coupled Tire Model Parameters vs. Time');
legend('r','U_x','U_y');

animateDrift(Ux_m,Uy_m,r_m,delta_m,veh,dT);
