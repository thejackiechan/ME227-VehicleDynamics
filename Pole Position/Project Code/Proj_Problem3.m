C_alphaf = 275000; %N/rad
C_alphar = 265000; %N/rad
u_f = 0.97;
u_fs = 0.97;
u_r = 1.03;
u_rs = 1.03;
m = 1648; % kg 
Iz = 2235; % kg m2
L = 2.468; % m
percent_front = .577;
percent_rear = .423;

%(1)
g = 9.8;
Fx_total = 0.1*m*g;
ux_diff = 1;
k_drive = Fx_total/ux_diff

Kla = 3500;
xla = 15;
Ux_des = 15;
e0 = 1;
ux0 = 13;
s = 0;
k = 0;

[t_s,ux_mps, uy_mps, r_radps, s_m, e_m, delta_psi_rad] = proj_nonlinear_sim(C_alphaf, C_alphar, u_f, u_fs, u_r, u_rs, m,Iz, L, s, k, Ux_des, percent_front, percent_rear, Kla, xla, k_drive, e0, ux0);

figure()
plot(t_s, ux_mps)
title('Longitudinal Velocity vs Time')
xlabel('time (s)')
ylabel('Longitudinal Velocity (m/s)')

figure()
plot(t_s, uy_mps)
title('Lateral Velocity vs Time')
xlabel('time (s)')
ylabel('Lateral Velocity (m/s)')

figure()
plot(t_s, r_radps)
title('Yaw Rate vs Time')
xlabel('time (s)')
ylabel('Yaw rate (rad/s)')

figure()
plot(t_s, s_m)
title('Path Position vs Time')
xlabel('time (s)')
ylabel('Path Position (m)')

figure()
plot(t_s, e_m)
title('Lateral Error vs Time')
xlabel('time (s)')
ylabel('Lateral Error (m)')

figure()
plot(t_s, delta_psi_rad)
title('Heading Error vs Time')
xlabel('time (s)')
ylabel('Heading Error (rad)')

%(2)
C_alphaf = 275000; %N/rad
C_alphar = 265000; %N/rad
u_f = 0.97;
u_fs = 0.97;
u_r = 1.03;
u_rs = 1.03;
m = 1648; % kg 
Iz = 2235; % kg m2
L = 2.468; % m
percent_front = .577;
percent_rear = .423;

%(1)
g = 9.8;
Fx_total = 0.1*m*g;
ux_diff = 1;
k_drive = Fx_total/ux_diff

Kla = 3500;
xla = 15;
Ux_des = 15;
e0 = 1;
ux0 = 15;

%path info
% constant radius  path
s = [0      350];
k = [1/40   1/40];

[t_s, ux_mps, uy_mps, r_radps, s_m, e_m, delta_psi_rad] = nonlinear_sim(C_alphaf, C_alphar, u_f, u_fs, u_r, u_rs, m,Iz, L, s, k, Ux_des, percent_front, percent_rear, Kla, xla, k_drive, e0, ux0);

% "undulating" path
s1 = [0      20      40   150];
k1 = [1/20   -1/20   0    0];

[t_s1, ux_mps1, uy_mps1, r_radps1, s_m1, e_m1, delta_psi_rad1] = nonlinear_sim(C_alphaf, C_alphar, u_f, u_fs, u_r, u_rs, m,Iz, L, s1, k1, Ux_des, percent_front, percent_rear, Kla, xla, k_drive, e0, ux0);

figure()
plot(t_s, e_m)
hold on
plot(t_s1, e_m1)
title('Lateral Error vs Time')
xlabel('time (s)')
ylabel('Lateral Error (m)')
legend('Constant Radius Path', 'Undulating Path')

figure()
plot(t_s, delta_psi_rad)
hold on
plot(t_s1, delta_psi_rad1)
title('Heading Error vs Time')
xlabel('time (s)')
ylabel('Heading Error (rad)')
legend('Constant Radius Path', 'Undulating Path')

%%  (3)
close all
clear
C_alphaf = 275000; %N/rad
C_alphar = 265000; %N/rad
u_f = 0.97;
u_fs = 0.97;
u_r = 1.03;
u_rs = 1.03;
m = 1648; % kg 
Iz = 2235; % kg m2
L = 2.468; % m
percent_front = .577;
percent_rear = .423;

%(1)
g = 9.8;
Fx_total = 0.1*m*g;
ux_diff = 1;
k_drive = Fx_total/ux_diff

Kla = 3500;
xla = 15;
Ux_des = 15;
e0 = 1;
ux0 = 15;

%path info
% constant radius  path
s = [0      150];
k = [1/40   1/40];

[t_s, ux_mps, uy_mps, r_radps, s_m, e_m, delta_psi_rad] = proj_nonlinear_sim(C_alphaf, C_alphar, u_f, u_fs, u_r, u_rs, m,Iz, L, s, k, Ux_des, percent_front, percent_rear, Kla, xla, k_drive, e0, ux0);

% "undulating" path
s1 = [0      20      40   150];
k1 = [1/20   -1/20   0    0];

[t_s1, ux_mps1, uy_mps1, r_radps1, s_m1, e_m1, delta_psi_rad1] = proj_nonlinear_sim(C_alphaf, C_alphar, u_f, u_fs, u_r, u_rs, m,Iz, L, s1, k1, Ux_des, percent_front, percent_rear, Kla, xla, k_drive, e0, ux0);

figure()
plot(t_s, e_m)
hold on
plot(t_s1, e_m1)
title('Lateral Error vs Time')
xlabel('time (s)')
ylabel('Lateral Error (m)')
legend('Constant Radius Path', 'Undulating Path')

figure()
plot(t_s, delta_psi_rad)
hold on
plot(t_s1, delta_psi_rad1)
title('Heading Error vs Time')
xlabel('time (s)')
ylabel('Heading Error (rad)')
legend('Constant Radius Path', 'Undulating Path')
