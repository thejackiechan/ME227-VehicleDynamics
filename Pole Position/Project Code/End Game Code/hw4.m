% % vehicle parameters
% veh.L = 2.468; %m
% veh.m = 1648; %kg
% veh.C_af = 188000; %N/rad
% veh.C_ar = 203000; %N/rad
% veh.Iz = 2235; %kg m^2
% veh.wd = .577; %percent front
% veh.a = (1-veh.wd)*veh.L; %length front
% veh.b = (veh.wd)*veh.L; %length rear
% veh.mu_f = 0.97;
% veh.mu_r = 1.03;
% 
% % speed params
% %Ux = 5:5:30;
% Ux = 30;
% Kla = 3500;
% %Kla = 1000:1000:10000;
% %xla = 10;
% xla = 0:2:30;
% 
% poles = [];
%  
%  for i = 1:length(xla)
%     d1 =  ((veh.C_af+veh.C_ar)*veh.Iz + (veh.a^2*veh.C_af + veh.b^2*veh.C_ar)*veh.m )/(veh.Iz*veh.m*Ux);
%     d2 = ( veh.C_af*veh.C_ar*veh.L^2 + (veh.b*veh.C_ar - veh.a*veh.C_af)*veh.m*Ux^2 ...
%         + Kla*Ux^2*(veh.Iz+veh.m*veh.a*xla(i)) )/(veh.Iz*veh.m*Ux^2);
%     d3 = ( Kla*veh.L*veh.C_ar*(veh.a+xla(i)) )/(veh.Iz*veh.m*Ux);
%     d4 = ( Kla*veh.L*veh.C_ar )/(veh.Iz*veh.m);
%     r = roots([1, d1, d2, d3, d4]);
%     poles = [poles, r];
%      
%  end
%  %plot(real(poles'), imag(poles'),'-x')
% % plotfixer;
%  xlabel('real')
%  ylabel('imaginary')
%  title('2.2: Poles for varying x_l_a')
%  
% %%
% %figure;
%  %Kla = 1000:1000:10000;
%  Kla = 3500;
%  %xla = 15;
%  xla = 25;
%  Ux = 30;
%  for j = 1:length(xla)
%     A = [0, 1, 0, 0;
%      (-Kla/veh.m),(-(veh.C_af + veh.C_ar)/(veh.m*Ux)), ((veh.C_af + veh.C_ar-Kla*xla(j))/(veh.m)), ((-veh.a*veh.C_af + veh.b*veh.C_ar)/(veh.m*Ux));
%      0, 0, 0, 1;
%      (-Kla*veh.a/veh.Iz),((veh.b*veh.C_ar-veh.a*veh.C_af)/(veh.Iz*Ux)), ((veh.a*veh.C_af - veh.b*veh.C_ar-Kla*xla(j)*veh.a)/(veh.Iz)), (-(veh.a^2*veh.C_af + veh.b^2*veh.C_ar)/(veh.Iz*Ux))];
% 
%      % simulation time
%     t_final = 10;
%     dT = 0.01;
%     t_s = 0:dT:t_final;
%     N = length(t_s);
% 
%     % allocate space for simulation data
%     e = zeros(N,1);
%     e_dot = zeros(N,1);
%     dpsi = zeros(N,1);
%     dpsi_dot = zeros(N,1);
% 
%     % set initial conditions
%     e(1) = 1;
%     e_dot(1) = 0;
%     dpsi(1) = 0;
%     dpsi_dot(1) = 0;
% 
%     for i = 1:N
%         x = [e(i); e_dot(i); dpsi(i); dpsi_dot(i)];
%         x_dot = A*x;
%         if i < N
%             e(i+1) = x(1) + x_dot(1)*dT;
%             e_dot(i+1) = x(2) + x_dot(2)*dT;
%             dpsi(i+1) = x(3) + x_dot(3)*dT;
%             dpsi_dot(i+1) = x(4) + x_dot(4)*dT;
%         end
%     end
%     %plot(t_s,e)
%     hold on;
%     %plot(t_s,dpsi)
%  end
% % plotfixer;
%  xlabel('time (s)')
%  ylabel('error')
%  title('2.3: Error for oversteering vehicle')
%  %legend('x_l_a=2','4','6','8','10','12','14','16','18','20')
%  %legend('lateral error (m)','heading error (rad)')

%%

veh.L = 2.468; %m
veh.m = 1648; %kg
veh.C_af = 188000; %N/rad
veh.C_ar = 203000; %N/rad
veh.Iz = 2235; %kg m^2
veh.wd = .577; %percent front
veh.a = (1-veh.wd)*veh.L; %length front
veh.b = (veh.wd)*veh.L; %length rear
veh.mu_f = 0.97;
veh.mu_r = 1.03;
veh.rW = 0.35;
veh.K = veh.m/veh.L*(veh.b*veh.C_ar-veh.a*veh.C_af)/(veh.C_af*veh.C_ar);

% constant radius  path
% s_sim = [0      150];
% k_sim = [1/40   1/40];

% "undulating" path
%  s_sim = [0      20      40   150];
%  k_sim = [1/20   -1/20   0    0];

%load project_data.mat
%path = generate_path(s_sim,k_sim,[0;0;0]);

% simulation time
t_final = 38;
dT = 0.01;
t_s = 0:dT:t_final;
N = length(t_s);

% allocate space for simulation data
e = zeros(N,1);
dpsi = zeros(N,1);
Ux = zeros(N,1);
Uy = zeros(N,1);
r = zeros(N,1);
s = zeros(N,1);
delta = zeros(N,1);

% set initial conditions
e(1) = 0;
dpsi(1) = 0;
Ux(1) = 4;
Uy(1) = 0;
r(1) = 0;
s(1) = 0;

Ux_des = 15;
xla = 15;
Kla = 9500;
Kdrive = .981*veh.m;
%kappa = 0;
 
for i = 1:N
    
    if s(i) < 0
        keyboard;
    end
    % look up K
    
    kappa = interp1(path.s_m, path.k_1pm, s(i));
    
    [delta_fn, Fxtotal] = me227_controller( s(i), e(i), dpsi(i), Ux(i), Uy(i), r(i), 1, path );
    
    % delta terms
%     dpsi_ss = kappa*(veh.m*veh.a*Ux(i)^2/(veh.L*veh.C_ar) - veh.b);
%     delta_ff = Kla*xla/veh.C_af*dpsi_ss + kappa*(veh.L + veh.K*Ux(i)^2);
%     delta(i) = -Kla*(e(i)+xla*dpsi(i))/veh.C_af + delta_ff;
    delta(i) = delta_fn;

    
    % slip angles
    alf_f = atan((Uy(i)+veh.a*r(i))./(Ux(i) + .001) ) - delta(i);
    alf_r = atan((Uy(i)-veh.b*r(i))./(Ux(i) + .001) );
    
    % tire forces
    Fyf = fiala(alf_f, 275000, veh.mu_f, veh.mu_f, veh.wd*(veh.m*9.81));
    Fyr = fiala(alf_r, 265000, veh.mu_r, veh.mu_r, (1-veh.wd)*(veh.m*9.81));
    
    % proportional speed controller
    %Fxtotal = Kdrive*(Ux_des - Ux(i));% + Kacc(Ux_dot_des - Ux_dot(i));
    Fxf = Fxtotal/2;
    Fxr = Fxtotal/2;
    
    Ux_dot = (Fxr + Fxf*cos(delta(i)) - Fyf*sin(delta(i)) + veh.m*r(i)*Uy(i))/veh.m;
    Uy_dot = (Fyf*cos(delta(i)) + Fyr + Fxf*sin(delta(i)) - veh.m*r(i)*Ux(i))/veh.m;
    r_dot = (veh.a*Fyf*cos(delta(i)) + veh.a*Fxf*sin(delta(i)) - veh.b*Fyr)/veh.Iz;
    s_dot = (1/(1-e(i)*kappa))*(Ux(i)*cos(dpsi(i)) - Uy(i)*sin(dpsi(i)));
    e_dot = Uy(i)*cos(dpsi(i)) + Ux(i)*sin(dpsi(i));
    dpsi_dot = r(i) - kappa*s_dot;

    if i < N
        Ux(i+1) = Ux(i) + Ux_dot*dT;
        Uy(i+1) = Uy(i) + Uy_dot*dT;
        r(i+1) = r(i) + r_dot*dT;
        s(i+1) = s(i) + s_dot*dT;
        e(i+1) = e(i) + e_dot*dT;
        dpsi(i+1) = dpsi(i) + dpsi_dot*dT;
    end
end
% figure;
 hold on;
 plot(t_s,Ux)
% plot(t_s,Uy)
% plot(t_s,e)
% plot(t_s,r)
% plot(t_s,s)
%plot(t_s,dpsi)
  title('Lateral error vs. time')
  %legend('lateral error (m)','heading error (rad)')
  xlabel('time (s)')
  ylabel('m')
  %plotfixer;
 animate(path, veh, dpsi, s, e, delta)