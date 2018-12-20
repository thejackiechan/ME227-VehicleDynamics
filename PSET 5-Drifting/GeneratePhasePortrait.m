function stateDot = GeneratePhasePortrait(uy,r,mode,desiredStateDerivative)
% GeneratePhasePortrait interfaces to PPLANE8 phase portrait
% plotting by providing computations of a vehicle's state derivatives in a batch format
%as required by PPLANE8
% Author: Rami Hindiyeh
% Date: April 19, 2011

%Define tire position indices
lf = 1; rf = 2; lr = 3; rr = 4;

%Vehicle parameters (Marty)
veh.m = 1450; %mass in kg
veh.Ca = [60000/2; 60000/2; 160000/2; 160000/2]; %cornering stiffnesses in N/rad
veh.Izz = 2300; %Yaw inertia in kg-m^2
veh.a = 1.608; %Distance from CG to front axle in m
veh.b = .792; %Distance from CG to rear axle in m
veh.L = veh.a + veh.b; %Wheelbase in m
veh.mu_peak = [1.1 1.1 0.967 0.967]'; %Peak friction coefficient
veh.mu_slide= [1.1 1.1 0.967 0.967]'; %sliding friction coefficient

%Define constant delta (must be fixed for phase portrait)
delta = -10*pi/180*[1 1 0 0]';

%Set simulation parameters
simulation.g = 9.81;
simulation.speed = 8;

%Static normal loads
Fz = [0.5*veh.b*veh.m*simulation.g/veh.L 0.5*veh.b*veh.m*simulation.g/veh.L ...
    0.5*veh.a*veh.m*simulation.g/veh.L 0.5*veh.a*veh.m*simulation.g/veh.L]';

% Wf = Fz(lf) + Fz(rf);
% Wr = Fz(lr) + Fz(rr);
% Cf = veh.Ca(lf) + veh.Ca(rf);
% Cr = veh.Ca(lr) + veh.Ca(rr);
% K = Wf/Cf - Wr/Cr

%'linear' simulation mode = bike model with linear tires. 'nonlinear'
%simulation mode = four wheel model with fiala lateral tire model
switch(lower(mode))
    case 'linear'
        simulation.vmodel = 'bike';
        simulation.tmodel = 'linear';
    case 'nonlinear'
%         simulation.vmodel = 'fourwheel';
        simulation.vmodel = 'bike';
        simulation.tmodel = 'fialalat';
    otherwise
        error('INVALID PHASE PORTRAIT MODE SPECIFIED')
end

%Store Uy and r inputs into state vector;
state = [uy; r];

%Compute slip angles
alpha = slipsPP(simulation,veh,state,delta);
%Compute tire lateral forces and state derivatives
switch(lower(mode))
    case 'linear'
        Fy = tireforcesPP(simulation,veh,alpha);
        dxdt = derivsPP(simulation,veh,state,Fy);
    case 'nonlinear'
        Fy = tireforcesPP(simulation,veh,alpha,Fz);
%         dxdt = derivsPP(simulation,veh,state,Fy,delta);
        dxdt = derivsPP(simulation,veh,state,Fy);

    otherwise
        error('INVALID PHASE PORTRAIT MODE SPECIFIED')
end

%Based on desiredState input, output desired state derivatives
switch(lower(desiredStateDerivative))
    case 'uydot'
        stateDot = dxdt(1,:);
    case 'rdot'
        stateDot = dxdt(2,:);
    otherwise
        error('INVALID DESIRED STATE DERIVATIVE SPECIFIED')
end
