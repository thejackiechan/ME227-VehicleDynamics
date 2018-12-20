function dxdt = derivsPP(simulation,vehicle,state, varargin)
% derivsPP - Calculate state derivatives for specified vehicle model for
% computation of phase portrait using PPLANE7
% Author: Rami Hindiyeh
% Date: April 19, 2011
% 
% Usage: dxdt = derivs(simulation,vehicle,state,varargin)
% where simulation is a structure that supplies necessary information about the
% simulation environment, vehicle is a structure that describes the physical
% properties of the vehicle being simulated, state is defined by the
% states being used in the simulation and varargin contains the necessary vector
% of the forces on the vehicle. The output is a vector of the state derivatives
% of the system described by a bicycle vehicle model.

% Enumerate the wheels (this should appear in all your files)
lf = 1; rf = 2; lr = 3; rr = 4;

% Here we use a "switch" statement to determine which derivatives we want
% the code to generate. The function 'lower' is used to help make the function
% more robust as it will convert the input string to lower-case
switch lower(simulation.vmodel)
    % Bicycle Model 
    case 'bike'             
        if (isfield(vehicle, 'm') & isfield(vehicle,'Izz') & isfield(simulation, 'speed')...
                & isfield(vehicle, 'a') & isfield(vehicle, 'b'))
            a = vehicle.a;
            b = vehicle.b;
            Izz = vehicle.Izz;
            m = vehicle.m;
            Ux = simulation.speed;
            if (nargin == 4)
                %Parse tire lateral forces from varargin
                Fy = varargin{1};
                for i = 1:size(state,2)
                    %Parse state variables (as necessary) from state vector
                    r = state(2,i);
                    %Compute derivative for each state from governing
                    %equation
                    Uydot(i) = ((Fy(lf,i) + Fy(rf,i)) + (Fy(lr,i)+Fy(rr,i)))/m -r*Ux;
                    rdot(i) = (a*(Fy(lf,i)+ Fy(rf,i)) - b*(Fy(lr,i)+Fy(rr,i)))/Izz;
                end
                
                %Concatenate into state derivative vector
                dxdt = [Uydot; rdot];
                
            else
                error('IMPROPER NUMBER OF INPUT ARGUMENTS')
            end
                    
        else                            
            error('PARAMETERS MISSING: Check m, Izz, a, b, and speed');
        end

    % Add more cases to make your code more flexible and add functionality 
    case 'fourwheel'
        switch(lower(simulation.tmodel))
            case{'linear', 'fialalat'}
                    if (isfield(vehicle, 'm') & isfield(vehicle,'Izz') & isfield(simulation, 'speed')...
                            & isfield(vehicle, 'a') & isfield(vehicle, 'b') & isfield(vehicle, 'd'))
                        a = vehicle.a;
                        b = vehicle.b;
                        Izz = vehicle.Izz;
                        m = vehicle.m;
                        d = vehicle.d;
                        Ux = simulation.speed;
                        if (nargin == 5)
                            %Parse tire lateral forces from varargin
                            Fy = varargin{1};
                            delta = varargin{2};
                            %Parse state variables (as necessary) from state vector

                            %Compute derivative for each state from governing
                            %equation
            
                            for i = 1:size(state,2)
                                r = state(2,i);
                                Uydot(i) = ((Fy(lf,i)*cos(delta(lf)) + Fy(rf,i)*cos(delta(rf))) + (Fy(lr,i)+Fy(rr,i)))/m -r*Ux;
                                rdot(i) = (a*(Fy(lf,i)*cos(delta(lf))+ Fy(rf,i)*cos(delta(rf))) - b*(Fy(lr,i)+Fy(rr,i)) + ...
                                    (d/2)*(Fy(lf,i)*sin(delta(lf)) -Fy(rf,i)*sin(delta(rf))))/Izz;
                            end
                            %Concatenate into state derivative vector
                            dxdt = [Uydot; rdot];
                              

                        else
                            error('IMPROPER NUMBER OF INPUT ARGUMENTS')
                        end
                    else
                        error('PARAMETERS MISSING: Check m, Izz, a, b, d, and speed')
                    end
            otherwise
                error('INVALID TIRE MODEL SPECIFIED')
        end
        

    % Any time you use a switch statement, you should have a default in case
    % there is an error and none of the cases is "activated."  This default
    % should give an indication of what went wrong
    otherwise
        error('INVALID VEHICLE MODEL SPECIFIED')
        
end

