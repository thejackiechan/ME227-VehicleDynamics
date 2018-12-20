function varargout = slipsPP(simulation,vehicle,state,delta)
% slipsPP - Calculation of tire slip angles with flexibility to handle linear
% and non-linear cases for purposes of phase portrait generation with
% PPLANE7
% Author: Rami Hindiyeh
% Date: April 4, 2011
%
% Usage: outputVariable = slipangles(simulation,vehicle,state,delta) where
% simulation is a structure that contains a variable which tells which vehicle
% model to use to calculate the slip angles. Vehicle contains the
% parameterization of the vehicle being used, while state and delta give the
% current vehicle state and driver input, respectively.  The slip angles are
% returned in a structure that will change, depending on the model being used to
% calculate the slip angles.
%
% Example: If simulation.model = 'bike', then alpha will contain the slip angles
% for all four wheels calculated with the bicycle model.

% Enumerate the wheels (this should appear in all of your files)
lf = 1; rf = 2; lr = 3; rr = 4;


% Here we use a "switch" statement to determine which type of vehicle model gets
% used to calculate the slip angles. The function 'lower' is used to help make
% the function more robust as it will convert the input string to lower-case
switch lower(simulation.vmodel)
    % Using the bicycle model
    case 'bike'
        if (isfield(vehicle, 'a') & isfield(vehicle, 'b') & isfield(simulation, 'speed'))
            %Parse state vector and speed information
            Uy = state(1,:);
            r = state(2,:);
            Ux = simulation.speed;
            a = vehicle.a;
            b = vehicle.b;
            if Ux >= 1
                alpha(lf,:) = (Uy + a*r)./Ux - delta(lf);
                alpha(rf,:) = (Uy + a*r)./Ux - delta(rf);
                alpha(lr,:) = (Uy - b*r)./Ux - delta(lr);
                alpha(rr,:) = (Uy - b*r)./Ux - delta(rr);
            else
                for wheel = lf:rr
                    alpha(wheel,:) = zeros(1,size(Uy,2));
                end
            end
            
            varargout{1} = alpha;

        else
            error('PARAMETERS MISSING: Check a, b, or speed')
        end
        
        

    % Add other models here
    % eg. case 'fourwheel'

    % Any time you use a switch statement, you should have a default in case
    % there is an error and none of the cases is "activated."  This default
    % should give an indication of what went wrong
    case 'fourwheel'
        switch(lower(simulation.tmodel))
            case{'linear', 'fialalat'}
                if (isfield(vehicle, 'a') & isfield(vehicle, 'b') & isfield(vehicle, 'd') & isfield(simulation, 'speed'))
                    Uy = state(1,:);
                    r = state(2,:);
                    Ux = simulation.speed;
                    a = vehicle.a;
                    b = vehicle.b;
                    d = vehicle.d;

                    if Ux >= 1
                        alpha(lf,:) = atan2(Uy + a*r,Ux-(d/2)*r) - delta(lf);
                        alpha(rf,:) = atan2(Uy + a*r,Ux+(d/2)*r) - delta(rf);
                        alpha(lr,:) = atan2(Uy - b*r,Ux-(d/2)*r) - delta(lr);
                        alpha(rr,:) = atan2(Uy - b*r,Ux+(d/2)*r) - delta(rr);
                    else
                        for wheel = lf:rr
                            alpha(wheel,:) = zeros(1,size(Uy,2));
                        end
                    end
                    varargout{1} = alpha;

                else
                    error('PARAMETERS MISSING: Check a, b, d, or speed')
                end

            otherwise
                error('INVALID TIRE MODEL SPECIFIED')
        end
                
                
    otherwise
        error('INVALID VEHICLE MODEL SPECIFIED')
end
