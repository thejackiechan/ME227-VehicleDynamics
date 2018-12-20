function varargout = tireforcesPP(simulation,vehicle,varargin)
% tireforcesPP - compute tire forces using simulation-specified model for
% purposes of phase portrait generation with PPLANE7
% Author: Rami Hindiyeh
% Date: April 4, 2011
%
% Usage: Changes depending on the input parameters.  If simulation.tmodel 
% indicates that the desired tire model is linear, then the syntax for calling
% this function should be: latForceVariable =
% tireforces(simulation,vehicle,varargin) which returns a linear tire force
% Fy = -Ca*alpha.  However, if simulation.tmodel indicates a nonlinear model,
% possibly including coupled lateral and longitudinal forces, then the usage
% could change to something like: [latForceVariable,longForceVariable] =
% tireforces(simulation,vehicle,varargin) which returns both lateral and
% longitudinal nonlinear tire forces according to the Fiala tire model.

% Enumerate the wheels (this should appear in all of your files)
lf = 1; rf = 2; lr = 3; rr = 4;

% Here we use a "switch" statement to determine which type of tire model gets
% used to calculate the tire forces. The function 'lower' is used to help make
% the function more robust as it will convert the input string to lower-case
switch lower(simulation.tmodel)
    % Linear tire model
    case 'linear'
        if isfield(vehicle, 'Ca')
            %Parse out relevant quantities
            Ca = vehicle.Ca;
            if nargin == 3
                alpha = varargin{1};
                for i = 1:size(alpha,2)
                    for wheel = lf:rr
                        Fy(wheel,i) = -Ca(wheel)*alpha(wheel,i);
                    end
                end
                varargout{1} = Fy;
            else
                error('IMPROPER NUMBER OF INPUT ARGUMENTS')
            end
        else
            error('VEHICLE PARAMETERS MISSING: Check Ca');
        end
    
    % Add other tire models here
    case 'fialalat'
        if (isfield(vehicle, 'Ca') & isfield(vehicle, 'mu_peak') & isfield(vehicle, 'mu_slide'))
            %Parse out relevant parameters
            Ca = vehicle.Ca;
            mu_peak = vehicle.mu_peak;
            mu_slide = vehicle.mu_slide;

            if nargin == 4
                %Parse out additional input arguments
                alpha = varargin{1};
                Fz = varargin{2};
                for i = 1:size(alpha,2)
                    %For each wheel
                    for wheel = lf:rr
                        %Compute alpha_sl...
                        alpha_sl(wheel) = atan2(3*mu_peak(wheel)*Fz(wheel), Ca(wheel));
                        %Compute tire force based on value of alpha relative to
                        %alpha_sl
                        if (abs(alpha(wheel,i)) < alpha_sl(wheel))
                            Fy(wheel,i) = -Ca(wheel)*tan(alpha(wheel,i)) ...
                                + (Ca(wheel)^2/(3*mu_peak(wheel)*Fz(wheel)))*(2-mu_slide(wheel)/mu_peak(wheel))*tan(alpha(wheel,i))*abs(tan(alpha(wheel,i))) ...
                                - (Ca(wheel)^3/(9*mu_peak(wheel)^2*Fz(wheel)^2))*(tan(alpha(wheel,i)))^3*(1-2*mu_slide(wheel)/(3*mu_peak(wheel)));
                        else
                            Fy(wheel,i) = -mu_slide(wheel)*Fz(wheel)*sign(alpha(wheel,i));
                        end
                    end
                end
                varargout{1} = Fy;
            else
                error('IMPROPER NUMBER OF INPUT ARGUMENTS')
            end
            
            %Define alpha_sl for each tire
            
        else
            error('VEHICLE PARAMETERS MISSING: Check Ca, mu_peak, mu_slide')
        end
    otherwise 
        error('INVALID TIRE MODEL SPECIFIED');       % Another error check
end
