function [ vx_prof, ax_prof ] = tracking_controller_test( path )
% %UNTITLED Summary of this function goes here
% %   Detailed explanation goes here

accel_until_dist = 0.5*3*3^2
hold_until_dist  = accel_until_dist +36;%m
decel_until_dist = hold_until_dist + 27/2;
vx_prof = zeros(length(path.s_m),1);
ax_prof = zeros(length(path.s_m),1);
ds = 0.25;
for idx = 2:length(path.s_m)
    s = path.s_m(idx);
    if (s <= accel_until_dist) 
        ax_prof(idx) = 3;
        vx_prof(idx) = sqrt(vx_prof(idx-1)^2 + 2*ax_prof(idx)*ds);
    elseif (s <= hold_until_dist)
        ax_prof(idx) = 0;
        vx_prof(idx) = sqrt(vx_prof(idx-1)^2 + 2*ax_prof(idx)*ds);
    elseif (s <= decel_until_dist)
        ax_prof(idx) = -3;
        vx_prof(idx) = sqrt(vx_prof(idx-1)^2 + 2*ax_prof(idx)*ds);
    elseif (s > decel_until_dist)
        vx_prof(idx) = 0;
        ax_prof(idx) = 0;
    end
    
end

% plot(path.s_m,vx_prof);
