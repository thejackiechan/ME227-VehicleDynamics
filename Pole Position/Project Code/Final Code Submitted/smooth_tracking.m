function [ vx_prof, ax_prof ] = smooth_tracking( path )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
vx_prof = zeros(length(path.s_m),1);
ax_prof = zeros(length(path.s_m),1);

%% Constants
R = 8.7; %m
amax = 2.75;
%ax_max_brake = -4;
ax_max = 2.75;
v_arc = sqrt(R*amax);
%Lstraight = 24.1;
ds = 0.25;

%% 0 Find all indexes and distance along path to first clothoid
[indices, curvatures] = find_indices(path);
% get indexing from indices vector

beginStraight1 = indices(1);
s_ecl1_idx = indices(2);
endStraight1 = s_ecl1_idx - 1;
enterArc1 = indices(3);
s_xcl1_idx = enterArc1 - 1;
s_ecl2_idx = indices(4);
exitArc1 = s_ecl2_idx - 1;
enterStraight2 = indices(5);
s_xcl2_idx = enterStraight2 - 1;
s_ecl3_idx = indices(6);
exitStraight2 = s_ecl3_idx - 1;
enterArc2 = indices(7) - 1; % curvature here is pretty close to constant radius curvature
s_xcl3_idx = enterArc2 - 1;
s_ecl4_idx = indices(8) + 1; % same curvature problem, need some sort of tolerance (maybe 0.1)
exitArc2 = s_ecl4_idx - 1;
enterStraight3 = indices(9);
s_xcl4_idx = enterStraight3 - 1;
s_ecl5_idx = indices(10); % 553
exitStraight3 = s_ecl5_idx - 1; % 552
enterArc3 = indices(11); % 600
s_xcl5_idx = enterArc3 - 1; % 599
s_ecl6_idx = indices(12) + 1; % 663
exitArc3 = s_ecl6_idx - 1; % 662
enterStraight4 = indices(13); % 709
s_xcl6_idx = enterStraight4 - 1; % 708
s_ecl7_idx = indices(14); % 805
exitStraight4 = s_ecl7_idx - 1; %804
enterArc4 = indices(15) - 1; % 852
s_xcl7_idx = enterArc4 - 1; % 851
s_ecl8_idx = indices(16); % 914
exitArc4 = s_ecl8_idx - 1; % 913
enterLastStraight = indices(17); % 961
s_xcl8_idx = enterLastStraight - 1; % 960
finish = length(path.s_m);


% set known velocities in vx_prof
vx_prof(s_xcl1_idx) = v_arc;
vx_prof(s_ecl2_idx) = v_arc;
vx_prof(s_xcl3_idx) = v_arc;
vx_prof(s_ecl4_idx) = v_arc;
vx_prof(s_xcl5_idx) = v_arc;
vx_prof(s_ecl6_idx) = v_arc;
vx_prof(s_xcl7_idx) = v_arc;
vx_prof(s_ecl8_idx) = v_arc;

% set known accelerations in ax_prof

%% 1 Solve for the velocity, acceleration profile through first straight
for i = beginStraight1:endStraight1-1
    vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*ax_max*ds);
    ax_prof(i+1) = ax_max;
end
  

%% 2 Define the velocity and acceleration profile from the first clothoid 
clothoid3_indexes = s_xcl1_idx - s_ecl1_idx; % clothoid 1 and 2 have the same length so thats cool

for i = 1:clothoid3_indexes
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl1_idx - i + 1)*vx_prof(s_xcl1_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl1_idx - i + 1);
    vx_prof(s_xcl1_idx - i) = vx_prof(s_xcl1_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl1_idx - i + 1) = -sqrt(amax^2-(vx_prof(s_xcl1_idx - i + 1)^2*path.k_1pm(s_xcl1_idx - i + 1))^2);
end 
% populate last unknown ax_prof at s_ecl1_idx
ax_prof(s_ecl1_idx) = sqrt(amax^2 - (path.k_1pm(s_ecl1_idx)*vx_prof(s_ecl1_idx)^2)^2)...
        /vx_prof(s_ecl1_idx)*vx_prof(s_ecl1_idx);
    
% smooth the first straight
% 
% for i = endStraight1:s_xcl1_idx
%     if (vx_prof(i) > vx_prof(endStraight1))
%         vx_prof(i) = vx_prof(endStraight1);
%         ax_prof(i) = 0;
%     else
%         break;
%     end
% end
%% 3 Define the first arc
for i = enterArc1:exitArc1
    vx_prof(i) = v_arc;
    ax_prof(i) = 0;
end

%% 4 Solve for the velocity and acceleration profile exiting the second clothoid
for i = s_ecl2_idx:s_xcl2_idx-1
    dux_ds = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2)...
        /vx_prof(i);
    vx_prof(i + 1) = vx_prof(i)+dux_ds*ds;
    ax_prof(i) = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2); %add
end
ax_prof(s_xcl2_idx) = sqrt(amax^2 - (path.k_1pm(s_xcl2_idx)*vx_prof(s_xcl2_idx)^2)^2); %add
    
%% 5 Solve for the velocity and acceleration profile along the second straight
       
% for now, just hold speed, don't accelerate

% ACCELERATION IN RAMP FASHION
ax_max_str      = ax_prof(s_xcl2_idx);
s_start_str     = path.s_m(s_xcl2_idx+1); 
s_end_str       = path.s_m(s_ecl3_idx-1); 
s_straight_mp   =(s_end_str - s_start_str)/2;

for i = s_xcl2_idx:s_ecl3_idx-1
        if (path.s_m(i) - s_start_str) < s_straight_mp
            ax_prof(i+1) = ax_max_str;
        else
            ax_prof(i+1) = -ax_max_str;
        end
        vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*ax_prof(i)*ds);
end

% NO ACCELERATION ON STRAIGHTS CODE
% for i = s_xcl2_idx:s_ecl3_idx-1
%         ax_prof(i+1) = 0;
%         vx_prof(i+1) = vx_prof(i);
% end


%% 6 Define the velocity and acceleration profile from the third clothoid 
clothoid3_indexes = s_xcl3_idx - s_ecl3_idx; % clothoid 1 and 2 have the same length so thats cool

for i = 1:clothoid3_indexes
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl3_idx - i + 1)*vx_prof(s_xcl3_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl3_idx - i + 1);
    vx_prof(s_xcl3_idx - i) = vx_prof(s_xcl3_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl3_idx - i + 1) = -dux_ds*vx_prof(s_xcl3_idx - i + 1);
end 
% populate last unknown ax_prof at s_ecl3_idx
ax_prof(s_ecl3_idx) = -sqrt(amax^2 - (path.k_1pm(s_ecl3_idx)*vx_prof(s_ecl3_idx)^2)^2)...
        /vx_prof(s_ecl3_idx)*vx_prof(s_ecl3_idx);

%% NEW CODE TO SMOOTH velocity profiles
% [vmaxst2, vmax_st2_idx] = max(vx_prof(s_xcl2_idx:s_ecl3_idx-1));
% vf = vx_prof(s_ecl3_idx);
% vi = vmaxst2;
% a2_smooth_max = (vf^2-vi^2)/(2*s_straight_mp);
% 
% for i = vmax_st2_idx:s_ecl3_idx-1
%     ax_prof(i+1) = a2_smooth_max;
%     vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*ax_prof(i+1)*ds);
% end

%% 7 Define the second arc
for i = enterArc2:exitArc2
    vx_prof(i) = v_arc;
    ax_prof(i) = 0;
end

%% 8 Solve for the velocity and acceleration profile exiting the fourth clothoid

for i = s_ecl4_idx:s_xcl4_idx-1
    dux_ds = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2)...
        /vx_prof(i);
    vx_prof(i + 1) = vx_prof(i)+dux_ds*ds;
    ax_prof(i) = dux_ds * vx_prof(i); %add
end
ax_prof(s_xcl4_idx) = sqrt(amax^2 - (path.k_1pm(s_xcl4_idx)*vx_prof(s_xcl4_idx)^2)^2)...
        /vx_prof(s_xcl4_idx) * vx_prof(s_xcl4_idx); %add    
    
%% 9 Solve for the velocity and acceleration profile along the third straight
% ACCELERATION IN RAMP FASHION
ax_max_str      = ax_prof(s_xcl4_idx);
s_start_str     = path.s_m(s_xcl4_idx+1); 
s_end_str       = path.s_m(s_ecl5_idx-1); 
s_straight_mp   =(s_end_str - s_start_str)/2;

for i = s_xcl4_idx:s_ecl5_idx-1
        if (path.s_m(i) - s_start_str) < s_straight_mp
            ax_prof(i+1) = ax_max_str;
        else
            ax_prof(i+1) = -ax_max_str;
        end
        vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*ax_prof(i)*ds);
end
% 
% for i = s_xcl4_idx:s_ecl5_idx-1
%         ax_prof(i+1) = 0;
%         vx_prof(i+1) = vx_prof(i);
% end

%% 10 Define the velocity and acceleration profile from the fifth clothoid 
clothoid5_indexes = s_xcl5_idx - s_ecl5_idx; % clothoid 1 and 2 have the same length so thats cool

for i = 1:clothoid5_indexes
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl5_idx - i + 1)*vx_prof(s_xcl5_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl5_idx - i + 1);
    vx_prof(s_xcl5_idx - i) = vx_prof(s_xcl5_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl5_idx - i + 1) = -dux_ds*vx_prof(s_xcl5_idx - i + 1);
end 
% populate last unknown ax_prof at s_ecl5_idx
ax_prof(s_ecl5_idx) = -sqrt(amax^2 - (path.k_1pm(s_ecl5_idx)*vx_prof(s_ecl5_idx)^2)^2)...
        /vx_prof(s_ecl5_idx)*vx_prof(s_ecl5_idx);
    
%% 11 Define the third arc
for i = enterArc3:exitArc3
    vx_prof(i) = v_arc;
    ax_prof(i) = 0;
end

%% 12 Solve for the velocity and acceleration profile exiting the sixth clothoid

for i = s_ecl6_idx:s_xcl6_idx-1
    dux_ds = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2)...
        /vx_prof(i);
    vx_prof(i + 1) = vx_prof(i)+dux_ds*ds;
    ax_prof(i) = dux_ds * vx_prof(i); %add
end
ax_prof(s_xcl6_idx) = sqrt(amax^2 - (path.k_1pm(s_xcl6_idx)*vx_prof(s_xcl6_idx)^2)^2)...
        /vx_prof(s_xcl6_idx) * vx_prof(s_xcl6_idx); %add    
    
%% 13 Solve for the velocity and acceleration profile along the fourth straight
% ACCELERATION IN RAMP FASHION
ax_max_str      = ax_prof(s_xcl6_idx);
s_start_str     = path.s_m(s_xcl6_idx+1); 
s_end_str       = path.s_m(s_ecl7_idx-1); 
s_straight_mp   =(s_end_str - s_start_str)/2;

for i = s_xcl6_idx:s_ecl7_idx-1
        if (path.s_m(i) - s_start_str) < s_straight_mp
            ax_prof(i+1) = ax_max_str;
        else
            ax_prof(i+1) = -ax_max_str;
        end
        vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*ax_prof(i)*ds);
end

%
% for i = s_xcl6_idx:s_ecl7_idx-1
%     ax_prof(i+1) = 0;
%     vx_prof(i+1) = vx_prof(i);
% end

%% 14 Define the velocity and acceleration profile from the seventh clothoid 
clothoid7_indexes = s_xcl7_idx - s_ecl7_idx; % clothoid 1 and 2 have the same length so thats cool

for i = 1:clothoid7_indexes
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl7_idx - i + 1)*vx_prof(s_xcl7_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl7_idx - i + 1);
    vx_prof(s_xcl7_idx - i) = vx_prof(s_xcl7_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl7_idx - i + 1) = -dux_ds*vx_prof(s_xcl7_idx - i + 1);
end 
% populate last unknown ax_prof at s_ecl7_idx
ax_prof(s_ecl7_idx) = -sqrt(amax^2 - (path.k_1pm(s_ecl7_idx)*vx_prof(s_ecl7_idx)^2)^2)...
        /vx_prof(s_ecl7_idx)*vx_prof(s_ecl7_idx);
    
%% 15 Define the fourth arc
for i = enterArc4:exitArc4
    vx_prof(i) = v_arc;
    ax_prof(i) = 0;
end

%% 16 Solve for the velocity and acceleration profile exiting the eighth clothoid

for i = s_ecl8_idx:s_xcl8_idx-1
    dux_ds = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2)...
        /vx_prof(i);
    vx_prof(i + 1) = vx_prof(i)+dux_ds*ds;
    ax_prof(i) = dux_ds * vx_prof(i); %add
end
ax_prof(s_xcl8_idx) = sqrt(amax^2 - (path.k_1pm(s_xcl8_idx)*vx_prof(s_xcl8_idx)^2)^2)...
        /vx_prof(s_xcl8_idx) * vx_prof(s_xcl8_idx); %add 
    
%% 17 Final Straight
v_straight_final = vx_prof(s_xcl8_idx);
stop_distance = 3; %distance before the end of the path
s_desired_finish = (path.s_m(finish) - path.s_m(s_xcl8_idx))-stop_distance;

% 0.1 to end at a negative commanded velocity so that we have some
% force resisting idling forward of car
braking_accel = (0.1^2 - vx_prof(s_xcl8_idx)^2)/(2*s_desired_finish);
toler = 0.01;
for i = s_xcl8_idx:finish - 1
    if (vx_prof(i) < toler)
        print = 'vx less than zero'
        vx_prof(i+1) = -0.1;
        ax_prof(i+1) = 0;
    else
        ax_prof(i+1) = braking_accel;
        vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*ax_prof(i+1)*ds);
    end
end
ax_prof(finish) = braking_accel;
vx_prof(finish) = sqrt(vx_prof(finish)^2 + 2*ax_prof(finish+1)*ds);
end

