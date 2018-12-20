%% proj_test_harness2
clear variables;
close all;
clc;

load('project_data.mat');
vx_prof = zeros(length(path.s_m),1);
ax_prof = zeros(length(path.s_m),1);
guess_limit =50;
ux_max = 40;
ux_min = 0;
R = 8.7; %m
amax = 4;
ax_max = 3;
v_clothoid_end = sqrt(R*amax);
lclothoid = 11.6; %m
lstraight = 24.1;


% variables for finding start s and end s
s_ecl1 = 0; %ecl for entered clothoid
s_ecl1_idx = 0;
s_xcl1_idx = 0;
s_xcl1 = 0;
tolerance = 0.20;

for idx = 1:length(path.s_m)-1
    if path.k_1pm(idx)==0 && ((path.k_1pm(idx) - path.k_1pm(idx+1)) ~= 0)
        s_ecl1 = path.s_m(idx);
        s_xcl1 = s_ecl1 + lclothoid;
        %index of path entering the first clothoid
        s_ecl1_idx = idx;
    end
    if ((abs(path.s_m(idx)-s_xcl1) < tolerance) && s_xcl1 ~= 0)
        %index exiting the first clothoid
        s_xcl1_idx = idx;
        break
    end
end

% code to fix indexing by 1
s_ecl1_idx = s_ecl1_idx + 1;
s_xcl1_idx = s_xcl1_idx + 1;

% code to construct velocity profile for straight path
% assumes a linear growth in velocity from 0 using y = mx, where m is
% just velocity/length; note that graph is discontinous --> need to scale
% down the other parts of the path; also we start halfway in the middle of
% the straight, not at the beginning

t = sqrt(lstraight/ax_max);
start_idx = 1;
ds = 0.25;

v_straight_1 = ax_max * t;

for i = start_idx:s_ecl1_idx - 1
    
    vx_prof(i) = v_straight_1/lstraight * path.s_m(i);
end

% fill in velocity profile with the speed at the end of first clothoid, this
% is also equal to the velocity of the arc and the speed at the beginning
% of the 2nd arc

clothoid_indexes = s_xcl1_idx - s_ecl1_idx; % clothoid 1 and 2 have the same length so thats cool
s_ecl2_idx = 159; % magic numbers but we need a way to find these indexes (maybe something
s_xcl2_idx = 205;   % similar to the for loop in the beginning)
vx_prof(s_xcl1_idx) = v_clothoid_end;
vx_prof(s_ecl2_idx) = v_clothoid_end;


% fills in velocity profile at the first clothoid
for i = 1:clothoid_indexes
    
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl1_idx - i + 1)*vx_prof(s_xcl1_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl1_idx - i + 1);
    vx_prof(s_xcl1_idx - i) = vx_prof(s_xcl1_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
end

% creates velocity profile through the arc of constant radius
for i = s_xcl1_idx:s_ecl2_idx
    vx_prof(i) = v_clothoid_end;
end

% creates velocity profile throught the second clothoid
for i = s_ecl2_idx:s_xcl2_idx
    
    dux_ds = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2)...
        /vx_prof(i);
    vx_prof(i + 1) = vx_prof(i)+dux_ds*ds;
end

% plots the velocity profile for the indexes we have so far
vx_total = vx_prof(start_idx:s_xcl2_idx);
plot(vx_total);
title('Velocity Profile');


