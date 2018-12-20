function [ vx_prof, ax_prof ] = tracking(path)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
vx_prof = zeros(length(path.s_m),1);
ax_prof = zeros(length(path.s_m),1);

%% Constants
R = 8.7; %m
amax = 3;
amax_brake = -4;
ax_max = 3;
v_arc = sqrt(R*amax);
Lstraight = 24.1;
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
    ax_prof(i+1) = amax;
end
  

%% 2 Define the velocity and acceleration profile from the first clothoid 
clothoid3_indexes = s_xcl1_idx - s_ecl1_idx; % clothoid 1 and 2 have the same length so thats cool

for i = 1:clothoid3_indexes
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl1_idx - i + 1)*vx_prof(s_xcl1_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl1_idx - i + 1);
    vx_prof(s_xcl1_idx - i) = vx_prof(s_xcl1_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl1_idx - i + 1) = dux_ds*vx_prof(s_xcl1_idx - i + 1);
end 
% populate last unknown ax_prof at s_ecl1_idx
ax_prof(s_ecl1_idx) = sqrt(amax^2 - (path.k_1pm(s_ecl1_idx)*vx_prof(s_ecl1_idx)^2)^2)...
        /vx_prof(s_ecl1_idx)*vx_prof(s_ecl1_idx);
    
%% 3 Define the first arc
for i = enterArc1:exitArc1
    vx_prof(i) = v_arc;
    ax_prof(i) = vx_prof(i)^2/R;
end

%% 4 Solve for the velocity and acceleration profile exiting the second clothoid
for i = s_ecl2_idx:s_xcl2_idx-1
    dux_ds = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2)...
        /vx_prof(i);
    vx_prof(i + 1) = vx_prof(i)+dux_ds*ds;
    ax_prof(i) = dux_ds * vx_prof(i); %add
end
ax_prof(s_xcl2_idx) = sqrt(amax^2 - (path.k_1pm(s_xcl2_idx)*vx_prof(s_xcl2_idx)^2)^2)...
        /vx_prof(s_xcl2_idx) * vx_prof(s_xcl2_idx); %add
    
%% 5 Solve for the velocity and acceleration profile along the second straight
v_xcl2 = vx_prof(s_xcl2_idx);
s_break = 0;
while(true)
    LHS = 2*ax_max*s_break;
    RHS = -2*amax_brake*(Lstraight - s_break);
    if (LHS > RHS)
        break;
    end
    s_break = s_break + ds;
end

% for i = s_xcl2_idx:s_ecl3_idx-1
%     if(path.s_m(i) < s_break)
%         ax_prof(i+1) = amax;
%         vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*ax_max*ds);
%     else
%         ax_prof(i+1) = amax_brake;
%         vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*amax_brake*ds);
%     end
% end

s_acc_straight2 = 0:ds:s_break - ds;
ux_acc_straight2 = sqrt(v_xcl2^2 + 2*ax_max*s_acc_straight2);
s_dec_straight2 = 0:ds:(Lstraight-s_break);
ux_dec_straight2 = sqrt(v_xcl2^2 - 2*amax_brake*s_dec_straight2);
ux_prof_straight2 = [ux_acc_straight2 fliplr(ux_dec_straight2)];
ux_prof_straight2(1) = [];

for i = 1:length(ux_prof_straight2) 
    
    vx_prof(s_xcl2_idx + i) = ux_prof_straight2(i);
    
end

%% 6 Define the velocity and acceleration profile from the third clothoid 
clothoid3_indexes = s_xcl3_idx - s_ecl3_idx; % clothoid 1 and 2 have the same length so thats cool

for i = 1:clothoid3_indexes
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl3_idx - i + 1)*vx_prof(s_xcl3_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl3_idx - i + 1);
    vx_prof(s_xcl3_idx - i) = vx_prof(s_xcl3_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl3_idx - i + 1) = dux_ds*vx_prof(s_xcl3_idx - i + 1);
end 
% populate last unknown ax_prof at s_ecl3_idx
ax_prof(s_ecl3_idx) = sqrt(amax^2 - (path.k_1pm(s_ecl3_idx)*vx_prof(s_ecl3_idx)^2)^2)...
        /vx_prof(s_ecl3_idx)*vx_prof(s_ecl3_idx);
    
%% 7 Define the second arc
for i = enterArc2:exitArc2
    vx_prof(i) = v_arc;
    ax_prof(i) = vx_prof(i)^2/R;
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

s_break = 0;
while(true)
LHS = 2*ax_max*s_break;
RHS = -2*amax_brake*(Lstraight - s_break);
if (LHS > RHS)
    break;
end
s_break = s_break + ds;
end

% for i = s_xcl4_idx:s_ecl5_idx-1
% if(path.s_m(i) < s_break)
%     ax_prof(i+1) = amax;
%     vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*ax_max*ds);
% else
%     ax_prof(i+1) = amax_brake;
%     vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*amax_brake*ds);
% end
% end

s_acc_straight3 = 0:ds:s_break - ds;
s_dec_straight3 = 0:ds:(Lstraight-s_break);
ux_dec_straight3 = sqrt(v_xcl2^2 - 2*amax_brake*s_dec_straight3);
ux_prof_straight3 = [ux_acc_straight2 fliplr(ux_dec_straight3)];
ux_prof_straight3(1) = [];

for i = 1:length(ux_prof_straight3) 
    
    vx_prof(s_xcl4_idx + i) = ux_prof_straight3(i);
    
end

%% 10 Define the velocity and acceleration profile from the fifth clothoid 
clothoid5_indexes = s_xcl5_idx - s_ecl5_idx; % clothoid 1 and 2 have the same length so thats cool

for i = 1:clothoid5_indexes
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl5_idx - i + 1)*vx_prof(s_xcl5_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl5_idx - i + 1);
    vx_prof(s_xcl5_idx - i) = vx_prof(s_xcl5_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl5_idx - i + 1) = dux_ds*vx_prof(s_xcl5_idx - i + 1);
end 
% populate last unknown ax_prof at s_ecl5_idx
ax_prof(s_ecl5_idx) = sqrt(amax^2 - (path.k_1pm(s_ecl5_idx)*vx_prof(s_ecl5_idx)^2)^2)...
        /vx_prof(s_ecl5_idx)*vx_prof(s_ecl5_idx);
    
%% 11 Define the third arc
for i = enterArc3:exitArc3
    vx_prof(i) = v_arc;
    ax_prof(i) = vx_prof(i)^2/R;
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

s_break = 0;
while(true)
LHS = 2*ax_max*s_break;
RHS = -2*amax_brake*(Lstraight - s_break);
if (LHS > RHS)
    break;
end
s_break = s_break + ds;
end

% for i = s_xcl6_idx:s_ecl7_idx-1
% if(path.s_m(i) < s_break)
%     ax_prof(i+1) = amax;
%     vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*ax_max*ds);
% else
%     ax_prof(i+1) = amax_brake;
%     vx_prof(i+1) = sqrt(vx_prof(i)^2 + 2*amax_brake*ds);
% end
% end


% s_dec_straight4 = 0:ds:(Lstraight-s_break);
% ux_dec_straight4 = sqrt(v_xcl2^2 - 2*amax_brake*s_dec_straight4);
% ux_prof_straight4 = [ux_acc_straight2 fliplr(ux_dec_straight4)];
% ux_prof_straight4(1) = [];
% 
% ax_acc_straight4 = path.s_m(s_xcl6_idx):ds:s_break + path.s_m(s_xcl6_idx);
% ax_dec_straight4 = s_break + path.s_m(s_xcl6_idx):ds:path.s_m(s_ecl7_idx);
% 
% for i = 1:length(ax_acc_straight4)
%     ax_acc_straight4(i) = amax;
% end
% 
% for i = 1:length(ax_dec_straight4)
%     ax_dec_straight4(i) = amax_brake;
% end
% 
% ax_prof_straight4 = [ax_acc_straight4 ax_dec_straight4];
% 
% for i = 1:length(ux_prof_straight4)
%     
%     vx_prof(s_xcl6_idx + i) = ux_prof_straight4(i);
%     ax_prof(s_xcl6_idx + i) = ax_prof_straight4(i);
% end


%% 14 Define the velocity and acceleration profile from the seventh clothoid 
clothoid7_indexes = s_xcl7_idx - s_ecl7_idx; % clothoid 1 and 2 have the same length so thats cool

for i = 1:clothoid7_indexes
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl7_idx - i + 1)*vx_prof(s_xcl7_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl7_idx - i + 1);
    vx_prof(s_xcl7_idx - i) = vx_prof(s_xcl7_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl7_idx - i + 1) = dux_ds*vx_prof(s_xcl7_idx - i + 1);
end 
% populate last unknown ax_prof at s_ecl7_idx
ax_prof(s_ecl7_idx) = sqrt(amax^2 - (path.k_1pm(s_ecl7_idx)*vx_prof(s_ecl7_idx)^2)^2)...
        /vx_prof(s_ecl7_idx)*vx_prof(s_ecl7_idx);
    
%% 15 Define the fourth arc
for i = enterArc4:exitArc4
    vx_prof(i) = v_arc;
    ax_prof(i) = vx_prof(i)^2/R;
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
s_desired_finish = path.s_m(length(path.s_m)) - path.s_m(enterLastStraight);

braking_slope = - (v_straight_final + 0.1)/s_desired_finish;

for i = s_xcl8_idx:finish - 1
    vx_prof(i+1) = vx_prof(i) + braking_slope*ds;
    ax_prof(i+1) = braking_slope;
    if (vx_prof(i) < 0)
        vx_prof(i) = 0;
        break;
    end
        
end

vx_prof(finish) = - 0.1;
ax_prof(finish) = - 3;
