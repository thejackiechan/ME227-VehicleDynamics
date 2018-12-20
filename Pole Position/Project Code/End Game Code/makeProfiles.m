function[vx_prof, ax_prof, vx_smooth, ax_smooth] = makeProfiles(path)

[indices,curvatures] = find_indices(path);

vx_prof = zeros(length(path.s_m),1);
ax_prof = zeros(length(path.s_m),1);

R = 8.7; %m
amax = 3; % REDUCED. 
amax_brake = -3.75;
ax_max = 3;
v_arc = sqrt(R*amax);
Lstraight = 24.1;
lFirstStraight = 12;
ds = 0.25;

smoothing_factor = 0.7; % default of smoothdata() funct is 0.25
smoothing_factor_accel = .25;

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

% code to construct velocity profile for beginning straight path
% assumes a linear growth in velocity from 0 using y = mx, where m is
% just velocity/length; note that graph is discontinous --> need to scale
% down the other parts of the path; also we start halfway in the middle of
% the straight, not at the beginning

v_straight_1 = sqrt(2*ax_max*lFirstStraight); 

for i = beginStraight1+1:endStraight1
    %vx_prof(i) = v_straight_1/lFirstStraight * path.s_m(i);
     ax_prof(i) = 3;
     vx_prof(i) = sqrt(vx_prof(i-1)^2 + 2*ax_prof(i)*ds);
    %ax_prof(i) = vx_prof(i)/ds;
end

% s_straight_1 = [beginStraight1:ds:endStraight1];
% vx_prof(1:length(s_straight_1)) = sqrt(2*ax_max*s_straight_1);

% fill in velocity profile with the speed at the end of first clothoid, this
% is also equal to the velocity of the arc and the speed at the beginning
% of the 2nd arc


% fills in velocity profile at the first clothoid

clothoid3_indexes = s_xcl1_idx - s_ecl1_idx; % clothoid 1 and 2 have the same length so thats cool

for i = 1:clothoid3_indexes
    
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl1_idx - i + 1)*vx_prof(s_xcl1_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl1_idx - i + 1);
    vx_prof(s_xcl1_idx - i) = vx_prof(s_xcl1_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl1_idx - i) = -vx_prof(s_xcl1_idx - i)*dux_ds;
end


% creates velocity profile through the first arc of constant radius
for i = enterArc1:exitArc1
    vx_prof(i) = v_arc;
    ax_prof(i) = 0;
end

% creates velocity profile throught the second clothoid
for i = s_ecl2_idx:s_xcl2_idx
    
    dux_ds = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2)...
        /vx_prof(i);
    vx_prof(i + 1) = vx_prof(i)+dux_ds*ds;
    ax_prof(i + 1) = vx_prof(i)*dux_ds;
end

% fill in second straight with braking

v_xcl2 = vx_prof(s_xcl2_idx);
v_ecl3 = vx_prof(s_ecl3_idx);
s_break = 0;

while(true)
    LHS = 2*ax_max*s_break;
    RHS = -2*amax_brake*(Lstraight - s_break);
    if (LHS > RHS)
        break;
    end
    s_break = s_break + ds;
end

s_acc_straight2 = 0:ds:s_break - ds;
ux_acc_straight2 = sqrt(v_xcl2^2 + 2*ax_max*s_acc_straight2);
s_dec_straight2 = 0:ds:(Lstraight-s_break);
ux_dec_straight2 = sqrt(v_xcl2^2 - 2*amax_brake*s_dec_straight2);
ux_prof_straight2 = [ux_acc_straight2 fliplr(ux_dec_straight2)];
ux_prof_straight2(1) = [];

ax_prof(s_xcl2_idx:1:s_xcl2_idx+s_break) = ax_max;
ax_prof(s_xcl2_idx+s_break:1:s_ecl3_idx) = amax_brake;

for i = 1:length(ux_prof_straight2) 
    
    vx_prof(s_xcl2_idx + i) = ux_prof_straight2(i);
    
end

% 3rd clothoid, modeled from code for 1st clothoid

clothoid3_indexes = s_xcl3_idx - s_ecl3_idx;

for i = 1:clothoid3_indexes
    
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl3_idx - i + 1)*vx_prof(s_xcl3_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl3_idx - i + 1);
    vx_prof(s_xcl3_idx - i) = vx_prof(s_xcl3_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl3_idx - i) = -vx_prof(s_xcl3_idx - i)*dux_ds;
end

% 2nd arc
for i = enterArc2:exitArc2
    vx_prof(i) = v_arc;
    ax_prof(i) = 0;
end

% 4th clothoid 

for i = s_ecl4_idx:s_xcl4_idx
    
    dux_ds = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2)...
        /vx_prof(i);
    vx_prof(i + 1) = vx_prof(i)+dux_ds*ds;
    ax_prof(i+1) = vx_prof(i)*dux_ds;
end

% third straight, insert braking built by Tatiana

v_xcl4 = vx_prof(s_xcl4_idx);
v_ecl5 = vx_prof(s_ecl5_idx);
s_break = 0;

while(true)
    LHS = 2*ax_max*s_break;
    RHS = -2*amax_brake*(Lstraight - s_break);
    if (LHS > RHS)
        break;
    end
    s_break = s_break + ds;
end

s_acc_straight3 = 0:ds:s_break - ds;
ux_acc_straight3 = sqrt(v_xcl2^2 + 2*ax_max*s_acc_straight3);
s_dec_straight3 = 0:ds:(Lstraight-s_break);
ux_dec_straight3 = sqrt(v_xcl2^2 - 2*amax_brake*s_dec_straight3);
ux_prof_straight3 = [ux_acc_straight2 fliplr(ux_dec_straight3)];
ux_prof_straight3(1) = [];


ax_prof(s_xcl4_idx:1:s_xcl4_idx+s_break) = ax_max;
ax_prof(s_xcl4_idx+s_break:1:s_ecl5_idx) = amax_brake;

for i = 1:length(ux_prof_straight3) 
    
    vx_prof(s_xcl4_idx + i) = ux_prof_straight3(i);
    
end

% 5th clothoid

clothoid5_indexes = s_xcl5_idx - s_ecl5_idx;

for i = 1:clothoid5_indexes
    
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl5_idx - i + 1)*vx_prof(s_xcl5_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl5_idx - i + 1);
    vx_prof(s_xcl5_idx - i) = vx_prof(s_xcl5_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl5_idx - i) = -vx_prof(s_xcl5_idx - i) * dux_ds;
end

% 3rd arc
for i = enterArc3:exitArc3
    vx_prof(i) = v_arc;
    ax_prof(i) = 0;
end

% 6th clothoid

for i = s_ecl6_idx:s_xcl6_idx
    
    dux_ds = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2)...
        /vx_prof(i);
    vx_prof(i + 1) = vx_prof(i)+dux_ds*ds;
    ax_prof(i+1) = vx_prof(i+1)*dux_ds;
end

% straight 4 

v_xcl6 = vx_prof(s_xcl6_idx);
v_ecl7 = vx_prof(s_ecl7_idx);
s_break = 0;

while(true)
    LHS = 2*ax_max*s_break;
    RHS = -2*amax_brake*(Lstraight - s_break);
    if (LHS > RHS)
        break;
    end
    s_break = s_break + ds;
end

s_acc_straight4 = 0:ds:s_break - ds;
ux_acc_straight4 = sqrt(v_xcl2^2 + 2*ax_max*s_acc_straight4);
s_dec_straight4 = 0:ds:(Lstraight-s_break);
ux_dec_straight4 = sqrt(v_xcl2^2 - 2*amax_brake*s_dec_straight4);
ux_prof_straight4 = [ux_acc_straight2 fliplr(ux_dec_straight4)];
ux_prof_straight4(1) = [];

ax_prof(s_xcl6_idx:1:s_xcl6_idx+s_break) = ax_max;
ax_prof(s_xcl6_idx+s_break:1:s_ecl7_idx) = amax_brake;


for i = 1:length(ux_prof_straight4) 
    
    vx_prof(s_xcl6_idx + i) = ux_prof_straight4(i);
    
end

% 7th clothoid

clothoid7_indexes = s_xcl7_idx - s_ecl7_idx;

for i = 1:clothoid7_indexes
    
    dux_ds = sqrt(amax^2 - (path.k_1pm(s_xcl7_idx - i + 1)*vx_prof(s_xcl7_idx - i + 1)^2)^2)...
        /vx_prof(s_xcl7_idx - i + 1);
    vx_prof(s_xcl7_idx - i) = vx_prof(s_xcl7_idx - i + 1) + dux_ds*ds; % + dux_ds... because decelerating through curve
    ax_prof(s_xcl7_idx - i) = -vx_prof(s_xcl7_idx - i)*dux_ds;
end

% 4th arc
for i = enterArc4:exitArc4
    vx_prof(i) = v_arc;
    ax_prof(i) = 0;
end

% 8th clothoid

for i = s_ecl8_idx:s_xcl8_idx
    
    dux_ds = sqrt(amax^2 - (path.k_1pm(i)*vx_prof(i)^2)^2)...
        /vx_prof(i);
    vx_prof(i + 1) = vx_prof(i)+dux_ds*ds;
    ax_prof(i+1) = vx_prof(i)*dux_ds;
end

% finish

v_straight_final = vx_prof(s_xcl8_idx);
s_desired_finish = path.s_m(length(path.s_m)) - path.s_m(enterLastStraight);
final_braking_offset = .1;
braking_slope = - (v_straight_final + final_braking_offset)/s_desired_finish;
ax_prof(s_xcl8_idx:finish) = -ax_max;

figure;
plot(ax_prof)
%keyboard;
for i = enterLastStraight:finish
    vx_prof(i) = vx_prof(i-1) + braking_slope*ds;
%     if (vx_prof(i) < 0)
%         vx_prof(i) = 0;
%         break;
%     end
        
end




% v_bound = 7.6;
% 
% % code to smooth out first straight into first clothoid
% for idx = 27:65
%     if (vx_prof(idx)>v_bound)
%         vx_prof(idx) = v_bound;
%     end
% end
vx_smooth = smoothdata(vx_prof, 'rloess','SmoothingFactor',smoothing_factor);
ax_smooth = smoothdata(ax_prof, 'rloess','SmoothingFactor',smoothing_factor_accel);
% % plots the velocity profile for the indexes we have so far
% vx_total = vx_prof(beginStraight1:finish);
% acc_total = diff(vx_total)./ds;
% last_val = acc_total(length(acc_total));
% acc_total = [acc_total' last_val];
% acc_total_smooth = smoothdata(acc_total, 'rloess','SmoothingFactor',smoothing_factor);
% acc_total_smooth = acc_total_smooth';
% vx_smooth = [];
% for i = 2:length(path.s_m)
%     to_add = trapz(path.s_m(1:i), acc_total_smooth(1:i));
%     vx_smooth = [vx_smooth, to_add];
% end


figure()
plot(path.s_m,vx_prof);
hold on;
plot(path.s_m,vx_smooth);
title('Velocity Profile');
figure()
plot(path.s_m,ax_prof);
hold on;
plot(path.s_m,ax_smooth);
title('Accleration Profile');
% figure;
% plot(path.s_m, acc_total);
% hold on;
% plot(path.s_m,acc_total_smooth);
% title('Acceleration Profile');


