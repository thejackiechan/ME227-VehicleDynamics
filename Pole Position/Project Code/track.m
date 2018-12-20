%% Path Velocity and Acceleration Profiles
clear;
clc;
close all;

%% Create the vectors to store velocity and acceleration profiles
load('project_data.mat');
vx_prof = zeros(length(path.s_m),1);
ax_prof = zeros(length(path.s_m),1);

%% 0 Find indexes and distance along path to first clothoid
    [indices, curvatures] = find_indices(path);
    % Smooth the velocity profile
    % Obtain the acceleration profile

%% 1 Solve for the velocity, acceleration profile through first straight
    % Smooth the velocity profile
    % Obtain the acceleration profile

%% 2 Define the velocity and acceleration profile from the first arc
    % Smooth the velocity profile
    % Obtain the acceleration profile

%% 3 Solve for the velocity and acceleration profile exiting the second clothoid
    % Smooth the velocity profile
    % Obtain the acceleration profile

%% 4 Solve for the velocity and acceleration profile along the second straight
    % Smooth the velocity profile
    % Obtain the acceleration profile
