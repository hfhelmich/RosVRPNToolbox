%% SCRIPT_UR_ValidatePose
% This script uses data correspondence between joint angles and pose
% returned by the UR ROS interface. Data should match pose estimates from
% forward kinematics to within several millimeters
%
%   Input(s)
%       data - n-element structured array containing the following fields:
%           data(i).camPose     - [UNUSED] 4x4 array element of SE(3)  
%                                 defining the pose of the camera MoCap 
%                                 frame relative to the MoCap world frame
%                                 with linear units of meters.
%           data(i).cartPose    - [UNUSED] 4x4 array element of SE(3)  
%                                 defining the pose of the UR cart MoCap 
%                                 frame relative to the MoCap world frame
%                                 with linear units of meters.
%           data(i).urPose      - 4x4 array element of SE(3) defining the 
%                                 UR end-effector pose relative to the base
%                                 frame with linear units of millimeters.  
%                                 This data is returned by the UR ROS 
%                                 interface.
%           data(i).jointAngles - 6x1 array containing UR joint angles in
%                                 radians. This dtaa is returned by the UR
%                                 ROS interface.
%           
%   M. Kutzer, 01Sep2022, USNA

clear all
close all
clc

%% Load data set
% TODO - replace with UI load
load('calibration1_083122.mat');

%% Create simulation for visualizing results
% NOTE: URsim is a visualization with linear units of millimeters
sim = URsim;
sim.Initialize('UR10');

%% Hide frames
for i = 0:6
    frm = sprintf('hFrame%d',i);
    hideTriad( sim.(frm) );
end
hideTriad( sim.hFrameT );

%% Adjust axis limits to account for pedestal
zlim(sim.Axes,[-1000,1400]); 

%% Create frame visualization
h_e2o = triad('Parent',sim.Axes,'Scale',300,'LineWidth',1);

%% Visualize
for i = 1:numel(data)
    % Move simulation
    sim.Joints = data(i).jointAngles;
    
    % Get pose
    H_e2o = data(i).urPose;

    % Update pose
    set(h_e2o,'Matrix',H_e2o);
    
    % Compare values
    H_e2o_fkin = sim.Pose;

    H_eRos2eFkin = invSE(H_e2o)*H_e2o_fkin;
    fprintf('%03d H_eRos2eFkin = \n',i);
    disp(H_eRos2eFkin);

    drawnow

    pause;
end