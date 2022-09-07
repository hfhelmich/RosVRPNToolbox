%% SCRIPT_UR_CompareMoCap
% This script uses data correspondence between joint pose returned by the 
% UR ROS interface and data returned by the MoCap to compare relative
% movements.
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
load('calibration1_090122_1.mat');

%% Establish repackage data to establish frame labels and common units
% Frame e - UR end-effector
% Frame o - UR base frame
% Frame c - MoCap camera frame rigidly fixed to the UR end-effector
% Frame b - MoCap cart frame rigidly fixed to the UR base frame
% Frame w - MoCap world frame
for i = 1:numel(data)
    % UR end-effector frame pose relative to base frame (mm)
    H_e2o{i} = data(i).urPose;
    % MoCap camera frame pose relative to MoCap world frame (m)
    H_c2w{i} = data(i).camPose;
    % MoCap cart frame pose relative to MoCap world frame (m)
    H_b2w{i} = data(i).cartPose;

    % Convert units (m to mm)
    d_c2w = H_c2w{i}(1:3,4);
    d_c2w = d_c2w*1000;
    H_c2w{i}(1:3,4) = d_c2w;

    d_b2w = H_b2w{i}(1:3,4);
    d_b2w = d_b2w*1000;
    H_b2w{i}(1:3,4) = d_b2w;

    % Calculate relative transform
    H_c2b{i} = invSE(H_b2w{i}) * H_c2w{i};
end

%% Create simulation for visualizing robot movements
% NOTE: URsim is a visualization with linear units of millimeters
sim = URsim;
sim.Initialize('UR10');

% Hide frames
for i = 0:6
    frm = sprintf('hFrame%d',i);
    hideTriad( sim.(frm) );
end
hideTriad( sim.hFrameT );

% Adjust axis limits to account for pedestal
zlim(sim.Axes,[-1000,1400]); 

% Create frame visualization
h_e2o = triad('Parent',sim.Axes,'Scale',300,'LineWidth',1);

%% Create simulation for visualizing MoCap movements
fig = figure('Name','MoCap Frames');
axs = axes('Parent',fig,'NextPlot','add','DataAspectRatio',[1 1 1]);

% Match view and axis limits to UR simulation
[az,el] = view(sim.Axes);
lims = axis(sim.Axes);
view(axs,[az,el]);
axis(axs,lims);

% Create frame visualizations
% h_w2a = triad('Parent',axs,'Scale',150,'LineWidth',1,...
%     'AxisLabels',{'x_w','y_w','z_w'});
% h_b2w = triad('Parent',h_w2a,'Scale',150,'LineWidth',1,...
%     'AxisLabels',{'x_b','y_b','z_b'});
% h_c2w = triad('Parent',h_w2a,'Scale',150,'LineWidth',1,...
%     'AxisLabels',{'x_c','y_c','z_c'});
h_b2a = triad('Parent',axs,'Scale',150,'LineWidth',1,...
    'AxisLabels',{'x_b','y_b','z_b'});
h_c2b = triad('Parent',h_b2a,'Scale',150,'LineWidth',1,...
    'AxisLabels',{'x_c','y_c','z_c'});

%% Visualize
set(h_b2a,'Matrix',Rx(pi/2));
for i = 1:numel(data)
    % Move simulation
    sim.Joints = data(i).jointAngles;

    % Update frame poses
    set(h_e2o,'Matrix',H_e2o{i});
    %set(h_b2w,'Matrix',H_b2w{i});
    %set(h_c2w,'Matrix',H_c2w{i});
    set(h_c2b,'Matrix',H_c2b{i});

    drawnow

    pause;
end