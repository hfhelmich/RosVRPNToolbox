%% SCRIPT_UR_CoCalibrateMoCap
% This script uses data correspondence between joint pose returned by the 
% UR ROS interface and data returned by the MoCap to establish the fixed 
% transformations relating the MoCap frames to the UR base and end-effector
% frames.
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
load('calibration1_090122_2.mat');

%% Create simulation for visualizing results
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

% Create frame visualizations
h_w2a = triad('Parent',sim.Axes,'Scale',150,'LineWidth',1,...
    'AxisLabels',{'x_w','y_w','z_w'});
h_b2w = triad('Parent',h_w2a,'Scale',150,'LineWidth',1,...
    'AxisLabels',{'x_b','y_b','z_b'});
h_c2w = triad('Parent',h_w2a,'Scale',150,'LineWidth',1,...
    'AxisLabels',{'x_c','y_c','z_c'});

% Set fix UR base frame to frame b
h_o2b = sim.hFrame0;
set(h_o2b,'Parent',h_b2w);

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
end

%% Solve for H_o2b using AX = XB
% For an explanation of the method, see "EW450 Lectures\13 - Robot-Camera
% Calibration\Robot-Camera Calibration - Definition & Use.pptx"
%
%      A       X    =    X       B
%   H_bi2bj H_oi2bi = H_oj2bj H_oi2oj
%
%   Where H_oi2bi = H_oj2bj for all i,j

% Define total number of correspondence pairs
n = numel(data);
% Initialize parameters
iter = 0;
A = {};
B = {};

% Define i/j pairs
for i = 1:n
    for j = 1:n
        % H_bi2bj
        H_bi2w = H_b2w{i};
        H_bj2w = H_b2w{j};
        H_bi2bj{i,j} = invSE(H_bj2w) * H_bi2w;

        % H_oi2oj
        H_e2oi = H_e2o{i};
        H_e2oj = H_e2o{j};
        H_oi2oj{i,j} = H_e2oj * invSE(H_e2oi);

        % Isolate unique, non-identity transformation pairs
        %   We are keeping the upper-triangular portion of H_bi2bj and
        %   H_oi2oj to avoid values equal to the identity, and values that
        %   are the inverse of others.
        if i ~= j && i < j
            iter = iter+1;
            A{iter} = H_bi2bj{i,j};
            B{iter} = H_oi2oj{i,j};
        end
    end
end
fprintf('Number of A/B pairs: %d\n',iter);

% Solve A * X = X * B
X = solveAXeqXBinSE(A,B);
H_o2b = X;
[msg,tf] = isSE(H_o2b);
if ~tf
    fprintf('Value calculated for H_o2b is not a valid element of SE(3):\n\n%s\tReplacing H_o2b with nearest element of SE(3).');
    H_o2b = nearestSE(H_o2b);
end

%% Solve for H_c2e using AX = XB
% For an explanation of the method, see "EW450 Lectures\13 - Robot-Camera
% Calibration\Robot-Camera Calibration - Definition & Use.pptx"
%
%      A       X    =    X       B
%   H_ei2ej H_ci2ei = H_cj2ej H_ci2cj
%
%   Where H_ci2ei = H_cj2ej for all i,j

% Define total number of correspondence pairs
n = numel(data);
% Initialize parameters
iter = 0;
A = {};
B = {};

% Define i/j pairs
for i = 1:n
    for j = 1:n
        % H_bi2bj
        H_ei2o = H_e2o{i};
        H_ej2o = H_e2o{j};
        H_ei2ej{i,j} = invSE(H_ej2o) * H_ei2o;

        % H_oi2oj
        H_ci2w = H_c2w{i};
        H_cj2w = H_c2w{j};
        H_ci2cj{i,j} = invSE(H_cj2w) * H_ci2w;

        % Isolate unique, non-identity transformation pairs
        %   We are keeping the upper-triangular portion of H_bi2bj and
        %   H_oi2oj to avoid values equal to the identity, and values that
        %   are the inverse of others.
        if i ~= j && i < j
            iter = iter+1;
            A{iter} = H_ei2ej{i,j};
            B{iter} = H_ci2cj{i,j};
        end
    end
end
fprintf('Number of A/B pairs: %d\n',iter);

% Solve A * X = X * B
X = solveAXeqXBinSE(A,B);
H_c2e = X;
[msg,tf] = isSE(H_c2e);
if ~tf
    fprintf('Value calculated for H_o2b is not a valid element of SE(3):\n\n%s\tReplacing H_o2b with nearest element of SE(3).');
    H_c2e = nearestSE(H_c2e);
end

%% Visualize
axis(sim.Axes,'tight');
set(h_w2a,'Matrix',Rx(pi/2));
set(h_o2b,'Matrix',H_o2b);
grid on;

for i = 1:numel(data)
    % Move simulation
    sim.Joints = data(i).jointAngles;

    % Update pose
    set(h_b2w,'Matrix',H_b2w{i});
    set(h_c2w,'Matrix',H_c2w{i});
    
    drawnow

    pause;
end