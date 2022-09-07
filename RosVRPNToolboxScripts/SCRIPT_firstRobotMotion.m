%%  SCRIPT_firstRobotMotion
%   Initialize robots and begin moving them at the same time. Objective is
%   to understand coordinate systems relative to eachother so that they can
%   coordinate trajectories simulataneously.
%
%   Harrison Helmich; 11Aug2022

clc;

addpath(fullfile('..', 'YaskawaCode'));
addpath(fullfile('..', 'URxCode'));

rosinit;

%%  Setup - UR10
ur_ip = '10.0.0.100';
ura = URx_ROS('/ura');

H_e2o = [   1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

ur10 = loadrobot("universalUR10");
ur10.DataFormat = 'row';
ur10.Gravity = [0 0 -9.81];

figure(1);
axs1 = axis;
show(ur10, ura.jointAngles');
hold on;
grid on;
% robots are 4.85 meters apart

%%  Setup - Yaskawa
ysk = Yaskawa_ROS;

sia20 = importrobot(fullfile('/home/universal_robots/USNA_YASKAWA_ROS_MATLAB/MATLAB_ROS_Interface/urdf', 'sia20.urdf'));
sia20.DataFormat = 'row';
sia20.Gravity = [0 0 -9.81];

show(sia20, ysk.jointAngles');


%%

