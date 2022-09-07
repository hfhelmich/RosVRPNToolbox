%%  SCRIPT_MultiRobot_FirstTest.m
%   
%   Get the Yaskawa SIA20F and UR10 moving at the same time. Use one file
%   for both.
%
%   Harrison Helmich; 28 July 2022
%

% Must be in MultiRobotCode Folder
addpath(fullfile('..', 'YaskawaCode'));
addpath(fullfile('..', 'URxCode'));

rosinit;

%%  Setup - OptiTrack
opti_sub_RedSquare    = rossubscriber('/vrpn_client_node/RedSquare/pose', 'geometry_msgs/PoseStamped','DataFormat','struct');
opti_sub_UR         = rossubscriber('/vrpn_client_node/RigidBodyUR/pose', 'geometry_msgs/PoseStamped','DataFormat','struct');
% opti_sub_Ysk        = rossubscriber('/vrpn_client_node/RigidBodyYsk/pose', 'geometry_msgs/PoseStamped','DataFormat','struct');

pause(1);

pose_RedSquare  = opti_sub_RedSquare.LatestMessage.Pose;
pose_UR         = opti_sub_UR.LatestMessage.Pose;
% pose_Ysk = opti_sub_Ysk.LatestMessage.Pose;

figure(1);
axs1 = axes;
hold on;
grid on;
plot3(pose_RedSquare.Position.Z, pose_RedSquare.Position.X, pose_RedSquare.Position.Y,'*r');
plot3(pose_UR.Position.Z, pose_UR.Position.X, pose_UR.Position.Y,'*g');
plot3(0, 0, 0, 'or');
xlim([-3 6]);
ylim([-2 7]);
zlim([-0.5 5]);

%%  Setup - UR10
ur_ip = '10.0.0.100';
ura = URx_ROS('/ura');
urMod = 'UR10';

%%  Setup - Yaskawa
ysk = Yaskawa('/ysk');

%%


