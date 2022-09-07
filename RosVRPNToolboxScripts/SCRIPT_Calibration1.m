%%  SCRIPT_Calibration1
%   Taking photos of UR with OptiTrack and gathering the following data:
%       - OptiTrack pose of camera
%       - OptiTrack pose of UR10 cart
%       - Joint angles of UR10
%       - Actual pose of UR10 end-effector
%   Objectives:
%       - Find correspondence between OptiTrack UR cart frame to the actual
%       base frame of the UR10
%       - Find correspondence between OptiTrack UR camera frame to the
%       actual end-effector frame of the UR10
%
%   Harrison Helmich; 31 Aug 2022

% temp = load("calibration1_083122.mat");
% for i = 1:length(temp.data)
%     q(1:6, i) = temp.data(i).jointAngles;
% end

%%  Initialize
%   ROS must be up and running the viper system on the linux machine
%   (10.0.0.31)
rosinit('10.0.0.31');
pause(1);

ura         = URx_ROS('');
v.Camera    = VRPN_ROS('Camera_DFK');
v.Ur        = VRPN_ROS('UR10_c');

% Some issue with the /tf topic in that there's no robot namespace before
% it. When UR10 and Yaskawa are running together, there is only one /tf
% topic. ONLY RUN THE UR10 FOR THIS CALIBRATION.
%poseSub     = rossubscriber('/tf');

%%

data(1).camPose(1:4, 1:4)       = v.Camera.pose;
data(1).camQuaternion(1:4, 1)   = v.Camera.quaternion;
data(1).camPosition(1:3, 1)     = v.Camera.position;
data(1).cartPose(1:4, 1:4)      = v.Ur.pose;
data(1).cartQuaternion(1:4, 1)  = v.Ur.quaternion;
data(1).cartPosition(1:3, 1)    = v.Ur.position;
data(1).urPose(1:4, 1:4)        = ura.HTform;
data(1).jointAngles(1:6, 1)     = ura.jointAngles;

% ura.send_jointAngles(q(:,1));
fprintf("Starting in 10 seconds");
pause(10);

n = 20;

for i =1:n
    % ura.send_jointAngles(q(:,i));
    % pause(10);

    data(i).camPose(1:4, 1:4)       = v.Camera.pose;
    data(i).camQuaternion(1:4, 1)   = v.Camera.quaternion;
    data(i).camPosition(1:3, 1)     = v.Camera.position;
    data(i).cartPose(1:4, 1:4)      = v.Ur.pose;
    data(i).cartQuaternion(1:4, 1)  = v.Ur.quaternion;
    data(i).cartPosition(1:3, 1)    = v.Ur.position;
    data(i).urPose(1:4, 1:4)        = ura.HTform;
    data(i).jointAngles(1:6, 1)     = ura.jointAngles;

    v.Camera.pose
    v.Ur.pose
    ura.HTform

    for j = 10:-1:1
        fprintf("Countdown: %d\n", j);
        pause(1);
    end

end