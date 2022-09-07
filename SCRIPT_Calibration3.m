%%  SCRIPT_Calibration3
%   Taking photos of UR with OptiTrack and gathering the following data:
%       - OptiTrack pose of camera
%       - OptiTrack pose of UR10 cart
%       - Joint angles of UR10
%       - Actual pose of UR10 end-effector
%   Objectives:
%       - Find
%
%   Harrison Helmich; 6 Sep 2022

imaqreset;
[cam, prv, handles] = initCamera;

%%  Initialize
%   ROS must be up and running the viper system on the linux machine
%   (10.0.0.31)
rosinit('10.0.0.31');
pause(1);

ura         = URx_ROS('');
v.Camera    = VRPN_ROS('Camera_DFK');
v.Ur        = VRPN_ROS('UR10_c');
v.Ysk       = VRPN_ROS('Yaskawa_o');
v.Checkerboard = VRPN_ROS('Checkerboard');

%%

imBaseName = 'checker';
calFolderName = 'checkerImages_StillUR_09SEP22';

% Create the calibration folder
if ~isfolder(calFolderName)
    mkdir(calFolderName);
end

data(1).camPose(1:4, 1:4)       = v.Camera.pose;
data(1).cartPose(1:4, 1:4)      = v.Ur.pose;
data(1).yskPose(1:4, 1:4)       = v.Ysk.pose;
data(1).checkPose(1:4, 1:4)     = v.Checkerboard.pose;
data(1).urPose(1:4, 1:4)        = ura.HTform;
data(1).jointAngles(1:6, 1)     = ura.jointAngles;

% ura.send_jointAngles(q(:,1));
fprintf("Starting in 10 seconds");
pause(10);

n = 30;

for i = 1:n
    
    drawnow;
    
    % Define filename of image
    fname = sprintf('%s_%03d.png',imBaseName,i);
    % Bring preview to foreground
    figure(handles.Figure);
    % Get the image from the preview
    im = get(prv,'CData');
    % Save the image
    imwrite(im,fullfile(calFolderName,fname),'png');

    data(i).camPose(1:4, 1:4)       = v.Camera.pose;
    data(i).cartPose(1:4, 1:4)      = v.Ur.pose;
    data(i).yskPose(1:4, 1:4)       = v.Ysk.pose;
    data(i).checkPose(1:4, 1:4)     = v.Checkerboard.pose;
    data(i).urPose(1:4, 1:4)        = ura.HTform;
    data(i).jointAngles(1:6, 1)     = ura.jointAngles;

    v.Checkerboard.pose
    i

    for j = 3:-1:1
        fprintf("Countdown: %d\n", j);
        pause(1);
    end
end

%%
save(fullfile(calFolderName), "data");