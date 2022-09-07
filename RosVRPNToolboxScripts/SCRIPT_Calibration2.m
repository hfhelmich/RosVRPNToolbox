%%  SCRIPT_Calibration2
%   Taking photos of April Tag on Yaskawa with camera on UR10 end-effector.
%
%   Objectives:
%       - Find...
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

%%
imBaseName = 'aprilView';
calFolderName = 'aprilTagImages_09SEP22';

% Create the calibration folder
if ~isfolder(calFolderName)
    mkdir(calFolderName);
end

% Initialize joint positions
n = 15;

for i = 1:n
    % Define filename of image
    fname = sprintf('%s_%03d.png',imBaseName,i);
    % Bring preview to foreground
    figure(handles.Figure);
    % Prompt user to move arm
    msg = sprintf(['Use the "Teach" button to move the arm to a new ',...
        'configuration with the checkerboard fully in the camera FOV. ',...
        'Taking Image %d of %d.'],i,n);
    f = msgbox(msg,'Position for Image');
    uiwait(f);

    % Get the image from the preview
    im = get(prv,'CData');

    % Save the image
    imwrite(im,fullfile(calFolderName,fname),'png');
end

% Define the filename for the robot data
save(fullfile(calFolderName),...
    'calFolderName','imBaseName');

%% Apriltag pose finder
% optional - check to see if there is more than 1 apriltag and that the
% apriltag is number 20
for i = 1:n
    img = imread(['aprilTagImages_09SEP22\aprilView_', num2str(i,'%03.f'), '.png']);
    intrinsics = params.Intrinsics;
    tagSize = 235; %in mm
    img = undistortImage(img,intrinsics,"OutputView","same");
    [id,loc,pose] = readAprilTag(img,"tag36h11",intrinsics,tagSize);
    worldPoints = [0 0 0; tagSize/2 0 0; 0 tagSize/2 0; 0 0 tagSize/2];

    % Get image coordinates for axes.
    imagePoints = worldToImage(intrinsics,pose(1).Rotation, ...
        pose(1).Translation,worldPoints);

    % Draw colored axes.
    img = insertShape(img,"Line",[imagePoints(1,:) imagePoints(2,:); ...
        imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
        "Color",["red","green","blue"],"LineWidth",7);

    img = insertText(img,loc(1,:,1),id(1),"BoxOpacity",1,"FontSize",25);
    figure()
    imshow(img)

    % Get pose information
    H_B2c{i} = [pose.Rotation' pose.Translation'; 0 0 0 1];
end
