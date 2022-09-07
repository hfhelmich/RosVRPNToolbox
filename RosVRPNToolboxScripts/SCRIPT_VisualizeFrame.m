%%  SCRIPT_VisualizeFrame.m
% This script initializes a ROS conn
%
%
%
%   

%% Initialize ROS node
% NOTE: 10.0.0.31 corresponds to the linux machine running the ROS Master
%       publishing desired topics
%rosshutdown
rosinit('10.0.0.31');

%% Initialize UR connection
%ura = URx_ROS('/ura');

%% Initialize OptiTrack rigid bodies
v.Camera         = VRPN_ROS('Camera_DFK');
v.Checkerboard   = VRPN_ROS('Checkerboard');
v.Ur             = VRPN_ROS('UR10_c');
v.Ysk            = VRPN_ROS('Yaskawa_o');
v.RedSquare      = VRPN_ROS('RedSquare');

%% Visualize OptiTrack rigid bodies
fig = figure('Name','VRPN Visualization [CLOSE TO EXIT]');
axs = axes('Parent',fig,'NextPlot','add','DataAspectRatio',[1 1 1],'View',...
    [-65,25]);
xlabel(axs,'x (m)');
ylabel(axs,'y (m)');
zlabel(axs,'z (m)');

% Get rigid body fields
flds = fieldnames(v);

xyzStr = 'xyz';
nRigidBodies = numel(flds);

% Create OptiTrack world frame
for j = 1:numel(xyzStr)
    lbls{j} = sprintf('%s_{%s}',xyzStr(j),'world');
end
h_w2a = triad('Parent',axs,'AxisLabels',lbls,'LineWidth',1.0,...
        'Matrix',Rx(pi/2));

% Initialize rigid bodies
for i = 1:nRigidBodies
    for j = 1:numel(xyzStr)
        lbls{j} = sprintf('%s_{%s}',xyzStr(j),flds{i});
    end
    h_i2w(i) = triad('Parent',h_w2a,'AxisLabels',lbls,'LineWidth',1.0,...
        'Scale',0.5);
end

% Visualize frames
while ishandle(fig)
    for i = 1:nRigidBodies
        H_i2w = v.(flds{i}).pose;
        if ishandle(h_i2w(i))
            set(h_i2w(i),'Matrix',H_i2w);
        else
            break
        end
    end
    drawnow;
end
