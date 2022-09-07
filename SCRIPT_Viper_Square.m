%%  SCRIPT_Viper_KeyPress
%   Initialize robots and begin moving them at the same time. Objective is
%   to understand coordinate systems relative to eachother so that they can
%   coordinate trajectories simulataneously.
%
%   Harrison Helmich; 11Aug2022

clc;

addpath(fullfile('..', 'YaskawaCode'));
addpath(fullfile('..', 'URxCode'));

%rosinit;

% If not running MATLAB on the same Linux machine w/ ROS Master, do the
% following command instead
rosinit('10.0.0.31');

%%  Setup - UR10
ur_ip = '10.0.0.100';
ura = URx_ROS('/ura');

sim_ur10 = loadrobot("universalUR10");
sim_ur10.DataFormat = 'row';
sim_ur10.Gravity = [0 0 -9.81];

figure(1);
axs1 = axis;
show(sim_ur10, ura.jointAngles');
hold on;
grid on;
% robots are 4.85 meters apart

% Manually found adequate start position
% TODO - find start pos using task space and convert
station_ang_ur = [  pi;
                   -0.4091;
                    0.7938;
                   -3.5137;
                   -1.5473;
                   -0.0000  ];

station_task_ur = [-1.25; 0.0; 0.05]; 

%%  Setup - Yaskawa
ysk = Yaskawa_ROS;

sim_sia20 = importrobot(fullfile('..', 'urdf', 'sia20.urdf'));
sim_sia20.DataFormat = 'row';
sim_sia20.Gravity = [0 0 -9.81];

show(sim_sia20, ysk.jointAngles');
hold off;

% Manually found adequate start position
% TODO - find start pos using task space and convert
station_ang_ysk = [ 0.0365;
                    1.0592;
                    -0.0522;
                    0;
                    0;
                    -0.4816;
                    pi/2    ];

station_task_ysk = [0.8; 0.0; 0.85];

%%  Move to initial point
% Gotta send to safe point before starting taskPos method. That method
% depends on current state of robot.
ura.send_jointAngles(station_ang_ur);
ysk.send_jointAngles(station_ang_ysk);

%%  Adjust from joint space movements to task space movements
ura.send_taskPos(1, station_task_ur);
ysk.send_taskPos(1, station_task_ysk);

%%  Do a square
%   Direction: bl > br > ur > ul > bl

square_ur.corner(1:3, 1) = [-1.25;    -0.1;   0.1];
square_ur.corner(1:3, 2) = [-1.25;     0.1;   0.1];
square_ur.corner(1:3, 3) = [-1.25;     0.1;   0.3];
square_ur.corner(1:3, 4) = [-1.25;    -0.1;   0.3];
square_ur.corner(1:3, 5) = square_ur.corner(1:3, 1);

square_ysk.corner(1:3, 1) = [0.8;    -0.1;   0.65];
square_ysk.corner(1:3, 2) = [0.8;    0.1;    0.65];
square_ysk.corner(1:3, 3) = [0.8;    0.1;    0.85];
square_ysk.corner(1:3, 4) = [0.8;    -0.1;   0.85];
square_ysk.corner(1:3, 5) = square_ysk.corner(1:3, 1);

time = 1;
%   Before starting square sequence, previous section must be successfully
%   run to ensure smooth execution.
ura.send_taskPos(time, square_ur.corner(:, 1));
ysk.send_taskPos(time, square_ysk.corner(:, 1));
pause(2.0);



for i = 2:5
    ura.send_taskPos(time, square_ur.corner(:, i));
    ysk.send_taskPos(time, square_ysk.corner(:, i));
    
    pause(3.0);
end

%%  Send Home
ura.goHome;
ysk.goHome;

%%  Done
rosshutdown





