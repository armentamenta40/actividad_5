
%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1; % Sample time [s]
tVec = 0:sampleTime:282 % Time array
initPose = [1;9;pi/4]; % Initial pose (x y theta)
pose = zeros(3,numel(tVec)); % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [1,9; 3,9; 3,10; 6,11; 5,12; 5,10; 4,9; 4,10; 5,10; 5,10.73; 6,11; 7,11; 7,10; 7,12; 8,9; 7,8; 8,9; 7,12; 9,8; 11,6; 6,4; 6,5; 10,7; 11,6; 12,6; 12,0; 10,0; 10,1; 10,0; 7,0; 6,2; 6,5; 5,6; 2,6; 3,7; 4,7; 3,7; 2,6; 1,8; 1,9; 2,9; 1,8; ];
% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.1;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 25;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
end