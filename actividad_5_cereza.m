
%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:96;         % Time array

initPose = [5;1;3/4*pi];             % Initial pose (x y theta)
pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
waypoints = [4.8,1.2; 4,2; 3,3; 3,4; 3,5; 4,6; 5,7; 6,7; 7,7; 8,6; 7,5; 6,6; 7,5; 8,5; 7,5; 8,6; 8.5,7; 9.2,8; 10,9; 9,10; 8,11; 7,11; 6,11; 5,10; 4,9; 5,9; 6,9; 7,9; 8,9; 9,9; 10,9; 9.2,8; 8.5,7; 8,6; 9,5; 9,4; 9,3; 8,2; 7,1; 6,1; 5.5,1];

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.45;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 20.5;

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