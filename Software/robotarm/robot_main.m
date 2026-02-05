%first small algorithm to solve very ptimitive motion planning
%Author: Matteo Gassler
%05.02.2026

clear; close all; clc;

% where the urdf and meshes are located, change to your location
   urdfFile = 'C:\Users\matte\Documents\Open-robot-arm\Software\robotarm\urdf\robot_armV2SLDASM.urdf';
try
    meshPath = 'C:\Users\matte\Documents\Open-robot-arm\Software\robotarm\meshes';
catch ME
    fprintf("Error:")
    disp(ME.identifier)
end

robot = importrobot(urdfFile, 'MeshPath', meshPath);
homeConfig = homeConfiguration(robot);

% Create IK solver
ik = inverseKinematics('RigidBodyTree', robot);
endEffectorName = robot.Bodies{end}.Name;
weights = [1, 1, 1, 0.1, 0.1, 0.1];
initialGuess = homeConfiguration(robot);

% Define 5 target positions (waypoints)
waypoints = [
    0.3,  0.0,  0.3;   % Position 1
    0.4,  0.1,  0.35;  % Position 2
    0.4, -0.1,  0.35;  % Position 3
    0.3,  0.0,  0.4;   % Position 4
    0.3,  0.0,  0.3;   % Position 5 (back to start)
];

% Calculate IK for each waypoint
numWaypoints = size(waypoints, 1);
jointAngles = zeros(numWaypoints, 12);  % 6 joints

fprintf('Calculating IK for %d waypoints...\n\n', numWaypoints);

for i = 1:numWaypoints
    targetPose = trvec2tform(waypoints(i, :));
    [configSolution, solnInfo] = ik(endEffectorName, targetPose, weights, initialGuess);
    
    if strcmp(solnInfo.Status, 'success')
        jointAngles(i, :) = [configSolution.JointPosition];
        initialGuess = configSolution;  % Use as next initial guess
        fprintf('✓ Waypoint %d: [%.3f, %.3f, %.3f]\n', i, waypoints(i, :));
    else
        fprintf('✗ Waypoint %d FAILED\n', i);
    end
end

fprintf('\n✓ All waypoints calculated!\n\n');

% Display joint angles
fprintf('Joint Angles (radians):\n');
disp(jointAngles);

% Save for Simulink
save('trajectory_waypoints.mat', 'jointAngles', 'waypoints');
fprintf('✓ Saved to trajectory_waypoints.mat\n');