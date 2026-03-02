%program to get the joint angles for every waypoint in the motion planning
%operation
%Author: Matteo Gassler
%02.03.2026

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
weights = [1, 1, 1, 1, 1, 1];
initialGuess = homeConfiguration(robot);

% Define 5 target positions (waypoints)
waypoints = [
0.0414, 0.3578, 0.3602;	%home position
0.2000, 0.4000, 0.11000;	%Above pick
0.2000, 0.4000, 0.0600;		%pick position
0.0414,0.2578,0.2502;		%between pick and place
-0.2000, 0.4000, 0.11000;	%Above place
-0.2000, 0.4000, 0.0600;	%plave position
0.0414, 0.3578, 0.3602;		%back to home
];

% Calculate IK for each waypoint
numWaypoints = size(waypoints, 1);
jointAngles = zeros(numWaypoints, 6);  % 6 joints

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
%save('trajectory_waypoints.mat', 'jointAngles', 'waypoints');
%fprintf('✓ Saved to trajectory_waypoints.mat\n');