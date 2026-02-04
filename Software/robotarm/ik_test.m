% File to get to know the Inverse Kinematics function
%Author: Matteo Gassler
%03.02.2026

clear; close all; clc;

% Load robot
urdfFile = 'C:\Users\matte\Documents\Open-robot-arm\Software\robotarm\urdf\robot_armV2.SLDASM.urdf';
try
    meshPath = 'C:\Users\matte\Documents\Open-robot-arm\Software\robotarm\meshes';
catch ME
    fprintf("Error, loaded without meshes")
    disp(ME.identifier)
end

robot = importrobot(urdfFile, 'MeshPath', meshPath);
fprintf('Robot loaded: %s\n\n', robot.BaseName);

% Create IK solver 
ik = inverseKinematics('RigidBodyTree', robot);

% Define target position
targetX = 0.4;
targetY = 0.3;
targetZ = 0.1;

fprintf('Target position: [%.3f, %.3f, %.3f] m\n', targetX, targetY, targetZ);

% Create target pose turns the vector into a goal matrix
targetPose = trvec2tform([targetX, targetY, targetZ]);

% Set priority, first 3 position, secound 3 orientatien(position is more important than orientation)
weights = [1, 1, 1, 0.1, 0.1, 0.1];

% Initial guess
initialGuess = homeConfiguration(robot);

endEffectorName = robot.Bodies{end}.Name;

% Solve IK 
[configSolution, solnInfo] = ik(endEffectorName, targetPose, weights, initialGuess);

% Check result
if strcmp(solnInfo.Status, 'success')
    fprintf('✓ IK solution found!\n\n');
    
    % Print joint angles
    fprintf('Joint Angles:\n');
    for i = 1:numel(configSolution)
        fprintf('  Joint %d: %.4f rad (%.2f°)\n', ...
            i, configSolution(i).JointPosition, rad2deg(configSolution(i).JointPosition));
    end
    fprintf('\n');
    
    % Verify with FK (built-in)
    transform = getTransform(robot, configSolution, endEffectorName);
    actualPosition = tform2trvec(transform);
    
    fprintf('Verification (Forward Kinematics):\n');
    fprintf('  Target:  [%.4f, %.4f, %.4f] m\n', targetX, targetY, targetZ);
    fprintf('  Actual:  [%.4f, %.4f, %.4f] m\n', actualPosition(1), actualPosition(2), actualPosition(3));
    fprintf('  Error:   [%.4f, %.4f, %.4f] m\n\n', ...
        abs(targetX-actualPosition(1)), abs(targetY-actualPosition(2)), abs(targetZ-actualPosition(3)));
    
    % Visualize
    figure;
    show(robot, configSolution);
    title(sprintf('Robot reaching [%.2f, %.2f, %.2f]', targetX, targetY, targetZ));
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    grid on;
    view(45, 30);
    
else
    fprintf('✗ IK solution NOT found!\n');
    fprintf('Target may be out of reach.\n');
end