%% Calculate Initial Guess for Simulink Model
% Run this ONCE before using your Simulink model

clear; clc;


%% 1. Load robot
urdfFile = 'C:\Users\matte\Documents\Open-robot-arm\Software\robotarm\urdf\robot_armV2SLDASM.urdf';
meshPath = 'C:\Users\matte\Documents\Open-robot-arm\Software\robotarm\meshes';

fprintf('Loading robot from URDF...\n');
robot = importrobot(urdfFile, 'MeshPath', meshPath);
fprintf('✓ Robot loaded: %d joints\n\n', numel(robot.Bodies));



