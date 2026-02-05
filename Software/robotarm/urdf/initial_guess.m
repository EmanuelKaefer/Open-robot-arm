%% Calculate Initial Guess for Simulink Model
% Run this ONCE before using your Simulink model

clear; clc;


%% 1. Load robot
urdfFile = 'C:\Users\matte\Documents\Open-robot-arm\Software\robotarm\urdf\robot_armV2SLDASM.urdf';
meshPath = 'C:\Users\matte\Documents\Open-robot-arm\Software\robotarm\meshes';

fprintf('Loading robot from URDF...\n');
robot = importrobot(urdfFile, 'MeshPath', meshPath);
fprintf('✓ Robot loaded: %d joints\n\n', numel(robot.Bodies));

%% 2. Calculate home configuration
homeConfig = homeConfiguration(robot);

%% 3. Extract joint angles
numJoints = length(homeConfig);
initialGuess = zeros(numJoints, 1);

fprintf('Home configuration:\n');
for i = 1:numJoints
    initialGuess(i) = homeConfig(i).JointPosition;
    fprintf('  Joint %2d: %8.4f rad (%6.2f°)\n', ...
        i, initialGuess(i), rad2deg(initialGuess(i)));
end
fprintf('\n');

%% 4. Save to workspace
assignin('base', 'initialGuess', initialGuess);
fprintf('✓ Variable "initialGuess" saved to workspace\n');

%% 5. Save to file (for future use)
save('robotInitialGuess.mat', 'initialGuess', 'numJoints');
fprintf('✓ Saved to file: robotInitialGuess.mat\n\n');

%% 6. Create formatted output for manual entry (if needed)
fprintf('=== For Manual Entry ===\n');
fprintf('If you need to hardcode the value, use:\n\n');
fprintf('initialGuess = [');
for i = 1:numJoints
    fprintf('%.6f', initialGuess(i));
    if i < numJoints
        fprintf('; ');
    else
        fprintf('];\n\n');
    end
end

%% 7. Instructions
fprintf('=== NEXT STEPS ===\n');
fprintf('1. Open your Simulink model\n');
fprintf('2. Use a CONSTANT block (not MATLAB Function!)\n');
fprintf('3. Set Constant value to: initialGuess\n');
fprintf('4. This will work for code generation!\n');
fprintf('==================\n');