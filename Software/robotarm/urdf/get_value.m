%program to get the joint angles for every waypoint in the motion planning
%operation
%Author: Matteo Gassler
%02.03.2026

%clear; close all; clc;

% Extract from out object
tout    = out.tout;
q_raw   = out.q_logged;

% Reshape from flat vector to [Nsamples x 6]
num_joints  = 6;
num_samples = length(tout);
q_logged    = reshape(q_raw, num_joints, num_samples)';

fprintf('q_logged size: %d samples x %d joints\n\n', size(q_logged,1), size(q_logged,2));

% Plot joint angles so you can visually read the waypoint times
figure;
plot(tout, rad2deg(q_logged), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Joint Angle (deg)');
legend('q1','q2','q3','q4','q5','q6', 'Location', 'best');
title('Joint Angles over Trajectory — find the flat regions');
grid on;

fprintf('Look at the plot and find the 7 flat regions (= robot at waypoint).\n');
fprintf('Then update waypoint_times below and run the script again.\n\n');

% =========================================================
% ↓↓↓ UPDATE THESE after looking at the plot ↓↓↓
waypoint_times = [0, 3, 6, 9, 12, 15, 18];  % ← adjust to your flat regions
% ↑↑↑ UPDATE THESE after looking at the plot ↑↑↑
% =========================================================

% Extract joint angles at each waypoint time
dt  = tout(2) - tout(1);
idx = round(waypoint_times / dt) + 1;
idx = min(idx, num_samples);  % clamp to valid range

joint_waypoints = q_logged(idx, :);

% Print for verification
fprintf('Extracted joint waypoints (degrees):\n\n');
labels = {'home', 'above pick', 'pick', 'transit', 'above place', 'place', 'home'};
for i = 1:size(joint_waypoints, 1)
    fprintf('Waypoint %d (%s):\t[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f] deg\n', ...
        i, labels{i}, ...
        rad2deg(joint_waypoints(i,1)), rad2deg(joint_waypoints(i,2)), ...
        rad2deg(joint_waypoints(i,3)), rad2deg(joint_waypoints(i,4)), ...
        rad2deg(joint_waypoints(i,5)), rad2deg(joint_waypoints(i,6)));
end

% Print Simulink-ready output
fprintf('\n--- Copy into Trapezoidal Velocity Profile Trajectory block ---\n\n');
fprintf('[');
for i = 1:size(joint_waypoints, 1)
    fprintf('[%.4f; %.4f; %.4f; %.4f; %.4f; %.4f]', ...
        joint_waypoints(i,1), joint_waypoints(i,2), joint_waypoints(i,3), ...
        joint_waypoints(i,4), joint_waypoints(i,5), joint_waypoints(i,6));
    if i < size(joint_waypoints, 1)
        fprintf(', ');
    end
end
fprintf(']\n');