robot = importrobot("robot_armV2SLDASM.urdf");
robot.DataFormat = "row";

homeConfig = homeConfiguration(robot);

names = cell(1, robot.NumBodies);
for i = 1:robot.NumBodies
    names{i} = robot.Bodies{i}.Joint.Name;
end

%% Home Position
fprintf("=== HOME POSITION ===\n");
fprintf("%-20s %10s %10s\n", "Joint", "rad", "deg");
fprintf("%s\n", repmat("-", 1, 43));
for i = 1:length(homeConfig)
    fprintf("%-20s %10.4f %10.4f\n", names{i}, homeConfig(i), rad2deg(homeConfig(i)));
end

%% Endeffektor
ee_name = robot.BodyNames{end};
T_home  = getTransform(robot, homeConfig, ee_name);
pos     = T_home(1:3, 4);
eul     = rotm2eul(T_home(1:3,1:3), "XYZ");

fprintf("\n=== ENDEFFEKTOR (Home) ===\n");
fprintf("Position:   x=%.4f  y=%.4f  z=%.4f  [m]\n", pos(1), pos(2), pos(3));
fprintf("Euler XYZ:  r=%.2f  p=%.2f  y=%.2f  [deg]\n", rad2deg(eul(1)), rad2deg(eul(2)), rad2deg(eul(3)));

%% Struktur
fprintf("\n=== ROBOT STRUKTUR ===\n");
fprintf("Anzahl Joints: %d\n", length(homeConfig));
fprintf("Endeffektor:   %s\n", ee_name);

fprintf("\n%-20s %-12s %10s %10s\n", "Joint", "Typ", "Min [deg]", "Max [deg]");
fprintf("%s\n", repmat("-", 1, 55));
for i = 1:robot.NumBodies
    jnt  = robot.Bodies{i}.Joint;
    lims = rad2deg(jnt.PositionLimits);
    fprintf("%-20s %-12s %10.2f %10.2f\n", jnt.Name, jnt.Type, lims(1), lims(2));
end

%% Massen
fprintf("\n=== MASSEN ===\n");
total_mass = 0;
fprintf("%-20s %10s\n", "Body", "Masse [kg]");
fprintf("%s\n", repmat("-", 1, 32));
for i = 1:robot.NumBodies
    m = robot.Bodies{i}.Mass;
    fprintf("%-20s %10.4f\n", robot.Bodies{i}.Name, m);
    total_mass = total_mass + m;
end
fprintf("%-20s %10.4f\n", "GESAMT", total_mass);

%% Jacobian
J     = geometricJacobian(robot, homeConfig, ee_name);
manip = sqrt(abs(det(J * J')));
fprintf("\n=== JACOBIAN & MANIPULIERBARKEIT ===\n");
fprintf("Groesse:           %dx%d\n", size(J,1), size(J,2));
fprintf("Manipulierbarkeit: %.6f\n", manip);
if manip < 0.01
    fprintf("  WARNUNG: Nahe an einer Singularitaet!\n");
end

%% Interaktive IK
fprintf("\n=== INTERAKTIVE IK (STRG+C zum Beenden) ===\n");
fprintf("Eingabe: x y z in Metern (z.B.: 0.3 0.1 0.2)\n\n");

ik = inverseKinematics("RigidBodyTree", robot);
ik.SolverParameters.MaxIterations = 1500;
weights  = [0.25 0.25 0.25 1 1 1];
currentQ = homeConfig;

while true
    raw  = input("Zielposition (x y z): ", "s");
    vals = str2double(strsplit(strtrim(raw)));
    if numel(vals) ~= 3 || any(isnan(vals))
        fprintf("  Fehler: Bitte drei Zahlen eingeben.\n");
        continue
    end

    T_target         = T_home;
    T_target(1:3, 4) = vals(:);

    [solConfig, info] = ik(ee_name, T_target, weights, currentQ);

    T_reached = getTransform(robot, solConfig, ee_name);
    reached   = T_reached(1:3, 4);
    err_mm    = norm(reached - vals(:)) * 1000;

    fprintf("\n  Status: %s\n", info.Status);
    fprintf("  %-20s %10s %10s\n", "Joint", "rad", "deg");
    fprintf("  %s\n", repmat("-", 1, 43));
    for i = 1:length(solConfig)
        fprintf("  %-20s %10.4f %10.4f\n", names{i}, solConfig(i), rad2deg(solConfig(i)));
    end
    fprintf("  Erreicht:  x=%.4f  y=%.4f  z=%.4f\n", reached(1), reached(2), reached(3));
    fprintf("  Fehler:    %.2f mm\n\n", err_mm);

    currentQ = solConfig;
end