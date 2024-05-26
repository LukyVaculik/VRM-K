%% INFO
%%V1.0, Cyclic Coordonate Descent Inverse Kynematic algorithm Matlab, 25.5.2024, Lukáš Vaculík
%% RESOURCES
% Cyclic Coordonate Descent Inverse Kynematic (CCD IK)
% Rodolphe Vaillant 
%Available online: https://rodolphe-vaillant.fr/entry/114/cyclic-coordonate-descent-inverse-kynematic-ccd-ik

%% CODE
function [iter, difA] = CCD(L1, L2, x_target, y_target, maxIterations, tolerance)
    % Initialize joint angles
    theta1 = 0;
    theta2 = 0;
    
    %tagrets
    target(1) = x_target;
    target(2) = y_target;

    % Iterate to adjust angles
    for i = 1:maxIterations
        % Calculate the current end effector position
        endEffector = forwardKinematics(L1, L2, theta1, theta2);

        % Check if the current position is close enough to the target
        difA = norm(target - endEffector);
        if difA < tolerance
            iter = i;
            return;
        end

        % Update joint angles using CCD
        % First joint (base joint)
        joint1Pos = [0, 0];
        joint2Pos = joint1Pos + L1 * [cos(theta1), sin(theta1)];
        endEffector = joint2Pos + L2 * [cos(theta1 + theta2), sin(theta1 + theta2)];

        % Update theta2
        r = endEffector - joint2Pos;
        d = target - joint2Pos;
        theta2 = theta2 + atan2(d(2), d(1)) - atan2(r(2), r(1));

        % Recalculate the end effector position
        endEffector = forwardKinematics(L1, L2, theta1, theta2);

        % Update theta1
        r = endEffector - joint1Pos;
        d = target - joint1Pos;
        theta1 = theta1 + atan2(d(2), d(1)) - atan2(r(2), r(1));
    end
    iter = i;
end

function pos = forwardKinematics(L1, L2, theta1, theta2)
    % Calculate the position of the end effector
    x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    pos = [x, y];
end