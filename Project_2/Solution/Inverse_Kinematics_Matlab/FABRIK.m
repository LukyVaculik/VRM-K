%% INFO
%%V1.0, Fabrik algorithm Matlab, 22.5.2024, Lukáš Vaculík
%% RESOURCES
% FABRIK: A fast, iterative solver for the Inverse Kinematics problem
% Andreas Aristidou ⇑, Joan Lasenby
% Department of Engineering, University of Cambridge, Cambridge CB2 1PZ, UK
%Available online: https://www.researchgate.net/publication/220632147_FABRIK_A_fast_iterative_solver_for_the_Inverse_Kinematics_problem

%% CODE
function [iter,points,difA] = FABRIK(x_target,y_target,L1,L2,max_iter,epsilon)
    iter =0;
    
    % Each column is a joint position vector, 
    %Matrix of joint positions (each column is a joint position vector),
    %BASE, END OF FIRST ARM (L1), END OF SECOND (L1+L2)
    points = [0, L1, L1+L2 ; 
              0, 0, 0]; 

    % Target position
    target = [x_target; y_target]; % Column vector

    % Segment distances
    distance = [L1, L2]; % Lengths L1 and L2 (Vector of distances between each pair of consecutive joints)

    % Tolerance for reaching the target
    tol = epsilon;

    % Number of joints
    n = size(points, 2); %3

    % Calculate the distance between the root and the target
    dist = norm(points(:,1) - target); %Distance from 0 is just t

    % Check if the target is within reach
    if dist > sum(distance)
        % The target is unreachable
        for i = 1:2
            % Find the distance ri between the target t and the joint position pi
            ri = norm(target - points(:,i));
            % Calculate the scaling factor ki
            ki = distance(i) / ri;
            % Find the new joint positions pi+1
            points(:,i+1) = (1 - ki) * points(:,i) + ki * target;
        end
    else
        % The target is reachable; save the initial position of the joint p1
        b = points(:,1);

        % Check whether the distance between the end effector pn and the target t is within tolerance
        difA = norm(points(:,3) - target);
        while difA > tol
            % STAGE 1: FORWARD REACHING
            % Set the end effector pn as the target t
            points(:,n) = target;
            for i = 2:-1:1
                % Find the distance ri between the new joint position pi+1 and the joint pi
                ri = norm(points(:,i+1) - points(:,i));
                % Calculate the scaling factor ki
                ki = distance(i) / ri;
                % Find the new joint positions pi
                points(:,i) = (1 - ki) * points(:,i+1) + ki * points(:,i);
            end

            % STAGE 2: BACKWARD REACHING
            % Set the root p1 to its initial position
            points(:,1) = b;
            for i = 1:2
                % Find the distance ri between the new joint position pi and the joint pi+1
                ri = norm(points(:,i+1) - points(:,i));
                % Calculate the scaling factor ki
                ki = distance(i) / ri;
                % Find the new joint positions pi+1
                points(:,i+1) = (1 - ki) * points(:,i) + ki * points(:,i+1);
            end

            % Update the distance between the end effector pn and the target t
            difA = norm(points(:,3) - target);
            iter = iter+1;
            if iter>=max_iter
                break
            end
        end
    end
end