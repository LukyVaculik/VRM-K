%% INFO
%%V1.0, Main function structure - calling IK methods, 25.5.2024, Lukáš Vaculík
%% RESOURCES
%Programming for robots and manipulators, Lecture 4, Ing. Roman Parak, BUT Course VRM-K

%% CODE
function [iterDE, iterFabrik, iterCCD,...
    difDE, difFabrik, difCcd,...
    unsuccesfullDE, unsuccesfullFabrik, unsuccesfullCCD]...;
    = InverseKinematicsAlgs(L1,L2, par_eps, max_iter)
    % Define arm parameters
    %%taken from funtion

    %% Generate random angles in range -pi to pi
    % Define joint angle limits
    theta_min = -pi;  % Minimum limit for theta
    theta_max = pi;  % Maximum limit for theta

    % Generate random joint angles within limits
    theta(1) = rand() * (theta_max - theta_min) + theta_min; %scaling + adding minimum value
    theta(2) = rand() * (theta_max - theta_min) + theta_min;
    theta(2,1) = rand() * (theta_max - theta_min) + theta_min;
    % Display the generated joint angles
    % disp('Random Joint Angles:');
    % disp(['theta1: ', num2str(theta(1))]);
    % disp(['theta2: ', num2str(theta(2))]);


    %% Calculate Inverse Kinematics

    %Get coordinates 
    x_target= L1*cos(theta(1))+L2*cos(theta(1)+theta(2));
    y_target= L1*sin(theta(1))+L2*sin(theta(1)+theta(2));


    % Check if the target point is reachable by the arm (will always be beacouse of FK)
    % Calculate the radius from the base to the target point
    % L = sqrt(x_target^2 + y_target^2);
    % if (L > L1 + L2)
    %     disp(['Target point [', num2str(x_target), ', ', num2str(y_target), '] is unreachable']);
    % end
    %% DIFFERENTIAL EVOLUTION
    unsuccesfullDE=false;
    x0= (theta_max - theta_min) * rand(2, 1) + theta_min; %initial in range of input

    % Define the fitness function
    fitness_function = @(x) calculate_fitness(x, x_target, y_target, L1, L2);

    [difDE, xs,iterDE,pops] = differential_evolution(fitness_function,x0,max_iter,par_eps,-pi,+pi);
    if ((iterDE == max_iter) && (difDE >= par_eps))
        unsuccesfullDE=true;
    end



    %% FABRIK
    unsuccesfullFabrik=false;
    [iterFabrik,p,difFabrik] = FABRIK(x_target,y_target,L1,L2,max_iter,par_eps);
    if ((iterFabrik == max_iter) && (difFabrik >= par_eps))
        unsuccesfullFabrik=true;
    end


    %% CCD
    unsuccesfullCCD = false;
    [iterCCD, difCcd] = CCD(L1, L2, x_target,y_target, max_iter, par_eps);
    if ((iterCCD == max_iter) && (difCcd >= par_eps))
        unsuccesfullCCD=true;
    end

    %% Other function definitions
    %Fitness calculation for diff. alg
    function distance = calculate_fitness(theta, x_target, y_target, l1, l2)
        % Calculate the end effector position for the given joint angles
            x_end = l1 * cos(theta(1)) + l2 * cos(theta(1) + theta(2));
            y_end = l1 * sin(theta(1)) + l2 * sin(theta(1) + theta(2));

        % Calculate the distance between the end effector and the target point
        distance = norm([x_end - x_target, y_end - y_target]);
    end

end