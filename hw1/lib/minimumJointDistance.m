% Task 2c: Minimum distance in joint space 
% This function integrates dq to minimize joint trajectory length
% USE THE SQUARE OF THE NORM FOR NUMERICAL STABILITY

% The robot arm is modeled with the following state space representation:
% - X: state vector = 4 joint angles of the robot arm
% - U(1:4): input vector = 4 joint speed
% - U(5): final time of the trajectory (constant for all timesteps), at 
% which the robot arm should reach a specified target
function cost = minimumJointDistance(X, U, e, data, robot, target)
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    numWaypoints = size(X, 1);
    cost = 0;

    for n = 2:numWaypoints
        % Compute the squared joint space distance between consecutive waypoints
        delta_q = X(n, :) - X(n-1, :);
        cost = cost + norm(delta_q)^2; % Sum of squared joint differences
        %cost = cost + norm(delta_q); % Sum of squared joint differences
    end
end
