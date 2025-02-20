% Task 2c: Minimum distance in joint space 
% This function integrates dq to minimize joint trajectory length
% USE THE SQUARE OF THE NORM FOR NUMERICAL STABILITY

% Input Shape: 
%      Please check the MATLAB Workspace after running it once

% Output Shape: 
%      cost: float
%

% The robot arm is modeled with the following state space representation:
% - X(i, 1:4): the ith time step state vector (joint angles) in a trajectory
% - U(i, 1:4): the ith time step input vector (joint velocities) in a trajectory
% - U(any index, 5): final time of the trajectory (constant for all timesteps), at 
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
