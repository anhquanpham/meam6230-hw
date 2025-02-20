% Task 2b: Minimum distance in task space 
% This function integrates dx = J*dq to minimize Cartesian trajectory length
% You can obtain the Jacobian J at configuration q using
% J = robot.fastJacobian(q)
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
function cost = minimumTaskDistance(X, U, e, data, robot, target)
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    % Initialize cost
    cost = 0;
    
    % Get number of timesteps
    N = size(X,1);
   
    % For each timestep, compute and sum up the squared Cartesian velocities
    for i = 1:N-1
        % Get current joint positions and velocities
        q = X(i,:)';          % 4x1 joint positions
        dq = U(i,1:4)';       % 4x1 joint velocities
        
        % Get Jacobian at current configuration 
        J = robot.fastJacobian(q);
        
        % Compute Cartesian velocity dx = J*dq
        dx = J*dq;


        % Add squared norm to cost (for numerical stability)
        cost = cost + dx' * dx;  
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%
end
