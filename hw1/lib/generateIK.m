%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Task 5: Feasible Trajectory Dataset Generation via Iterative Optimization  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Input Shape: 
%      Please check the MATLAB Workspace after running it once

% Output Shape: 
%      trajectory:    8xN (N is the iteration steps you took to reach)
%

function trajectory = generateIK(robot, lambda, q0, targetPosition, maxJointSpeed, toleranceDistance, dt)
    trajectory = []; % 8xN array with 4 joints position and 4 joints speed

    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Set the initial joint configuration and time
    q = q0;
    t = 0;
    
    % Main loop
    while true
        % Compute the current end-effector position
        currentPosition = robot.fastForwardKinematic(q);
        
        % Check if the end-effector is close enough to the target position
        if norm(currentPosition - targetPosition) < toleranceDistance
            break;
        end
        
        % Compute the Jacobian matrix
        J = robot.fastJacobian(q);
        
        % Compute the desired end-effector velocity
        desiredVelocity = (targetPosition - currentPosition) / norm(targetPosition - currentPosition);
        
        % % Damped pseudo-inverse
        J_inv = pinv(J' * J + lambda^2 * eye(size(J, 2))) * J';

        % Moore-Penrose inverse
        % J_inv = pinv(J);

        % detJ = det(J_inv * J_inv')
        % disp(detJ)
        % svdJ = svd(J_inv)
        % disp(svdJ)

        % Compute the joint velocities using the damped pseudo-inverse of the Jacobian
        q_dot = J_inv * desiredVelocity;
        
        % Scale the joint velocities to ensure they are within the maximum joint speed
        q_dot = q_dot * min(1, maxJointSpeed / max(abs(q_dot)));
        
        % Update the joint positions
        q = q + q_dot * dt;
        
        % Store the joint positions and velocities in the trajectory array
        trajectory = [trajectory, [q; q_dot]];
        
        % Update the time
        t = t + dt;
    end

end