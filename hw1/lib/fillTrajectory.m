%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Task 4: Compute closed-form trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Input Shape: 
%      Please check the MATLAB Workspace after running it once

% Output Shape: 
%      cartesianTrajectory:    3x50
%

function cartesianTrajectory = fillTrajectory(time, initial_position, waypoints, target_position)
    % Compute trajectory based on third order polynomial
    cartesianTrajectory = nan(3, length(time));
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Combine all points: initial, waypoints, and target
    all_positions = [initial_position, waypoints, target_position];
    
    % Time allocation for the entire trajectory
    t_start = 0;
    t_end = max(time);
    
    % Solve for polynomial coefficients for each dimension
    for dim = 1:3
        % Values for start and end positions and their derivatives (velocities)
        p_start = all_positions(dim, 1);
        p_end = all_positions(dim, end);
        
        % Set up the system of equations
        A = [1, t_start, t_start^2, t_start^3;
             0, 1, 2*t_start, 3*t_start^2;
             1, t_end, t_end^2, t_end^3;
             0, 1, 2*t_end, 3*t_end^2];
        
        % Boundary conditions for position and velocity
        b = [p_start; 0; p_end; 0];
        
        % Solve for cubic coefficients
        coeffs = A \ b;

        % Print the polynomial coefficients
        fprintf('Polynomial coefficients for dimension %d: a0 = %.4f, a1 = %.4f, a2 = %.4f, a3 = %.4f\n', ...
                dim, coeffs(1), coeffs(2), coeffs(3), coeffs(4));
        
        % Evaluate the polynomial for the entire time
        for i = 1:length(time)
            t = time(i);
            cartesianTrajectory(dim, i) = coeffs(1) + coeffs(2)*t + ...
                                          coeffs(3)*t^2 + coeffs(4)*t^3;
        end
    end

    
end
