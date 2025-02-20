%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Question 3a: Add disturbance to trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [disturbance_idx, q_mid] = addDisturbance(optimal_solution_full, q_disturbance, disturbance_val)
    % Get midpoint of trajectory
    disturbance_idx = round(size(optimal_solution_full.Xopt, 2) / 2);
    %disturbance_idx = 4;
    % Get joint positions at midpoint
    q_mid = optimal_solution_full.Xopt(:, disturbance_idx);
    
    
    % Apply disturbance to specified joint
    q_mid(q_disturbance) = q_mid(q_disturbance) + deg2rad(disturbance_val);
end