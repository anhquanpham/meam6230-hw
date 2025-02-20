%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Question 3b: Generate complete trajectory  %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function optimal_solution_final = generateTraj(disturbance_idx, optimal_solution_full, optimal_solution_after_disturbance)
    optimal_solution_final = [];
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Fill student code here
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Combine trajectories
    optimal_solution_final.Xopt = [optimal_solution_full.Xopt(:, 1:disturbance_idx), optimal_solution_after_disturbance.Xopt];
    optimal_solution_final.Yopt = [optimal_solution_full.Yopt(:, 1:disturbance_idx), optimal_solution_after_disturbance.Yopt];
    optimal_solution_final.MVopt = [optimal_solution_full.MVopt(:, 1:disturbance_idx), optimal_solution_after_disturbance.MVopt];
    
    % Get time points up to disturbance
    pre_time = optimal_solution_full.Topt(1:disturbance_idx);
    
    % Start new trajectory from the time at disturbance_idx
    post_time = optimal_solution_after_disturbance.Topt + pre_time(end);
    
    % Combine and ensure column vector format
    optimal_solution_final.Topt = [pre_time(:); post_time(:)];
    

end
