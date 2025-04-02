%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file is a template file to import 3D dataset to learn a DS from    %
% trajectories, and exporting the DS to be deployed on the Panda robot.   %
%  
% ====>> You should use functions from part 2 of this homework!           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Import dependencies
close all; clear; clc
filepath = fileparts(which('learn_DS_3D.m'));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-ds-opt')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-sods-opt')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-phys-gmm')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-thirdparty')));
addpath(genpath(fullfile(filepath, '..', 'libraries', 'book-robot-simulation')));
addpath(genpath(fullfile(filepath, 'dataset')));
% cd(filepath); %<<== This might be necessary in some machines

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 1 (DATA LOADING): Choose among the predifined datasets %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load and convert 3D dataset
% Import from the dataset folder either:
% - Task 1: 'theoretical_DS_dataset.mat'
% - Task 2: 'MPC_train_dataset.mat'
% - Task 2: 'MPC_test_dataset.mat'
% - Task 3: '3D_Cshape_bottom_processed.mat'
% - Task 3: 'raw_demonstration_dataset.mat'

load("theoretical_DS_dataset.mat"); % --> Modify me to load different datasets!!
% usingSEDS --> Modify me if you will use seds or lpvds! 
% (necessary for parameter storing method used in robot_DS_control.m)
usingSEDS = false;
% filter --> Modify me if you want to pre-process the datasets 
% (relevant for task 3)
filter = false;

% All code below is used to extract trajectories in format amenable to
% learning the DS with the codes provided in part 2 .m scripts
nTraj = size(trajectories, 3);
nPoints = size(trajectories, 2);

Data = [];
attractor = zeros(3, 1);
x0_all = zeros(3, nTraj);

% When filter = true the next lines of code will apply a savitzky golay
% filter to your data (this is recommended for raw human demonstrations)
for i = 1:nTraj
    traj = trajectories(:,:,i);
    if filter
        % Filter Trajectories and Compute Derivativess with Savitzky Golay filter
        %   traj: The trajectory you want to filter
        %   sample_step: subsample the traj before filtering
        %   nth_order :     max order of the derivatives 
        %   n_polynomial :  Order of polynomial fit
        %   window_size :   Window length for the filter
        traj = sgolay_filter_smoothing(trajectories(:,:,i), 5, 1, 2, 10);
    end

    Data = [Data traj];
    x0_all(:,i) = traj(1:3,1);
    attractor = attractor + traj(1:3,end);
end
attractor = attractor / nTraj;

% Normalizing dataset attractor position
M = size(Data, 1) / 2; 
Data(1:M,:) = Data(1:M,:) - attractor;
x0_all = x0_all - attractor;
att = [0; 0; 0];

% Plot position/velocity Trajectories
vel_samples = 5; vel_size = 0.75; 
[h_data, h_att, ~] = plot_reference_trajectories_DS(Data, att, vel_samples, vel_size);

% Extract Position and Velocities
M = size(Data,1) / 2;    
Xi_ref = Data(1:M,:);
Xi_dot_ref  = Data(M+1:end,:);   
axis_limits = axis;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%    Step 2: ADD YOUR CODE BELOW TO LEARN 3D DS      %%
%% vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %%%%%

%%% MODIFY ME!!!
%%% MODIFY ME!!!
%%% MODIFY ME!!!
if usingSEDS
    %% Start of ch3_ex3_seDS.m
    tStart = cputime;
    
    % GMM FITTING
    est_options = [];
    est_options.type = 1; %GMM-EM Model Selection via BIC
    est_options.maxK = 10;
    est_options.samplerIter = 50;
    sub_sample = 1;
    est_options.sub_sample       = sub_sample;
    l_sensitivity = 2;
    est_options.l_sensitivity    = l_sensitivity; 
    est_options.estimate_l       = 1;
    est_options.do_plots = 0; % Ensure do_plots is defined
    disp(size(Xi_ref));      % Should be (D/2 x N)
    disp(size(Xi_dot_ref));  % Should be (D/2 x N)

    [Priors, Mu, Sigma] = fit_gmm([Xi_ref; Xi_dot_ref], [], est_options);
    
    % SEDS SOLVER
    options.objective = 'likelihood';
    options.max_iter = 100;
    [Priors, Mu, Sigma] = SEDS_Solver(Priors, Mu, Sigma, [Xi_ref; Xi_dot_ref], options);
    ds_seds = @(x) GMR_SEDS(Priors, Mu, Sigma, x - repmat(att,[1 size(x,2)]), 1:M, M+1:2*M);
    tEnd = cputime - tStart;
    
else
    %% Start of ch3_ex4_lpvDS.m
    tStart = cputime;
    
    % GMM FITTING
    est_options = [];
    est_options.type = 1;
    est_options.maxK = 10;
    est_options.samplerIter = 50;
    sub_sample = 8;
    est_options.sub_sample       = sub_sample;
    l_sensitivity = 2;
    est_options.l_sensitivity    = l_sensitivity; 
    est_options.estimate_l       = 1;
    est_options.do_plots = 0; % Ensure do_plots is defined
    [Priors, Mu, Sigma] = fit_gmm(Xi_ref, Xi_dot_ref, est_options);
    
    % LPV-DS OPTIMIZATION
    constr_type = 2;
    init_cvx = 1;
    [A_k, b_k, ~] = optimize_lpv_ds_from_data(Data, att, constr_type, struct('Mu', Mu, 'Sigma', Sigma, 'Priors', Priors), eye(M), init_cvx);
    ds_lpv = @(x) lpv_ds(x, struct('Mu', Mu, 'Sigma', Sigma, 'Priors', Priors), A_k, b_k);
    ds_gmm.Mu = Mu;
    ds_gmm.Sigma = Sigma;
    ds_gmm.Priors = Priors;
    tEnd = cputime - tStart;
end

%%   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ %%%
%%    Step 2: ADD YOUR CODE ABOVE TO LEARN 3D DS      %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Step 3 (SAVE DS): Save learned DS parameters for robot control %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save DS for simulation using 'DS_control.m'
filename = strcat(filepath,'/ds_control.mat');
if usingSEDS
    ds_control = @(x) ds_seds(x - attractor);
    save('ds_control.mat', "ds_control", "attractor", "Priors", "Mu", "Sigma", "att", "M")
else
    ds_control = @(x) ds_lpv(x - attractor);
    save('ds_control.mat', "ds_control", "attractor", "ds_gmm", "A_k", "b_k", "att")
end

%% %%%%%%%%%%%% Plot Resulting DS %%%%%%%%%%%%%%%%%%%
% Fill in plotting options
ds_plot_options = [];
ds_plot_options.sim_traj = 1; % To simulate trajectories from x0_all
ds_plot_options.x0_all = x0_all; % Initial Points
ds_plot_options.init_type = 'ellipsoid'; % For 3D DS, to initialize streamlines
% 'ellipsoid' or 'cube'
ds_plot_options.nb_points = 30; % # of streamlines to plot (3D)
ds_plot_options.plot_vol = 0; % Plot volume of initial points (3D)

% Choose the correct DS function based on usingSEDS
if usingSEDS
    ds_func = ds_seds;
else
    ds_func = ds_lpv;
end

% Visualize the estimated DS
[~, hs, hr, x_sim] = visualizeEstimatedDS(Data(1:M,:), ds_func, ds_plot_options);