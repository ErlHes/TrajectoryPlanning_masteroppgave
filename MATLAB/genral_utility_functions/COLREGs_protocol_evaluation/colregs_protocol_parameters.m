CPE_parameters = struct;


% Safety evaluation parameters
safety = struct;
safety.alpha_cut = pi/2;            % Cutuff value for the relative bearings
safety.beta_cut = pi/2;             % Cutuff value for the relative bearings
safety.s_theta_max = 100;           % Maximum value for the pose safety score.
safety.r_pref = 15;                 % Preferred CPA (evrything above this value is acceptable)
safety.r_min = 11;                  % Minimum acceptable CPA
safety.r_nm = 8;                    % CPA considered to be a near miss
safety.r_col = 5;                   % CPA considered to be a collision
safety.ranges = [5,8,11,15];        % Lookup table values of all the ranges for the safety
safety.range_scores = [0,25,50,100];% Lookup table for brakepoint of score for the safety-ranges.
safety.s_range = 0.5;               % Weighting of range score in total score
safety.s_theta = 0.5;               % Weighting of pose score in total score
safety.tcpa_horizon = 100;          % 
CPE_parameters.safety = safety;