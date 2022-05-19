% This scipts runs a multi-agent simulator for surface vessels.
clear;
clear MPC_with_Assist.m;    % Need to clear some persistent variables.
clc;
%% OLD SIMULATIONS
for d = 1
% simulation = 'pid_controller_tesiting';
% simulation = 'test';
% simulation = 'panama_south_to_north';
% simulation = 'crossing_from_starboard';
% simulation = 'crossing_from_starboard_2';
% simulation = 'crossing_from_port';
% simulation = 'head_on';
% simulation = 'head_on_02';
% simulation = 'overtaking_02';
% simulation = 'head_on_east_west';
% simulation = 'brattora_ranvkloa_along_canal_7';
% simulation = 'exit_to_open_waters_1';
% simulation = 'exit_to_open_waters_2';
% simulation = 'harbour_exit';
% simulation = 'star_maneuver_01';
% simulation = 'star_maneuver_02';
% simulation = 'traffic_junction_01';

%% MPC WITH ASSIST TESTS
%simulation = 'COLREGs_flagtest';
%simulation = 'Test1';
%simulation = 'Test2';
%simulation = 'open_sea';
%simulation = 'constrained_strait';
%simulation = 'local_min_strait';
% simulation = 'local_min_strait_Offstart';
%simulation = 'islands';
% simulation = 'islands_GWcrossing';
%simulation = 'islands_SOcrossing';
%simulation = 'islands_HO';

%% MPC with assist ENV SCENARIOS:
% simulation = 'Local_minimum';
% simulation = 'Blocked_path';

%% MPC WITH ASSIST COLREGs SCENARIOS:
% simulation = 'islands_GWcrossing'; % OUTDATED
% simulation = 'islands_SOcrossing'; % OUTDATED
% simulation = 'Simple_HO'; % OUTDATED
% simulation = 'StraitCross'; %non compliant TS from port side
% simulation = 'StraitCross_HO';
% simulation = 'StraitCross_OT';

%% MPC WITH ASSIST TRAFFIC PATTERN DIFFERENCE:
% simulation = 'island_heavy_turn'; % This shit don't work.

%% Velocity Obstacle scenarios
% simulation = 'VO_head_on_east_west';
% simulation = 'VO_c_shaped_path_testing_filtered_referece_ocp';
%% OCP scenarios
% Without acceleration
% simulation = 'head_on_ocp_full';
% simulation = 'crossing_from_starboard_ocp';
% With acceleration
% simulation = 'crossing_from_starboard_ocp_02';
% simulation = 'head_on_ocp_full';
% simulation = 'brattora_ranvkloa_along_canal_4_ocp';
% simulation = 'brattora_ranvkloa_along_canal_5_ocp';
% simulation = 'brattora_ranvkloa_along_canal_7_ocp';
% simulation = 'kristiansund_01';
%  simulation = 'kristiansund_02';
% simulation = 'kristiansund_03';
% simulation = 'kristiansund_04';
% simulation = 'straight_north_debug';
% simulation = 'c_shaped_path_testing_filtered_referece_ocp';
% simulation = 'sandefjord_01';
% simulation = 'sandefjord_02';
% simulation = 'sandefjord_03';

end

%% Useful parameter tuning sims
%  simulation = 'Walk_in_the_park';

%% COLREGs Classification Testing
% simulation = 'Classificationtest';
 
%% Head-on Testing
% simulation = 'HO1';

%% With and without prediction sims

% simulation = 'Havn2';

% simulation = 'Ferjekryss';

%% The scenario list!!
% simulation = 'Havn1';
% simulation = 'Trheimfjord';
% simulation = 'enkel_HO';
% simulation = 'enkel_GW';
% simulation = 'enkel_SO';
% simulation = 'sving_HO';
% simulation = 'sving_GW';
% simulation = 'sving_SO';
% simulation = 'skjergard_m_trafikk';
simulation = 'Helloya';

%% Discretization step length debugg
% simulation = 'Race';

%% Gather simulation parameters

run('get_combined_parameter_struct.m');

home_dir = 'C:\Users\erlen\Documents\GitHub\TrajectoryPlanning_masteroppgave\MATLAB\';
% Define all simulator parameters
run(strcat(home_dir,'simulations/', simulation,'/sim_settings.m'));
% Define all agents
run(strcat(home_dir,'simulations/',simulation,'/sim_agents.m'));
run(strcat(home_dir,'simulations/',simulation,'/env.m'));


if~(exist('static_obs', 'var'))
    static_obs = [];
end

set_global_map_data(static_obs);
% Define environment
% Preallocate array for data storage
% run(strcat(home_dir,'utility/','preallocate_data_array.m'));


disp('done');



%% Modify parameters
controller_param.delta_t = settings.dt;
visualization = true;
sys_parameters.run_visualization = true;
sys_parameters.plot_barriers = true;
sys_parameters.line_of_sight_visibility = 0;

if(parameters.system.make_video)
    visualization_interval = 9;
else
    visualization_interval = 50;
end

vizualization_counter = visualization_interval;
%% Run Simulations
run(strcat(home_dir,'/run_simulation.m'));


%% Data Saving
sim_output = struct;
sim_output.agent_data = agent_data;
sim_output.time = 0:settings.dt:time-settings.dt;
sim_output.settings = settings;
save(strcat(home_dir,'simulations/', simulation, '/sim_output.mat'), 'sim_output');

%% Clear variables
clear('agent_data', 'agents', 'empty_agent', 'time','sim_output','iteration','dt','j','t_sim','settings');
clear;