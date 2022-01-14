

yaml_struct = ReadYaml('/home/emilht/NTNU-aFerry/workspace/src/ros_cbf_based_colav_utils/config/cbf_based_colav_utils.yaml');

% Defined the CBF parameters 
% Classification is numbered as follows:
% 1: SF - Safe
% 2: SO - Stand-on
% 3: OTs - Overtaking on starboard side
% 4: OTp - Overtaking on port side
% 5: HO - Head-on
% 6: GW - Give-way.
cbf_parameters = struct;


% Linear gain for inequality of h_dot lower limit as function of h.
cbf_parameters.gamma_dyn = zeros(1,6);
cbf_parameters.gamma_dyn(1) = yaml_struct.cbf_parameters.gamma_dyn_1;
cbf_parameters.gamma_dyn(2) = yaml_struct.cbf_parameters.gamma_dyn_2;
cbf_parameters.gamma_dyn(3) = yaml_struct.cbf_parameters.gamma_dyn_3;
cbf_parameters.gamma_dyn(4) = yaml_struct.cbf_parameters.gamma_dyn_4;
cbf_parameters.gamma_dyn(5) = yaml_struct.cbf_parameters.gamma_dyn_5;
cbf_parameters.gamma_dyn(6) = yaml_struct.cbf_parameters.gamma_dyn_6;


% Parameter for mitigation between range and relative velocity towards obstacle
cbf_parameters.c_dyn = zeros(1,6);
cbf_parameters.c_dyn(1) = yaml_struct.cbf_parameters.c_dyn_1;
cbf_parameters.c_dyn(2) = yaml_struct.cbf_parameters.c_dyn_2;
cbf_parameters.c_dyn(3) = yaml_struct.cbf_parameters.c_dyn_3;
cbf_parameters.c_dyn(4) = yaml_struct.cbf_parameters.c_dyn_4;
cbf_parameters.c_dyn(5) = yaml_struct.cbf_parameters.c_dyn_5;
cbf_parameters.c_dyn(6) = yaml_struct.cbf_parameters.c_dyn_6;

% Deflection angle alpha_d
cbf_parameters.alpha_d = zeros(1,6);
cbf_parameters.alpha_d(1) = yaml_struct.cbf_parameters.alpha_d_1;
cbf_parameters.alpha_d(2) = yaml_struct.cbf_parameters.alpha_d_2;
cbf_parameters.alpha_d(3) = yaml_struct.cbf_parameters.alpha_d_3;
cbf_parameters.alpha_d(4) = yaml_struct.cbf_parameters.alpha_d_4;
cbf_parameters.alpha_d(5) = 0;%yaml_struct.cbf_parameters.alpha_d_5;
cbf_parameters.alpha_d(6) = yaml_struct.cbf_parameters.alpha_d_6;

% Port starboard split angle, affecth which way 
% the SO deflects, based on relative bearing of the OS from the TS.
cbf_parameters.port_starboard_split = zeros(1,6);
cbf_parameters.port_starboard_split(1) = yaml_struct.cbf_parameters.port_starboard_split_1;
cbf_parameters.port_starboard_split(2) = yaml_struct.cbf_parameters.port_starboard_split_2;
cbf_parameters.port_starboard_split(3) = yaml_struct.cbf_parameters.port_starboard_split_3;
cbf_parameters.port_starboard_split(4) = yaml_struct.cbf_parameters.port_starboard_split_4;
cbf_parameters.port_starboard_split(5) = yaml_struct.cbf_parameters.port_starboard_split_5;
cbf_parameters.port_starboard_split(6) = yaml_struct.cbf_parameters.port_starboard_split_6;

cbf_parameters.alpha_D_max = zeros(1,6);
cbf_parameters.alpha_D_max(1) = yaml_struct.cbf_parameters.alpha_D_max_1;
cbf_parameters.alpha_D_max(2) = yaml_struct.cbf_parameters.alpha_D_max_2;
cbf_parameters.alpha_D_max(3) = yaml_struct.cbf_parameters.alpha_D_max_3;
cbf_parameters.alpha_D_max(4) = yaml_struct.cbf_parameters.alpha_D_max_4;
cbf_parameters.alpha_D_max(5) = yaml_struct.cbf_parameters.alpha_D_max_5;
cbf_parameters.alpha_D_max(6) = yaml_struct.cbf_parameters.alpha_D_max_6;

cbf_parameters.alpha_D_min = zeros(1,6);
cbf_parameters.alpha_D_min(1) = yaml_struct.cbf_parameters.alpha_D_min_1;
cbf_parameters.alpha_D_min(2) = yaml_struct.cbf_parameters.alpha_D_min_2;
cbf_parameters.alpha_D_min(3) = yaml_struct.cbf_parameters.alpha_D_min_3;
cbf_parameters.alpha_D_min(4) = yaml_struct.cbf_parameters.alpha_D_min_4;
cbf_parameters.alpha_D_min(5) = yaml_struct.cbf_parameters.alpha_D_min_5;
cbf_parameters.alpha_D_min(6) = yaml_struct.cbf_parameters.alpha_D_min_6;


cbf_parameters.l_delta = zeros(1,6);
cbf_parameters.l_delta(1) = yaml_struct.cbf_parameters.l_delta_1;
cbf_parameters.l_delta(2) = yaml_struct.cbf_parameters.l_delta_2;
cbf_parameters.l_delta(3) = yaml_struct.cbf_parameters.l_delta_3;
cbf_parameters.l_delta(4) = yaml_struct.cbf_parameters.l_delta_4;
cbf_parameters.l_delta(5) = yaml_struct.cbf_parameters.l_delta_5;
cbf_parameters.l_delta(6) = yaml_struct.cbf_parameters.l_delta_6;



cbf_parameters.max_free_distance_addition =  30;%yaml_struct.cbf_parameters.max_free_distance_addition;
cbf_parameters.k_l_free = yaml_struct.cbf_parameters.k_l_free;
cbf_parameters.k_l_free = 0.5; %yaml_struct.cbf_parameters.k_l_free;

cbf_parameters.delta_static_obstacles = yaml_struct.cbf_parameters.delta_static_obstacles;
cbf_parameters.delta_dynamic_obstacles = yaml_struct.cbf_parameters.delta_dynamic_obstacles;


% Parameters for static obstacles.
cbf_parameters.gamma_stat = yaml_struct.cbf_parameters.gamma_stat;
cbf_parameters.c_stat = yaml_struct.cbf_parameters.c_stat;
% cbf_parameters.delta_xi = yaml_struct.cbf_parameters.delta_xi;
cbf_parameters.n_sectors = yaml_struct.cbf_parameters.n_sectors;
cbf_parameters.max_distance_to_static_obs = yaml_struct.cbf_parameters.max_distance_to_static_obs;


cbf_parameters_bus = cbf_parameters;

Simulink_obj = Simulink.Parameter;
Simulink_obj.Value = cbf_parameters_bus;
Simulink_obj.CoderInfo.StorageClass = 'ExportedGlobal';
cbf_parameters_bus = Simulink_obj;
clear Simulink_el;

busInfo=Simulink.Bus.createObject(cbf_parameters_bus.Value);
CbfParameters = eval(busInfo.busName);
cbf_parameters_bus.DataType='Bus: CbfParameters';
clear(busInfo.busName);
clear busInfo;



