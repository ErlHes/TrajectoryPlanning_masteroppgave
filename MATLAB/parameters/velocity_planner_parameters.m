% Parameters file for the COLAV deliberate system.
%% Define Structs
COLAV_sys_param = struct;
node_param = struct;
cost_settings = struct;
FR = struct;
SPR = struct;
MPR = struct;
BFR = struct;


%% Vessel states
vessel.transit_velocity = 1.6;      %m/s
vessel.max_velocity = 2.0;         %m/s

vessel.length = 5; % meters
vessel.width = 3; %meters 

%% Environment struct;
% This parameters is used to fetch the paths in "get_waypoints_according_to_environment"
COLAV_sys_param.env = 1; % Canal Crossing  Five paths
COLAV_sys_param.env = 2; % Along Canal transit. Five paths


%% Node Parameters
% FR - Forbidden Region
FR.region_type = 1;
FR.node_penaty = 20;
FR.front_vel_gain = 0;
FR.rear_vel_gain = 0;
% FR.angles = [0 , pi/2 , pi,-pi/2];
FR.angles = [pi/6 , pi*3/4 , pi,-pi*4/6];


FR.front_length_gain = 1;
FR.rear_length_gain = 1;
FR.width_gain = 1;


FR.fore_region_addition = 5;
FR.aft_region_addition = 5;
FR.port_region_addition = 2.5;
FR.starboard_region_addition = 2.5;

FR.fore_region_addition = 8;
FR.aft_region_addition = 5;
FR.port_region_addition = 2.5;
FR.starboard_region_addition = 7.5;


% SPR - Strict Penalty Region
SPR.region_type = 2;
SPR.node_penaty = 50;
SPR.front_vel_gain = 0;
SPR.rear_vel_gain = 0;
% SPR.angles = [0 , pi/2 , pi,-pi/2];
SPR.angles = [pi/6 , pi*3/4 , pi,-pi*4/6];


SPR.front_length_gain = 1;
SPR.rear_length_gain = 1;
SPR.width_gain = 1;

SPR.fore_region_addition = 12.5;
SPR.aft_region_addition = 12.5;
SPR.port_region_addition = 11.25;
SPR.starboard_region_addition = 11.25;



% MPR - Mild Penalty Region
MPR.region_type = 3;
MPR.node_penaty = 0;
MPR.front_vel_gain = 0;%12;
MPR.rear_vel_gain = 0;%8;
% MPR.angles = [0 , pi/2 , pi,-pi/2];
MPR.angles = [pi/6 , pi*3/4 , pi,-pi*4/6];

MPR.front_length_gain = 1;
MPR.rear_length_gain = 1;
MPR.width_gain = 1;

MPR.fore_region_addition = 20;
MPR.aft_region_addition = 20;
MPR.port_region_addition = 17.5;
MPR.starboard_region_addition = 17.5;


% Cost parameters for edge cost.
cost_settings.velocity_cost_weight = 0;
cost_settings.penalty_region_cost_weight = 20;
cost_settings.time_cost_weight = 10;
cost_settings.max_time_cost = 20;
cost_settings.branch_cost = 40;
cost_settings.merge_cost = 20;





%% Assign to COLAV_sys_param struct.
node_param.FR = FR;
node_param.SPR = SPR;
node_param.MPR = MPR;
node_param.BFR = BFR;
node_param.cost_settings = cost_settings;
COLAV_sys_param.node_param = node_param;
COLAV_sys_param.vessel = vessel;
COLAV_sys_param.branching_paths_mode = nan;






