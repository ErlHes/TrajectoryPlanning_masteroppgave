% Parameters for the dummy object detection module for the milliAmpere
% collision avoidance system. This script holds the defenition of the
% objects with velocity, heading and size, as well as any uncetainteis and
% conditions on the "sensorsignals" the dummy module represents.


% Defined structs.
settings = struct;
parameters = struct;
situation_parameters = struct;
object_detection = struct;


%% Simulator Parameters
% nu_0
nu_0_u      = 0;
nu_0_v      = 0;
nu_0_r      = 0;
nu_0 = [0,0,0]';
% eta
eta_0 = [0.1,0.1,0]';




%% Obstacle detection parameters

settings.disturbance             = 0; % 0 = OFF, 1 = ON;
settings.region_of_observation   = 0; % 0 = OFF, 1 = ON;
settings.driver_beviour          = 0; % 0=normal, 1=slow_down, 2=go_in_front, 3=go_behind, 4=follow_colregs

% Parameters on behaviour
parameters.radius_of_slowdown = 40;
parameters.radius_of_stop = 15;

parameters.radius_of_going_in_front = 60;
parameters.distance_of_going_in_front = 20;
parameters.radius_of_going_behind = 60;
parameters.distance_of_going_behind = 20;

parameters.radius_of_observation = inf;
parameters.factor_increase_velocity_when_going_in_front = 1.3;
parameters.alpha_smoothing = 0.99;





%% Useless parameters that should be removed.
%Local NED frame at havnebassenge Brattøra.
parameters.north_west_transit_destination = [ 28,-81];
parameters.south_east_transit_destination = [-51,-19];

% situation_parameters.destination = [100,30];            % [N,E]
% situation_parameters.start_position = [0,0];            % [N,E]
% situation_parameters.transit_velocity = 1;              % m/s
% situation_parameters.region_of_frequent_replan = 30;    % m
        

%% Allocate everyting to one struct.
object_detection.parameters = parameters;
object_detection.settings = settings;
object_detection.obstacles_visible_to_vp = 1;

