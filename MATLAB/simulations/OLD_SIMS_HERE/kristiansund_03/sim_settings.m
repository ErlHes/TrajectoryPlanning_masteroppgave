settings = struct;
settings.t_sim = 500; % Simulation time in seconds
settings.dt = 0.1; %sample time in seconds
settings.time_steps = settings.t_sim/settings.dt;


settings.axis = [-450,650,-260,500];
settings.gcf_position = [0,300,1200,700];
settings.axes_handle = [-0.09,-0.06,1.18,1.13];

settings.t_snapshot = 50:50:settings.t_sim;
settings.agent_color = {get_rgb('blue'), get_rgb('shamrock_green'), get_rgb('orange'),get_rgb('yellow')};


settings.text_offset_x = 10;
settings.text_offset_y = 10;
settings.text_time_stamps_position = [-10,200];
settings.delta_text = 35;
settings.delta_text_x = 150;
settings.text_lines_per_column = 5;
settings.vessel_scale = 4;

settings.legend_position = [260,-440]';
settings.legend_size = [250,230]';

settings.os_1_text = 'OS with CRP';
settings.os_2_text = 'OS with flat cost';