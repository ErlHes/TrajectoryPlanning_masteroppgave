% Script that loads and plots data from two simulations in one figure, for
% comparison purposes. We assume that the scenario setup is the same for
% both simulations. 

%% Fetch data

home_dir = '/home/emilht/MATLAB/simulator_multiAgent/';
run(strcat(home_dir,'simulations/', simulation,'/sim_settings.m'));
run(strcat(home_dir,'simulations/', simulation,'/env.m'));
run(strcat(home_dir,'simulations/', simulation,'/sim_agents.m'));

% data_file_one = 'sim_output_dynamic_trajectory_reference.mat';
% data_file_one = 'sim_output_benchmark_flat_cost';
data_file_one = 'sim_output.mat';
% data_file_one = 'cost_reduction_profile_sim_output.mat';
% data_file_one = 'cost_reduction_profile_sim_output.mat';
% data_file_one = 'cost_reduction_profile_sim_output_80_seconds.mat';
% data_file_two = 'sim_output_30_second_ACRP.mat';
% data_file_one = 'cost_reduction_profile_sim_output';

% data_file_two = 'sim_output_non_dynamic_trajectory_reference.mat';
% data_file_two = 'Copy_of_sim_output.mat';
% data_file_two = 'adaptive_cost_new_benchmark_sim_output.mat';
% data_file_two = 'no_cost_reduction_profile_sim_output.mat';
% data_file_two = 'no_velocity_cost_sim_output.mat';
data_file_two = 'sim_output_constant_velocity_predictions';
% data_file_one = 'sim_output.mat';


load(strcat(home_dir,'simulations/', simulation, '/',data_file_one));
agent_data_one = sim_output.agent_data;
num_agents = size(agent_data_one,2);
settings.time_steps = size(agent_data_one,1);

load(strcat(home_dir,'simulations/', simulation, '/',data_file_two));
agent_data_two = sim_output.agent_data;




%% Set up figure
figure276 = figure(276);
clf(276)
hold on;
% grid on;
% grid minor;
axis(settings.axis);
set(gca,'Color',get_rgb('water'))

% set(gca,'xtick',[-1000:25:1000])
% set(gca,'ytick',[-1000:25:1000])
% set(gca, 'XColor', [0,0,0])
% set(gca, 'YColor', [0,0,0])

% Set Figure position and size.
if isfield(settings,'gcf_position')
    set(gcf, 'Position', settings.gcf_position)
else
    set(gcf, 'Position', [0,100,1200,500])
end
AxesHandle=findobj(figure276,'Type','axes'); 
if isfield(settings,'axes_handle')
    set(AxesHandle(1,1),'OuterPosition',settings.axes_handle); 
else
    set(AxesHandle(1,1),'OuterPosition',[0,0,1,1]); 
end


figure_position_dimensions = get(gcf, 'Position');
figure_width =figure_position_dimensions(1,3);
font_size_scaling = figure_width/1200; % Ensures that font_size of figures in report appears similar

figure_font_size  = get(gca, 'FontSize');
set(gca,'FontSize',figure_font_size*font_size_scaling);

label_font_size = 14*font_size_scaling;
vessel_text_font_size = 14*font_size_scaling;
info_text_font_size = 15*font_size_scaling;

xlabel('East $(m)$' , 'Interpreter','latex','FontSize',label_font_size)
ylabel('North $(m)$', 'Interpreter','latex','FontSize',label_font_size)

%% Plot static environment
if(exist('static_obs', 'var'))
    [static_obs_handle] = plot_static_obs(static_obs,271);
end


%% Plot situation


delta_t_plot = round(3/settings.dt);


agent_track_history = nan((num_agents+1)*2,size(1:delta_t_plot:settings.time_steps,2));

for i=1:delta_t_plot:settings.time_steps
    count = 1+ (i-1)/delta_t_plot;
    for k=1:size(agent_data_one,2)
        agent_track_history((1+(k-1)*2):k*2,count) = agent_data_one(i,k).eta(1:2); 
    end
    k = k+1;
    agent_track_history((1+(k-1)*2):k*2,count) = agent_data_two(i,1).eta(1:2); 
end

l = size(agent_track_history,2);
cmap = hsv(round(settings.time_steps*1.2));


for i=1:l-1
    for k=1:(size(agent_data_one,2)+1)
      plot(agent_track_history(k*2,i:(i+1)),agent_track_history(1+(k-1)*2,i:(i+1)), '-', 'color',cmap(i*delta_t_plot,:), 'LineWidth',2);
    end
end


text_time_stamps_position = settings.text_time_stamps_position;
delta_text = settings.delta_text;
if isfield(settings,'delta_text_x')
    delta_text_x = settings.delta_text_x;
else
    delta_text_x = 45;
end
if(isfield(settings,'text_lines_per_column'))
    lines_per_column = settings.text_lines_per_column;
else
    lines_per_column = 4;
end
if(isfield(settings,'vessel_scale'))
    vessel_scale = settings.vessel_scale;
else
    vessel_scale = 1;
end

if(isfield(settings,'text_offset_x'))
    text_offset_x = settings.text_offset_x;
    text_offset_y = settings.text_offset_y;
else
    text_offset_x = 5;
    text_offset_y = 5;
end

handle_holder = [];
i_index_from_timesteps = round(settings.t_snapshot*1/settings.dt);
for i_index=1:size(i_index_from_timesteps,2)
    i = i_index_from_timesteps(i_index);
%         delete(handle_holder);
    count = 1+ (i-1)/delta_t_plot;
    text(text_time_stamps_position(2)+delta_text_x*floor((i_index-1)/lines_per_column), text_time_stamps_position(1)-delta_text*mod(i_index-1,lines_per_column),strcat("$",num2str(i_index),": t=", num2str(round(i*settings.dt)), "s$"),'FontSize',info_text_font_size , 'Interpreter','latex'); 
    for k=1:size(agent_data_one,2)
        if(k==1) 
            % Plot OS 1
            agent_ = agent_data_one(i,k);
            course = atan2(agent_.eta_dot(2), agent_.eta_dot(1));
            agent_.eta(3) = course;
            handle_ = plot_os(agent_.eta, [0,0,1],vessel_scale); % Eta

            figure_axis =  settings.axis;
            if(within_figure_axis(flip(agent_data_one(i,k).eta(1:2,1)),figure_axis))
                text(agent_data_one(i,k).eta(2)+text_offset_x,agent_data_one(i,k).eta(1)+text_offset_y, strcat(num2str(i_index)), 'color', settings.agent_color{1,k},'FontSize',vessel_text_font_size , 'Interpreter','latex');
            end
            
            % Plot OS 2
            agent_ = agent_data_two(i,k);
            course = atan2(agent_.eta_dot(2), agent_.eta_dot(1));
            agent_.eta(3) = course;
            handle_ = plot_os(agent_.eta, [0.5,0.5,0.5],vessel_scale); % Eta
            figure_axis =  settings.axis;
            if(within_figure_axis(flip(agent_data_two(i,k).eta(1:2,1)),figure_axis))
                text(agent_data_two(i,k).eta(2)+text_offset_x,agent_data_two(i,k).eta(1)+text_offset_y, strcat(num2str(i_index)), 'color', settings.agent_color{1,k},'FontSize',vessel_text_font_size , 'Interpreter','latex');
            end
            
        else
            agent_ = agent_data_one(i,k);
            course = atan2(agent_.eta_dot(2), agent_.eta_dot(1));
            agent_.eta(3) = course;
            handle_ = plot_ts_from_struct(agent_,cmap(i,:),vessel_scale);
            figure_axis =  settings.axis;
            if(within_figure_axis(flip(agent_data_one(i,k).eta(1:2,1)),figure_axis))
                text(agent_data_one(i,k).eta(2)+text_offset_x,agent_data_one(i,k).eta(1)+text_offset_y, strcat(num2str(i_index)), 'color', settings.agent_color{1,k},'FontSize',vessel_text_font_size , 'Interpreter','latex');
            end

        end
    end

end
%% Plot Legend


if(isfield(settings,'legend_position'))
     % Make Custom Legend
    
    legend_size = settings.legend_size;
    legend_position = settings.legend_position;
    
    graph_handle =[];
   
    handle_ = rectangle('Position',[legend_position(2), legend_position(1),legend_size(1),legend_size(2)],'FaceColor',[1 1 1],'EdgeColor',[0 0 0],'LineWidth',1);
    graph_handle = [graph_handle, handle_];
    
    vessel = agent_data_one(1,1);
    
    % Own Ship Vessel Legend
    vessel.eta(1:2) = legend_position + [legend_size(2)-15; 25];
    vessel.eta(3,1) = -pi/2;
    handle_ = plot_os_from_struct(vessel,'b',6.0);
    graph_handle = [graph_handle, handle_];
%     thandle = text(vessel.eta(2)+25, vessel.eta(1), settings.os_1_text, 'fontsize',13);
    thandle = text(vessel.eta(2)+25, vessel.eta(1), 'OS simulation 1', 'fontsize',13);
%     thandle = text(vessel.eta(2)+25, vessel.eta(1), 'OS 1', 'fontsize',13);
    graph_handle = [graph_handle, handle_,thandle];

    
    % Own Ship Vessel Legend
    vessel.eta(1:2) = legend_position + [legend_size(2)-45; 25];
    vessel.eta(3,1) = -pi/2;
    handle_ = plot_os_from_struct(vessel,[0.5,0.5,0.5],6.0);
    graph_handle = [graph_handle, handle_];
%     thandle = text(vessel.eta(2)+25, vessel.eta(1), settings.os_2_text, 'fontsize',13);
    thandle = text(vessel.eta(2)+25, vessel.eta(1), 'OS simulation 2', 'fontsize',13);

    %     thandle = text(vessel.eta(2)+25, vessel.eta(1), 'OS 2', 'fontsize',13);
    graph_handle = [graph_handle, handle_,thandle];
    
    
    % Target Ship Vessel Legend
    vessel.eta(1:2) = legend_position + [legend_size(2)-70; 25];
    vessel.eta(3,1) = -pi/2;
    handle_ = plot_ts_from_struct(vessel,get_rgb('light_grey'),6.0);
    thandle = text(vessel.eta(2)+25, vessel.eta(1), 'Target ship vessel', 'fontsize',13);
    graph_handle = [graph_handle, handle_,thandle];
    
    
    % Static obstacles legend
    position = legend_position + [legend_size(2)-98; 25];
    handle_ = legend_filled_square(position, 30, get_rgb('land'));
    thandle = text(position(2) +25, position(1), 'Static obstacles', 'fontsize',13);
    graph_handle = [graph_handle, handle_,thandle];

end

%%
AxesHandle=findobj(figure276,'Type','axes'); 
if isfield(settings,'axes_handle')
    set(AxesHandle(1,1),'OuterPosition',settings.axes_handle); 
else
    set(AxesHandle(1,1),'OuterPosition',[0,0,1,1]); 
%     set(AxesHandle(1,1),'OuterPosition',[-0.09,-0.06,1.18,1.13]); 
end




%% Supporting functions
function within_axis = within_figure_axis(p,axis)
within_axis = (p(1)>axis(1)) && (p(1)<axis(2)) && (p(2)>axis(3))&& (p(2)<axis(4));
end