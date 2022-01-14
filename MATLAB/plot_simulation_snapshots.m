%% Fetch data

    home_dir = '/home/emilht/MATLAB/multiAgent_simulator/';
    load(strcat(home_dir,'simulations/', simulation, '/sim_output.mat'));
    run(strcat(home_dir,'simulations/', simulation,'/sim_settings.m'));
    run(strcat(home_dir,'simulations/', simulation,'/env.m'));
    run(strcat(home_dir,'simulations/', simulation,'/sim_agents.m'));
    % settings = sim_output.settings;
    agent_data = sim_output.agent_data;
    % time = sim_output.time;
    num_agents = size(agent_data,2);
    settings.time_steps = size(agent_data,1);

%% Set up figure
figure271 = figure(271);
clf(271)
hold on;
% grid on;
% grid minor;
axis(settings.axis);
set(gca,'Color',get_rgb('water'))
% 
% set(gca,'xtick',[-1000:25:1000])
% set(gca,'ytick',[-1000:25:1000])
set(gca, 'XColor', [0,0,0])
set(gca, 'YColor', [0,0,0])

% Set Figure position and size.
if isfield(settings,'gcf_position')
    set(gcf, 'Position', settings.gcf_position)
else
    set(gcf, 'Position', [0,100,1200,500])
end
AxesHandle=findobj(figure271,'Type','axes'); 
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


%%
xlabel('East $[m]$' , 'Interpreter','latex','FontSize',label_font_size)
ylabel('North $[m]$', 'Interpreter','latex','FontSize',label_font_size)

%% Plot static environment

if(exist('static_obs', 'var'))
    [static_obs_handle] = plot_static_obs(static_obs,271);
end


%% Draw legend

draw_custom_legend(settings,agent_data(1,1))

%% Plot states of the involved vessels
delta_t_plot = round(1/settings.dt);       
cmap = hsv(round(settings.time_steps*1.2)); % Colormap for plotting time-evolvement



% Plot reference trajectories
for k=1:1 %size(agent_data,2)
   plot(agent_data(1,k).wp(2,:), agent_data(1,k).wp(1,:), '--', 'color', get_rgb('light_grey'), 'LineWidth',2);
end

agent_track_history = nan(num_agents*2,size(1:delta_t_plot:settings.time_steps,2));

for i=1:delta_t_plot:settings.time_steps
    count = 1+ (i-1)/delta_t_plot;
    for k=1:size(agent_data,2)
        agent_track_history((1+(k-1)*2):k*2,count) = agent_data(i,k).eta(1:2); 
    end
end

l = size(agent_track_history,2);


% 
% 
% %%
% % Plot areas on trajectory with cost reduction profiles for the MPC method
% if(agent_data(1,1).gnc == 4) %% MPC-based trajectory planner
%    acceleration_cost_reduction = nan(size(agent_track_history(1:2,:)));
%    position_cost_reduction =  acceleration_cost_reduction;
%   
%    for i=1:l
%        
%        
%        if(size(agent_data(i*delta_t_plot,1).acceleration_cost_reduction,2) > 1)
%            if(agent_data(i*delta_t_plot,1).acceleration_cost_reduction(1,2) > 1) && (agent_data(i*delta_t_plot,1).acceleration_cost_reduction(1,1) <= 2)
%                 acceleration_cost_reduction(:,i) = agent_track_history(1:2,i);
%            end
%        end
%        
%        if(size(agent_data(i*delta_t_plot,1).position_cost_reduction,2) > 1)
%            if(agent_data(i*delta_t_plot,1).position_cost_reduction(1,2) > 1) && (agent_data(i*delta_t_plot,1).position_cost_reduction(1,1) <= 2)
%                 position_cost_reduction(:,i) = agent_track_history(1:2,i);
%            end
%        end       
%        
%        
% %        if(agent_data(i*delta_t_plot,1).position_cost_reduction(1,1) == 2)
% %            position_cost_reduction(:,i) = NaN;
% %        end
%    end
%    
%     plot(acceleration_cost_reduction(2,:), acceleration_cost_reduction(1,:), 'm', 'LineWidth',10);
%     plot(position_cost_reduction(2,:), position_cost_reduction(1,:), 'b', 'LineWidth',6);
% 
% end
% 
% 



for i=1:l-1
    for k=1:size(agent_data,2)
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
handle_holder = [];
i_index_from_timesteps = round(settings.t_snapshot*1/settings.dt);
text_lines_per_column = settings.text_lines_per_column;



%%
for i_index=1:size(i_index_from_timesteps,2)
    i = i_index_from_timesteps(i_index);
%         delete(handle_holder);
    count = 1+ (i-1)/delta_t_plot;
%     text(text_time_stamps_position(2)+delta_text_x*floor((i_index-1)/text_lines_per_column), text_time_stamps_position(1)-delta_text*mod(i_index-1,text_lines_per_column),strcat("$",num2str(i_index),": t=", num2str(round(i*settings.dt)), "s$"),'FontSize',info_text_font_size , 'Interpreter','latex'); 
    for k=1:size(agent_data,2)
        if(k==1) 
            agent_ = agent_data(i,k);
            course = atan2(agent_.eta_dot(2), agent_.eta_dot(1));
            agent_.eta(3) = course;
%             handle_ = plot_ts_from_struct(agent_,cmap(i,:),3.5);
            handle_ = plot_os(agent_data(i,k).eta, cmap(i,:),4.5); % Eta
            text_offset_x = 5;
            text_offset_y = 5;
            figure_axis =  settings.axis;
%             if(within_figure_axis(flip(agent_data(i,k).eta(1:2,1)),figure_axis))
%                 text(agent_data(i,k).eta(2)+text_offset_x,agent_data(i,k).eta(1)+text_offset_y, strcat(num2str(i_index)), 'color', settings.agent_color{1,k},'FontSize',vessel_text_font_size , 'Interpreter','latex');
%             end
        else
%             handle_ = plot_os(agent_data(i,k).eta, settings.agent_color{k},2); % Eta
%             handle_ = plot_ts_from_struct(agent_data(i,k),'k',2);
            agent_ = agent_data(i,k);
%             course = atan2(agent_.eta_dot(2), agent_.eta_dot(1));
%             agent_.eta(3) = course;
%             agent_.eta(3) = atan2(agent_.eta_dot(2), agent_.eta_dot(1));
            handle_ = plot_ts_from_struct(agent_,cmap(i,:),3.5);

%             text(agent_data(i,k).eta(2), agent_data(i,k).eta(1), strcat("t=",num2str(round(i*settings.dt))));
            text_offset_x = 4;
            text_offset_y = 4;
%             text(agent_data(i,k).eta(2)+offset_x, agent_data(i,k).eta(1)+offset_y, strcat(num2str(i_index)), 'color', 'r');
        end
    end

end
%%







%%
if(0)%plot_from_rosbag)
    yamlStruct = ReadYaml('/home/emilht/NTNU-aFerry/workspace/src/ros_af_sim/scenarios/CBF_december_experiment_01.yaml');
    
    for i=1:size(yamlStruct.own_ship.waypoints,2)-1
        plot( [yamlStruct.own_ship.waypoints{1,i+1}.E,yamlStruct.own_ship.waypoints{1,i}.E],[yamlStruct.own_ship.waypoints{1,i+1}.N,yamlStruct.own_ship.waypoints{1,i}.N], 'r--');
    end


%     for i=1:size(yamlStruct.own_ship.waypoints,2)-1
%         wp1 = [yamlStruct.own_ship.waypoints{i}.N;yamlStruct.own_ship.waypoints{i}.E];
%         wp2 = [yamlStruct.own_ship.waypoints{i+1}.N;yamlStruct.own_ship.waypoints{i+1}.E];
%        plot( [wp1(2), wp2(2)], [wp1(1), wp2(1)], 'r' )
%        plot( [wp1(2), wp2(2)], [wp1(1), wp2(1)], '*r' )
%     end
% else
%     plot( agents(1).wp(2,:), agents(1).wp(1,:), 'r')
%     
% end
if(scenario>0)
   switch scenario
       case 1
           p1 = [-350, -170];
           p2 = [-455, -170];
           t1 = "Det røde murbygget";
           t2 = "Nordvest hjørne av Rød sjøbu";
           t1_offset = -60;
           t2_offset = -100;
       case 2
           p1 = [-540, -460];
           p2 = [-614, -430];
           t1 = "Lagerhall, spiss tak";
           t2 = "Oransje Sjøbu";
           t1_offset = -20;
           t2_offset = -20;
       case 3
           p1 = [-495, -400];
           p2 = [-570, -360];
           t1 = "Innovation skilt";
           t2 = "Gate Mellom sjøbu og lavt bygg";
           t1_offset = -20;
           t2_offset = -40;
       case 4
           p1 = [-402, -253];
           p2 = [-466, -223];
           t1 = "Hjørne Molo/Land";
           t2 = "Hjørne Molo/Land";
           t1_offset = -30;
           t2_offset = -30;
       case 5
           p1 = [-495, -400];
           p2 = [-570, -360];
           t1 = "Innovation skilt";
           t2 = "Gate Mellom sjøbu og lavt bygg";
           t1_offset = -40;
           t2_offset = -60;
       case 6
           p1 = [-350, -170];
           p2 = [-455, -170];
           t1 = "Det røde murbygget";
           t2 = "Nordvest hjørne av Rød sjøbu";
           t1_offset = -40;
           t2_offset = -60;
       case 7
           p1 = [-495, -400];
           p2 = [-570, -360];
           t1 = "Innovation skilt";
           t2 = "Gate Mellom sjøbu og lavt bygg";
           t1_offset = -60;
           t2_offset = -100;
   
   
   end

plot(p1(2), p1(1), 'r*');
plot(p2(2), p2(1), 'r*');
plot([p1(2), p2(2)], [p1(1), p2(1)], 'r--');
text(p1(2)+t1_offset, p1(1)+10, t1,'FontSize',16 , 'Interpreter','latex'); 
text(p2(2)+t2_offset, p2(1)-10, t2,'FontSize',16 , 'Interpreter','latex'); 

end
end


function within_axis = within_figure_axis(p,axis)
within_axis = (p(1)>axis(1)) && (p(1)<axis(2)) && (p(2)>axis(3))&& (p(2)<axis(4));
end


function draw_custom_legend(settings,vessel)


if(isfield(settings,'legend_position'))
     % Make Custom Legend
    scale = (settings.axis(2)-settings.axis(1))/760*0.9;

    legend_size = settings.legend_size*scale;
    legend_position = settings.legend_position;
    
    graph_handle =[];
   
    handle_ = rectangle('Position',[legend_position(2), legend_position(1),legend_size(1),legend_size(2)],'FaceColor',[1 1 1],'EdgeColor',[0 0 0],'LineWidth',1);
    graph_handle = [graph_handle, handle_];
    
    % Own Ship Vessel Legend
    vessel.eta(1:2) = legend_position + [legend_size(2)-15*scale; 25*scale];
    vessel.eta(3,1) = -pi/2;
    handle_ = plot_os_from_struct(vessel,'b',6.0*scale);
    graph_handle = [graph_handle, handle_];
%     thandle = text(vessel.eta(2)+25, vessel.eta(1), settings.os_1_text, 'fontsize',13);
    thandle = text(vessel.eta(2)+25*scale, vessel.eta(1), 'Own ship', 'fontsize',13);
%     thandle = text(vessel.eta(2)+25, vessel.eta(1), 'OS 1', 'fontsize',13);
    graph_handle = [graph_handle, handle_,thandle];
        
    
    % Target Ship Vessel Legend
    vessel.eta(1:2) = legend_position + [legend_size(2)-42*scale; 25*scale];
    vessel.eta(3,1) = -pi/2;
    handle_ = plot_ts_from_struct(vessel,[1,0,0],6.0*scale);
    thandle = text(vessel.eta(2)+25*scale, vessel.eta(1), 'Target ships', 'fontsize',13);
    graph_handle = [graph_handle, handle_,thandle];
    
    
    % Static obstacles legend
    position = legend_position + [legend_size(2)-70*scale; 25*scale];
    handle_ = legend_filled_square(position, 30*scale, get_rgb('land'));
    thandle = text(position(2) +25*scale, position(1), 'Static obstacles', 'fontsize',13);
    graph_handle = [graph_handle, handle_,thandle];
    
    if(vessel.gnc==4) % MPC based method
        line_delta = [-23*scale;0];
        
        % Nominal Path
        line_start = legend_position + [legend_size(2)-97*scale; 10*scale];
        line_end   = legend_position + [legend_size(2)-97*scale; 40*scale];
%         line = [line_start, line_end];
%         position = 0.5*(line_start+line_end);
%         plot(position(2), position(1,:),'color',get_rgb('red'), 'linewidth',1,'Marker','*' )
%         text(line_start(2)+40, line_start(1), 'Waypoints', 'fontsize',13);%         line_start = line_start + line_delta;
%         line_end   = line_end + line_delta;
        line = [line_start, line_end];
        plot(line(2,:), line(1,:),'--','color',get_rgb('light_grey'), 'linewidth',2)
        text(line_start(2)+40*scale, line_start(1), 'Nominal path', 'fontsize',13);
        
        
%         % Reference Trajectory
%         line_start = line_start + line_delta;
%         line_end   = line_end + line_delta;
%         line = [line_start, line_end];
%         plot(line(2,:), line(1,:),'color',get_rgb('red'), 'linewidth',2)
%         text(line_start(2)+40, line_start(1), 'Reference trajectory', 'fontsize',13);
% 
%         % LOS guidance trajectory
%         line_start = line_start + line_delta;
%         line_end   = line_end + line_delta;
%         line = [line_start, line_end];
%         plot(line(2,:), line(1,:),'color',get_rgb('orange'), 'linewidth',2)
%         text(line_start(2)+40, line_start(1), 'LOS guidance trajectory', 'fontsize',13);
% 
%         % Optimal Trajectory
%         line_start = line_start + line_delta;
%         line_end   = line_end + line_delta;
%         line = [line_start, line_end];
%         plot(line(2,:), line(1,:),'color','c', 'linewidth',2)
%         text(line_start(2)+40, line_start(1), 'Optimal trajectory', 'fontsize',13);
        
        % Acceleration Cost Recution Window
        line_start = line_start + line_delta;
        line_end   = line_end + line_delta;
        line = [line_start, line_end];
        plot(line(2,:), line(1,:),'color','m', 'linewidth',8)
        text(line_start(2)+40*scale, line_start(1), 'ACRP window', 'fontsize',13);
        
        % Tracking cost reduction window
        line_start = line_start + line_delta;
        line_end   = line_end + line_delta;
        line = [line_start, line_end];
        plot(line(2,:), line(1,:),'color',get_rgb('blue'), 'linewidth',6)
        text(line_start(2)+40*scale, line_start(1), 'TCRP window', 'fontsize',13);
        
    end
    
end
end

