
function draw_custom_legend(settings,vessel)


if(isfield(settings,'legend_position'))
     % Make Custom Legend
    scale = (settings.axis(2)-settings.axis(1))/760*0.8;

    legend_size = settings.legend_size*scale;
    legend_position = settings.legend_position;
    
    graph_handle =[];
   
    handle_ = rectangle('Position',[legend_position(2), legend_position(1),legend_size(1),legend_size(2)],'FaceColor',[1 1 1],'EdgeColor',[0 0 0],'LineWidth',1);
    graph_handle = [graph_handle, handle_];
    
    % Own Ship Vessel Legend
    vessel.eta(1:2) = legend_position + [legend_size(2)-15*scale; 25];
    vessel.eta(3,1) = -pi/2;
    handle_ = plot_os_from_struct(vessel,'b',6.0*scale);
    graph_handle = [graph_handle, handle_];
%     thandle = text(vessel.eta(2)+25, vessel.eta(1), settings.os_1_text, 'fontsize',13);
    thandle = text(vessel.eta(2)+25, vessel.eta(1), 'Own ship', 'fontsize',13);
%     thandle = text(vessel.eta(2)+25, vessel.eta(1), 'OS 1', 'fontsize',13);
    graph_handle = [graph_handle, handle_,thandle];
        
    
    % Target Ship Vessel Legend
    vessel.eta(1:2) = legend_position + [legend_size(2)-42*scale; 25];
    vessel.eta(3,1) = -pi/2;
    handle_ = plot_ts_from_struct(vessel,[1,0,0],6.0*scale);
    thandle = text(vessel.eta(2)+25, vessel.eta(1), 'Target ships', 'fontsize',13);
    graph_handle = [graph_handle, handle_,thandle];
    
    
    % Static obstacles legend
    position = legend_position + [legend_size(2)-70*scale; 25];
    handle_ = legend_filled_square(position, 30*scale, get_rgb('land'));
    thandle = text(position(2) +25, position(1), 'Static obstacles', 'fontsize',13);
    graph_handle = [graph_handle, handle_,thandle];
    
    if(vessel.gnc==4) % MPC based method
        line_delta = [-23*scale;0];
        
        % Nominal Path
        line_start = legend_position + [legend_size(2)-97*scale; 10];
        line_end   = legend_position + [legend_size(2)-97*scale; 40];
        line = [line_start, line_end];
        position = 0.5*(line_start+line_end);
        plot(position(2), position(1,:),'color',get_rgb('red'), 'linewidth',1,'Marker','*' )
        text(line_start(2)+40, line_start(1), 'Waypoints', 'fontsize',13);
        
        
        % Reference Trajectory
        line_start = line_start + line_delta;
        line_end   = line_end + line_delta;
        line = [line_start, line_end];
        plot(line(2,:), line(1,:),'color',get_rgb('red'), 'linewidth',2)
        text(line_start(2)+40, line_start(1), 'Reference trajectory', 'fontsize',13);

        % LOS guidance trajectory
        line_start = line_start + line_delta;
        line_end   = line_end + line_delta;
        line = [line_start, line_end];
        plot(line(2,:), line(1,:),'color',get_rgb('orange'), 'linewidth',2)
        text(line_start(2)+40, line_start(1), 'LOS guidance trajectory', 'fontsize',13);

        % Optimal Trajectory
        line_start = line_start + line_delta;
        line_end   = line_end + line_delta;
        line = [line_start, line_end];
        plot(line(2,:), line(1,:),'color','c', 'linewidth',2)
        text(line_start(2)+40, line_start(1), 'Optimal trajectory', 'fontsize',13);
        
        % Acceleration Cost Recution Window
        line_start = line_start + line_delta;
        line_end   = line_end + line_delta;
        line = [line_start, line_end];
        plot(line(2,:), line(1,:),'color','g', 'linewidth',2)
        text(line_start(2)+40, line_start(1), 'ACRP window', 'fontsize',13);
        
        % Tracking cost reduction window
        line_start = line_start + line_delta;
        line_end   = line_end + line_delta;
        line = [line_start, line_end];
        plot(line(2,:), line(1,:),'color',get_rgb('blue'), 'linewidth',2)
        text(line_start(2)+40, line_start(1), 'TCRP window', 'fontsize',13);
        
    end
    
end
end