function handle_ = plot_trajectory(agent, option)
    dash_orig_x = agent.eta(2);
    dash_orig_y = agent.eta(1);
    dash_angle = agent.eta(3) + pi/2;
    dash_direction = [sin(dash_angle);cos(dash_angle)];
    dash_orig = [dash_orig_x;dash_orig_y];
    if isfield(agent,'nu')
        dash_thickness = agent.nu(1)*2 + 0.1;
%         dash_end = dash_orig + agent.nu(1)/2 *dash_direction;
    else
        dash_thickness = agent.vel*2 + 0.1;
%         dash_end = dash_orig + agent.vel/2 * dash_direction;
    end
    dash_start = dash_orig - 0.5 * dash_direction;
    dash_end = dash_orig + 0.5 * dash_direction;
    dash = [dash_start,dash_end];
    handle_ = mapshow(dash(1,:),dash(2,:),'Color',option,'LineWidth',dash_thickness);
end