function handle = plot_arrow(p_start, p_end, head_width, color, linewidth)
dp = p_end-p_start;
dp = dp/norm(dp,2);


p_right = p_end + rot2(3*pi/4)*dp*head_width*sqrt(2);
p_left = p_end + rot2(-3*pi/4)*dp*head_width*sqrt(2);

arrow = [p_start, p_end,p_right, p_left, p_end];
handle = plot(arrow(2,:), arrow(1,:), 'color', color, 'linewidth' , linewidth);


end