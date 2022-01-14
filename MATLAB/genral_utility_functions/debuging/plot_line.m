function handle = plot_line(point, angle, length , fig_number, color)

tangent = [cos(angle);sin(angle)];
line = [point, point + tangent*length];
figure(fig_number)
hold on;
handle = plot(line(2,:), line(1,:), 'color' , color);

end