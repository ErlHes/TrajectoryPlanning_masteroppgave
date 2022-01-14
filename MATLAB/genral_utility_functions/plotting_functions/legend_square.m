function handle = legend_square(position, length, option, color)
p1 = position + [0.25*length;0.5*length];
p2 = position + [-0.25*length;0.5*length];
p3 = position - [0.25*length;0.5*length];
p4 = position - [-0.25*length;0.5*length];
trc = [p1,p2,p3,p4,p1];
handle = plot(trc(2,:) , trc(1,:),  option, 'color', color, 'linewidth',1.3);
end