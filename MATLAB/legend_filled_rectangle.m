function handle = legend_filled_rectangle(position, length, color)
p1 = position + [0.07*length;0.5*length];
p2 = position + [-0.07*length;0.5*length];
p3 = position - [0.07*length;0.5*length];
p4 = position - [-0.07*length;0.5*length];
trc = [p1,p2,p3,p4];
handle = fill(trc(2,:) , trc(1,:),color,'LineStyle','none' );
end