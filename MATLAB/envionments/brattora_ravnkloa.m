% This script defines the brattøra, ravnkloa environment.



% Static Environment 
env.origin =  [0;0];
% Load from struct
static_env = load('/home/emilht/MATLAB/full_pipeline_colav_development/object_simulator/environments/static_env_trondheim_harbour.mat');
static_env = static_env.static_env_trondheim_harbour;




% Plot static environment with same figure handle as visualization uses
figure(600)
clf(600)
hold on;
axis('equal');
axis([-350,350,-150,200]);

ax = skinny_whitespace(gca);
% 
% ax = gca;
% outerpos = ax.OuterPosition;
% ti = ax.TightInset; 
% left = outerpos(1) + ti(1);
% bottom = outerpos(2) + ti(2);
% ax_width = outerpos(3) - ti(1) - ti(3);
% ax_height = outerpos(4) - ti(2) - ti(4);
% ax.Position = [left bottom ax_width ax_height];


c = [0.7,0.7,0.7];
coord_accumulated = nan(0,2);
for i=1:size(static_env,1)
    coord = static_env{i,:};
%      pgon = polyshape(coord(:,2), coord(:,1));
     fill(coord(:,2), coord(:,1),c)
%      p = plot(pgon);
     coord_accumulated = [coord_accumulated;nan,nan;coord];
end

env.polygons = coord_accumulated;

plot([0,-47],[0,104],'linewidth',2);
hold off;


% Paths with waypoints.
% Lines across the canal, where the waypoints are on the line.



l1 = [-244, -596;...
      -285, -590]';
  
l2 = [-188, -456;...
      -236, -426]';
  
l3 = [-132, -343;...
      -180, -308]';
        
l4 = [ -9, -169;...
      -54, -135]';

l5 = [ 79, -57;...
       14, -29]';
   
l6 = [148, 46;...
       84, 77]';
   
l7 = [167, 114;...
      106, 123]';
 
l8 = [191, 337;...
      139, 341]';
  
l9 = [185, 540;...
      143, 534]';
  
l10 = [127, 658;...
       84 , 663]';
   
l11 = [124, -30;...
       134, -16]';
   
l12 = [196, -52;...
       206, -36]';
   
l13 = [281, -66;...
       228, -38]';

l14 = [281, -66;...
       310, -30]';
   
l15 = [450, -180;...
       530,  -20]';

waypoint_lines = [l1,l2,l3,l4,l5,l6,l7,l8,l9,l10,l11,l12,l13,l14,l15];
% waypoint_lines{1,1} = l1;
% waypoint_lines{2,1} = l2;
% waypoint_lines{3,1} = l3;
% waypoint_lines{4,1} = l4;
% waypoint_lines{5,1} = l5;
% waypoint_lines{6,1} = l6;
% waypoint_lines{7,1} = l7;
% waypoint_lines{8,1} = l8;
% waypoint_lines{9,1} = l9;
% waypoint_lines{10,1} = l10;
% waypoint_lines{11,1} = l11;
% waypoint_lines{12,1} = l12;
% waypoint_lines{13,1} = l13;
% waypoint_lines{14,1} = l14;
% waypoint_lines{15,1} = l15;
env.waypoint_lines = waypoint_lines;


paths = nan(6,10);
paths(1,:) = 1:10;
paths(2,:) = flip(paths(1,:));
paths(3,:) = [1:5, 11:15];
paths(4,:) = flip(paths(3,:));
paths(5,:) = [10:-1:6, 11:15];
paths(6,:) = flip(paths(5,:));
env.paths = paths;



% 
figure(600)
hold on;

for i=1:size(waypoint_lines,2)/2
   
    plot([waypoint_lines(2,2*i-1),waypoint_lines(2,2*i)],[waypoint_lines(1,2*i-1),waypoint_lines(1,2*i)],'r', 'linewidth',2);
    if(i==14)
        text(waypoint_lines(2,2*i), waypoint_lines(1,2*i),num2str(i), 'FontSize', 30);
    else
        text(waypoint_lines(2,2*i-1), waypoint_lines(1,2*i-1),num2str(i), 'FontSize', 30);
    end
end