run('/home/emilht/MATLAB/genral_utility_functions/COLREGs_protocol_evaluation/colregs_protocol_parameters.m');


%% Test 1 

S = CPE_give_way_crossing_score(pi, -pi/2);





%% Test 2
range = CPE_parameters.safety.ranges(1):0.1:CPE_parameters.safety.ranges(end);
range_cost = zeros(1,length(range));
for i = 1:size(range,2)
   range_cost(i) = CPE_safety_from_range(range(i),CPE_parameters.safety);
end

figure(99)
clf(99)
hold on;
plot(range, range_cost);
title('range cost');


%% Test 3
alpha_bearings = -pi:0.1:pi;
pose_cost = zeros(1,length(alpha_bearings));

for i = 1:length(alpha_bearings)
   pose_cost(i) = CPE_safety_from_pose(pi/2, alpha_bearings(i), CPE_parameters.safety);
    
end
figure(98)
clf(98)
hold on;
title('pose cost');
plot(alpha_bearings, pose_cost);



%% Test 4

alpha_bearings = -pi:0.1:pi;
pose_cost = zeros(length(alpha_bearings));

for i = 1:length(alpha_bearings)
for j = 1:length(alpha_bearings)
   pose_cost(i,j) = CPE_safety_from_pose(alpha_bearings(i), alpha_bearings(j), CPE_parameters.safety);
end   
end

figure(98)
clf(98)
hold on;
title('Safety score from pose');
xlabel('alpha bearing');
ylabel('beat bearing');
zlabel('pose cost');
[X,Y] = meshgrid(alpha_bearings, alpha_bearings);
surf(X,Y,  pose_cost);




