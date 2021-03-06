
%get_agent(id, eta_0, nu_0,[l,b,], model, gnc,speed; waypoints,visible)

agents = [
get_agent(101, [0,0,pi/4]', [1,0,0]',[5,3], 1, 1,[1,1], [0,0;200,200]',1)...
,get_agent(102, [200,0,3*pi/4]', [1,0,0]',[5,3], 1, 1,[1,1], [200,0;0,200]',1)...
,get_agent(103, [200,200, -3*pi/4]', [1,0,0]',[5,3], 1, 1,[1,1], [200,200;0,0]',1)...
,get_agent(104, [0,200, -pi/4]', [1,0,0]',[5,3], 1, 1,[1,1], [0,200;200,0]',1)...
,get_agent(105, [300,100, pi]', [1,0,0]',[5,3], 1, 1,[1.5,1.5], [300,100;-100,100]',1)...
,get_agent(106, [100,-100, pi/2]', [1,0,0]',[5,3], 1, 1,[1.5,1.5], [100,-100;100,300]',1)...
,get_agent(107, [-100,100, 0]', [1,0,0]',[5,3], 1, 1,[1.5,1.5], [-100,100;300,100]',1)...
,get_agent(108, [100,300, -pi/2]', [1,0,0]',[5,3], 1, 1,[1.5,1.5], [100,300;100,-100]',1)...
];