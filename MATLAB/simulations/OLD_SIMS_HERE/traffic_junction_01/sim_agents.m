
%get_agent(id, eta_0, nu_0,[l,b,], model, gnc,speed; waypoints,visible)

agents = [
get_agent(101, [30,400,-pi/2-0.01]', [1.7,0,0]',[5,3], 1, 1,[1.7,1.7], [30,400;30,-1000]',1)...
,get_agent(102, [30,330,-pi/2]', [1,0,0]',[5,3], 1, 1,[1,1], [30,400;30,-1000]',1)...
,get_agent(103, [350,-180, -pi]', [1.2,0,0]',[5,3], 1, 1,[1.2,1.2], [600,-180; 75,-180;30,-130;30,1000]',1)...
];
