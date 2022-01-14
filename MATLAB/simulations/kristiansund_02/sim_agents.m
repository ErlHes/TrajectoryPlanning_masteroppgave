

transit_01_to_03 = [
                   -175.0, -290.0;...
                   -157.0, -285.0;...
                   7.0, 9.0;...
                   200.0, 200.0;...
                   350.0, 450.0;...
                   380,450;...
                   400.0,450.0]';
transit_01_to_03_velocity = [0.3,2.4,2.4,2.4,2.4,2.0,0.6];
        



TS1_path = [-600,120;...
            -150,82;...
            840,-70;...
            ]';
        
TS2_path = [300,1200;...
            250,550;...
            350,100;...
            640,-30;...
            1140,-130;...
            ]';
            
% Define agent structs in array
%get_agent(id, eta_0, nu_0,[l,b,], model, gnc,speed; waypoints,visible)
 agents = [     
 get_agent(101, [-168.0, -290.0, deg2rad(0)]', [0.1,0,0]',[5,3], 2, 2,transit_01_to_03_velocity, transit_01_to_03,1)...
,get_agent(102, [-270,120, deg2rad(-5)]', [1,0,0]',[5,3], 2, 2,ones(1,size(TS1_path,2))*1.5, TS1_path,1)...
,get_agent(103, [300,1000, deg2rad(-90)]', [1,0,0]',[5,3], 2, 2,ones(1,size(TS2_path,2))*1.5, TS2_path,1)...
];