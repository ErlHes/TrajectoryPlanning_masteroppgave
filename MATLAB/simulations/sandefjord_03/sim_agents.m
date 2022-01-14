


transit_01_to_03 = [
    1131,754;...
    1040,820;...
    950,811;...
    700,680;...
    400,600;...
    270,630]';

% transit_01_to_03 = [
%     1131,754;...
%     1040,820;...
%     950,811;...
%     700,680;...
%     400,600;...
%     270,630;...
%     241,668]';

transit_01_to_03_velocity = [0.8,2.4,2.4,2.4,2.4,1.5,0.6];
        



TS1_path = [-400,800;...
            -50,500;...
            370,520;...
            550,670;...
            585,775;...
            526,810
            ]';
        
TS2_path = [555,830;...
            577,786;...
            600,755;...
            740,790;...
            950,1100;...
            920,1140;...
            880,1100;...
            800,1010;...
            ]';
TS2_vel = ones(1,size(TS2_path,2))*3;
TS2_vel(1) = 0.17;
TS2_vel(end-3:end) = 0.4;


% TS3_path = [920,310;...
%             940,410;...
%             950,430;...
%             840,580;...
%             100,450;...
%            -400,450;...
%             ]';
%         
        
TS3_path = [920,310;...
            945,410;...
            950,430;...
            840,580;...
            950,1100;...
            920,1140;...
            880,1100;...
            860, 1065;...
            ]';
        
TS3_vel = ones(1,size(TS3_path,2))*3;
TS3_vel(1) = 1.5;
TS3_vel(end-3:end) = 0.5;
TS3_vel(end) = 0.05;

      
% Define agent structs in array
% get_agent(id, eta_0, nu_0,[l,b,], model, gnc,speed; waypoints,visible)
 agents = [     
 get_agent(101, [1124.0, 757.0, deg2rad(170)]', [0.6,0,0]',[5,3], 2, 2,transit_01_to_03_velocity, transit_01_to_03,1)...
 get_agent(102, [-450, 680, deg2rad(-30)]', [3,0,0]',[5,3], 2, 2,ones(1,size(TS1_path,2))*3, TS1_path,1)...
,get_agent(103, [555,830, deg2rad(-90)]', [1,0,0]',[5,3], 2, 2,TS2_vel, TS2_path,1)...
,get_agent(104, [920,310, deg2rad(90)]', [1,0,0]',[5,3], 2, 2,TS3_vel, TS3_path,1)...
];