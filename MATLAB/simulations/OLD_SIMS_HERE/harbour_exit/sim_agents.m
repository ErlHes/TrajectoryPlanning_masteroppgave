
        %get_agent(id, eta_0, nu_0,[l,b,], model, gnc,speed; waypoints,visible)

agents = [
get_agent(101, [80,4,0]', [1,0,0]',[5,3], 1, 1,[1,1], [80,4; 228,-48; 276,-25; 510,180]',1)...
,get_agent(102, [390,60,pi]', [1,0,0]',[5,3], 1, 1,[1,1], flip([80,4; 228,-48; 276,-25; 510,180]',2),1)...
,get_agent(103, [250,-54,pi]', [1,0,0]',[5,3], 1, 1,[1,1],[250,-54; 198,-47; 135,-25; 99, 21]',1)...
% ,get_agent(104, [150,0,pi]', [1,0,0]',[5,3], 2, 2,[1,1], [100,0;0,0]',1)...
];
              
              
              
              
