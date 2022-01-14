% This scripts described the static obstacles and sea-markings of the
% environment that is given the name panama.


west_bank = [
    0,0;...
    0,-100;...
    500,-100;...
    500, 0;...
    ]';
east_bank = [
  0,200;...
  0,100;...
  500,20;...
  500,120;...
]';



separator= [nan;nan];
static_obs = [separator,west_bank,separator,east_bank,separator];
          
% figure(10)
% clf(10)
% hold on;
% grid on;
% axis('equal');
% plot_static_obs(static_obs,10);