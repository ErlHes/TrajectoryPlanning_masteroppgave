% This scripts described the static obstacles and sea-markings of the
% environment that is given the name panama.


west_bank = [
    0,-20;...
    0,-100;...
    500,-100;...
    500, -20;...
    ]';
east_bank = [
  0,200;...
  0,100;...
  500,20;...
  500,120;...
]';

west_bank_2 = [
    0,-100;...
    0,-20;...
    -500,-20;...
    -500,-100;...
]';

east_bank_2 = [
    0,100;...
    0,200;...
    -500, 200;...
    -500, 100;...
]';
separator= [nan;nan];
static_obs = [separator,west_bank,separator,east_bank,separator,west_bank_2,separator,east_bank_2, separator];
static_obs = flip(static_obs,1);
static_obs(1,:) = -static_obs(1,:);
%           static_obs = [];
% figure(10)
% clf(10)
% hold on;
% grid on;
% axis('equal');
% plot_static_obs(static_obs,10);