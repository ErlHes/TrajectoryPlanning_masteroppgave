% This scripts described the static obstacles and sea-markings of the
% environment that is given the name panama.
% origin = [-550,-300]';
origin = [0,0]';


nordlandet_1 = [
    10,160;...
    151,300;...
    180,1200;...
    -600, 1200;...
    -650,400;...
    -550,230;...
    -470,170;...
    -70,130;...    
    ]';

sorlandet_1 = [
    -140,0;...
    -700,40;...
    -800,-30;...
    -900,-1000;...
    -500,-1000;...
    -200,-350;...
    -120,-90;...
    ]';


nord_vest_landet_1 = [
  0,-400;...
  -60,-400;...
  -120,-500;...
  -300,-1000;...
  200,-1000;...
  200,-420;...
  
]';

nord_vest_landet_2 = [
  200,-420;...
  200,-1000;...
  400,-1000;...
  300,-410;...
  
]';
nord_vest_landet_3 = [
  300,-410;...
  400,-1000;...
  800,-1000;...
  960,-300;...
  900,-155;...
  600,-110;...
]';

nord_vest_landet_4 = [
  800,-1000;...
  2000,-1000;...
  1435,-435;...
  960,-300;...
]';

nord_vest_landet_5 = [
  1435,-435;...
  2000,-1000;...
  2000,1200;...
  1440,-335;...
]';
nord_ost_landet_1 = [
  1440,-335;...
  2000,1200;...
  1300,1200;...
  1100,120;...
]';
nord_ost_landet_2 = [
  1100,120;...
  1300,1200;...
  500,1200;...
  400,400;...
  600,50;...
  750,-20;...
  880,-35;...
]';

nordholmen = [
  320,500;...
  350,550;...
  370,650;...
  340,700;...
  310,650;...
  290,550;...
]';



separator= [nan;nan];
static_obs = [separator,nordlandet_1,...
              separator,sorlandet_1...
              separator,nord_vest_landet_1,...
              separator,nord_vest_landet_2,...
              separator,nord_vest_landet_3,...
              separator,nord_vest_landet_4,...
              separator,nord_vest_landet_5,...
              separator,nord_ost_landet_1,...
              separator,nord_ost_landet_2,...
              separator,nordholmen,separator...
              ];

          
clear('separator','nordlandet_1',...
      'separator','sorlandet_1',...
      'separator','nord_vest_landet_1',...
      'separator','nord_vest_landet_2',...
      'separator','nord_vest_landet_3',...
      'separator','nord_vest_landet_4',...
      'separator','nord_vest_landet_5',...
      'separator','nord_ost_landet_1',...
      'separator','nord_ost_landet_2',...
      'separator','nordholmen')


% figure(600)
% clf(600)
% hold on;
% grid on;
% axis('equal');
% plot_static_obs(static_obs,10);

