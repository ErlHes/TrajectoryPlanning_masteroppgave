% This scripts described the static obstacles and sea-markings of the
% environment that is given the name panama.


south_canal_bank = [
    40,-370;...
    0,-170;...
    0,-60;...
    -25,-60;...
    -25,0;...
    0,0;...
    0,200;...
    -50,200;
    -50,-370;...
    ]';
north_east_canal_bank = [
  60,200;...
  60,-70;...
  80,-90;...
  85,-85;...
  100,200;...
]';

north_west_canal_bank = [
    100,-200;...
    140,-370;...
    160,-370;...
    120,-200;...
    150,-150;...
    130,-150;...
]';

north_quay_molo_west = [
    450,-100;...
    500,-370;...
    600,-370;...
    600,-250;...
    500,-250;...
    465,-95;...
]';

nort_quay_bank = [
    600,-140;...
    600,200;...
    520,200;...
    520,100;...
    550,100;...
    550,50;...
    520,50;...
    550,-140;...
]';
north_quay_molo_east = [
    440,0;...
    450,5;...
    450,200;...
    440,200;...
]';

separator= [nan;nan];
static_obs = [separator,south_canal_bank,separator,north_east_canal_bank,separator,...
              north_west_canal_bank,separator,north_quay_molo_west,separator,...
              nort_quay_bank,separator,north_quay_molo_east,separator];
          
% plot_simulation