blokk1 = [
    40,-100;...
    40, 0;...
    ]';

blokk2 = [
    40, 0;...
    20, 0;...
    20, 20;...
    ]';

blokk3 = [
    20, 50;...
    20, 100;...
    ]';


separator= [nan;nan];
static_obs = [separator,blokk1,...
              separator,blokk2,...
              separator,blokk3,separator];