function rgb = get_rgb(color)


switch color
    case 'dark_red',        rgb = [153,0,0]/256;
    case 'red',             rgb = [1,0,0];
    case 'orange',          rgb = [255,128,0]/256;
    case 'green' ,          rgb = [0,255,0]/256;
    case 'green_feature',   rgb = [88,175,47]/256;
    case 'pale_green',      rgb = [167,212,137]/256;
    case 'forest_green',    rgb = [11,102,35]/256;
    case 'shamrock_green',  rgb = [3,172,19]/256;
    case 'turquoise',       rgb = [0,255,255]/256;
    case 'blue',            rgb = [0,0,1];
    case 'violet' ,         rgb = [153,0,153]/256;
    case 'light_grey',      rgb = [0.7,0.7,0.7];
    case 'dark_grey',       rgb = [0.4,0.4,0.4];
    case 'beige',           rgb = [249,228,183]/256;
    case 'yellow',          rgb = [255,255,0]/256;
        
    % Object specific color
    case 'land',            rgb = [240,228,221]/256;
    case 'water',           rgb = [201,232,253]/256;
    case 'blue_feature',    rgb = [18,114,174]/256;
        
    % COLREGs classification
    case 'colregs_ot',      rgb = [190,26,17]/256;
    case 'colregs_safe',    rgb = [0,222,51]/256;
        
        
    otherwise
        rgb = [0 0 0];
        disp('Color not yet added to get_rgb() function');

end