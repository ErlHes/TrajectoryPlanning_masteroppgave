function move = shortest_angle_path(angle, angle_d)
    diff = mod(angle_d - angle + pi, 2*pi) - pi;
    if diff < - pi
        move = diff + 2*pi;
    else
        move = diff;
end