function angle = wrap_zero_to_2pi(angle)
while angle < 0
    angle = angle + 2*pi;
end
while angle >= 2*pi
    angle = angle - 2*pi;
end