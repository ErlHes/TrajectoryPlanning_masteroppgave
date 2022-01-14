function angle = wrap_plus_minus_pi(angle)
for i=1:max(size(angle))
    while angle(i) < pi
        angle(i) = angle(i) + 2*pi;
    end
    while angle(i) >= pi
        angle(i) = angle(i) - 2*pi;
    end
end
end % function