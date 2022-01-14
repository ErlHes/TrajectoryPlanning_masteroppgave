function out = saturate(in, lower, upper)
    out = min(upper, max(lower,in));
end

