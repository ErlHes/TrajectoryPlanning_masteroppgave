function time_string = time_string_from_seconds(seconds)

seconds = round(seconds);
hours = floor(seconds/3600);
minutes = floor((seconds-hours*3600)/60);
seconds = seconds - hours*3600 - minutes*60;

time_string = strcat( num2str(seconds), " seconds");
if(minutes > 0)
    time_string = strcat(num2str(minutes), " minutes and ", time_string);
end
if(hours == 1)
    time_string = strcat(num2str(hours), " hour, ", time_string);
elseif(hours > 1)
    time_string = strcat(num2str(hours), " hours, ", time_string);
end

end