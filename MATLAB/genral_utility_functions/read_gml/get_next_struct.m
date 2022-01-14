function [ret_i, ret_struct] = get_next_struct(content, i)

    ret_struct = struct([]);
    ret_i = i;
    
    if i < 1
        
        return
        
    end
    
    while true
        
        name = content{i};           
        
        if name ~= ']'
            
            value = content{i+1};
            
            if value == '['
                
                [i, substruct] = get_next_struct(content, i+2);
                
                if i < 1
                    return;
                else
                    
                    try
                        ret_struct(1).(name) = [ret_struct(1).(name), substruct];
                    catch
                        ret_struct(1).(name) = substruct;
                    end
                    
                end
               
            else
                
                [num, ~, errmsg] = sscanf(value, '%f');
                
                if numel(errmsg) == 0
                    value = num;
                end
            
                ret_struct(1).(name) = value;
                i = i + 2;
                
            end
            
        else
            
            if i == numel(content)
                ret_i = -1;
                return
            else
                ret_i = i + 1;
                return
            end
        
        end
          
    end
    
end