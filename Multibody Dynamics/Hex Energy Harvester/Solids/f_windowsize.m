% Sets window size for trajectory plots & animation

function [x_low, x_high, y_low, y_high] = f_windowsize(r_final)

 % Final Position of head
    x_final = r_final(1);
    y_final = r_final(2);
    
    % Sets the size of the window
    buffer = 25;  % Buffer
    if y_final >= 0 && x_final >= 0
        if y_final >= x_final
            y_high = y_final+buffer;
            y_low  = -buffer;
            x_high = x_final + (y_high-y_low)/2;
            x_low  = x_final - (y_high-y_low)/2;
        else
            x_high = x_final+buffer;
            x_low  = -buffer;
            y_high = y_final + (x_high-x_low)/2;
            y_low  = y_final - (x_high-x_low)/2;
        end
    elseif y_final >= 0 && x_final < 0
        if y_final >= -x_final
            y_high = y_final+buffer;
            y_low  = -buffer;
            x_high = x_final + (y_high-y_low)/2;
            x_low  = x_final - (y_high-y_low)/2;
        else
            x_high = buffer;
            x_low  = x_final-buffer;
            y_high = y_final + (x_high-x_low)/2;
            y_low  = y_final - (x_high-x_low)/2;
        end
    elseif y_final < 0 && x_final >= 0
        if -y_final >= x_final
            y_high = buffer;
            y_low  = y_final-buffer;
            x_high = x_final + (y_high-y_low)/2;
            x_low  = x_final - (y_high-y_low)/2;
        else
            x_high = x_final+buffer;
            x_low  = -buffer;
            y_high = y_final + (x_high-x_low)/2;
            y_low  = y_final - (x_high-x_low)/2;
        end    
    elseif y_final < 0 && x_final < 0
        if -y_final >= -x_final
            y_high = buffer;
            y_low  = y_final-buffer;
            x_high = x_final + (y_high-y_low)/2;
            x_low  = x_final - (y_high-y_low)/2;
        else
            x_high = buffer;
            x_low  = x_final-buffer;
            y_high = y_final + (x_high-x_low)/2;
            y_low  = y_final - (x_high-x_low)/2;
        end
    end

end