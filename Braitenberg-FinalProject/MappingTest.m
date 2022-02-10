% Map Environment
bt = Bluetooth('BRAITENBERG_BT', 1);
fopen(bt);

%%
startString = '~';
stopString = 'stop!';
while (true)
    message = fread(bt);

    if (startsWith(message, startString))
        message = erase(message, startString);
        
        if (strcmp(message, stopString))
            break;
        else
            % Do things
            % x location, y location, wall type
        end
    end    
end