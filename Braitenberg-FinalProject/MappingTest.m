% Map Environment
bt = Bluetooth('BRAITENBERG_BT', 1);
fopen(bt);

%%
startString = '~';
stopString = '!';
while (true)
    message = fread(bt);
    if (message(1) == double(char(startString)))
        xLoc = message(2);
        yLoc = message(3);
        walls = message(4);
        if (message(5) == double(char(stopString)))
            break;
        end
    end    
end