bt = Bluetooth('BRAITENBERG_BT',1)
fopen(bt);
%% Say 'Hello World' to the robot
fwrite(bt,[72 101 108 108 111 32 87 111 114 108 100])

