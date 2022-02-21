function callbackFcn(bt)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

startString = '~';

message = read(bt,4);
disp(message)
if (message(1) == double(char(startString)))
    xLoc = message(2);
    yLoc = message(3);
    assignin('caller','xLoc',xLoc)
    assignin('caller','yLoc',yLoc)
end 
GUI.readLoc(xLoc, yLoc);
end

