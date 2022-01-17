LEFT_PHOTO = 'A0';
RIGHT_PHOTO = 'A1';
configurePin(a, LEFT_PHOTO, 'AnalogInput');
configurePin(a, RIGHT_PHOTO, 'AnalogInput');

for i = 1:100
    left(i) = readVoltage(a,LEFT_PHOTO);
    right(i) = readVoltage(a,RIGHT_PHOTO);
end