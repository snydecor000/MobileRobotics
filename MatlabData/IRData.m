RIGHT_IR = 'A12';
BACK_IR = 'A15';
LEFT_IR = 'A14';
FRONT_IR = 'A13';
configurePin(a, RIGHT_IR, 'AnalogInput');
configurePin(a, BACK_IR, 'AnalogInput');
configurePin(a, LEFT_IR, 'AnalogInput');
configurePin(a, FRONT_IR, 'AnalogInput');

for i = 1:100
%    front(i) = readVoltage(a,FRONT_IR);
%     back(i) = readVoltage(a,BACK_IR);
    left(i) = readVoltage(a,LEFT_IR);
%     right(i) = readVoltage(a,RIGHT_IR);
end

j = 12;
leftIR(j,:) = left;

%% Plots
data = backIR;

% figure()
% hold on;
% for i = 1:12
%     splitData = reshape(data(i,:),5,20);
%     plot(median(splitData,2))
% end
% hold off;

mins = min(data,[],2);
maxs = max(data,[],2);
medians = median(data,2);
means = mean(data,2);

figure()
hold on;
plot(mins);
plot(maxs);
plot(means);
plot(medians);
legend('MIN','MAX','MEAN','MEDIAN');
hold off;

% figure();
% for i = 1:12
%     subplot(3,4,i);
%     histogram(data(i,:),20);
%     title([num2str(i) 'in away'])
% end

