%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description: Data loading, light signal analysis + derivative
%   Last modified: 2023-09-06 (modified)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;
clear all;
close all;

%% Load CSV file
filename = '/home/ifragnie/Desktop/MyFiles/BA3_SIS/material/controllers/controller/data/light.csv';
data = readtable(filename);

% Clean column names
data.Properties.VariableNames = strtrim(data.Properties.VariableNames);

%% Extract signals
time  = data.time;
light = data.intensity;
filtered_light = data.filtered_light;


%% Remove invalid values (if needed)
light_filtered = light(light ~= 1 & ~isnan(light));

mean_light   = mean(light_filtered);
median_light = median(light_filtered);

%% Smooth signal (reduce noise before derivative)
light_smooth = smoothdata(light, 'gaussian', 10);

%% Compute derivative (after smoothing)
light_derivative = gradient(light_smooth, time);

%% Plot
figure;

subplot(2,2,1);

% --- Left axis: light signal ---
yyaxis left
plot(time, light, 'b', 'LineWidth', 1.2);
hold on;
grid on;
ylabel('Light');

% Mean and median lines
yline(mean_light, 'r--', ['Mean = ' num2str(mean_light)], ...
    'LineWidth', 1.2);

yline(median_light, 'g-.', ['Median = ' num2str(median_light)], ...
    'LineWidth', 1.2);

% --- Right axis: derivative ---
yyaxis right
plot(time, filtered_light, 'm', 'LineWidth', 1.2);
ylabel('dLight/dt');

title('Light intensity and smoothed derivative');
xlabel('Time [s]');

legend('Light', 'Mean', 'Median', 'Derivative');