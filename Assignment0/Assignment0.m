%% Initializazion
clear
clc
close all

set(groot, 'defaultFigureWindowStyle', 'docked');


%% Data extraction
data1=load("data1.mat");
data2=load("data2.mat");
data3=load("data3.mat");
data= {data1.data1, data2.data2, data3.data3};


%% Data plotting
% Tiledlayout allows to condense the three plots in only one figure
% The if-else structure is used due to data2 containing a two-row array 

figure(1);
tiledlayout(3,1);
for i = 1:3
    nexttile

    if i == 2
        plot(data{i}(1,:), data{i}(2,:)); 
        axis equal

    else
        plot(data{i});

    end

    grid on
end


%% Frequency estimation 
% Done by using the signal's Fourier transform 
% fftshift is used for more ordered plots

f_v = [2000, 166, 250];

figure(2);
tiledlayout(3,1);
for i = 1:3
    nexttile
    y = fftshift(data{i});
    fs = f_v(i);
    f = (0:length(y)-1)*fs/length(y);

    plot(f,abs(y));

    grid on
end
 

%% Data evaluation
% data1 is EMG data due to the signal's symmetry shown in the respective plot 

% data2 represents motion data due to the plot in figure 1 
% having movements in the eight directions from a specified center

% data3 is EEG data due to its pseudo-oscillatory movement, which shows
% various stages in brain activity