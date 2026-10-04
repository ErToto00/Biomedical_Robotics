%% Initializazion
clear
clc

%% Data extraction
data1=load("data1.mat");
data2=load("data2.mat");
data3=load("data3.mat");
data= {data1.data1, data2.data2, data3.data3};

%% Data plotting
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