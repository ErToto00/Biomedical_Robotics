%% Initializazion
clear all
clc


%% Data extraction
data1=load("data1.mat");
data2=load("data2.mat");
data3=load("data3.mat");
data= {data1.data1, data2.data2, data3.data3};


%% Data plotting
for i = 1:3
    plot(data{i});
    grid on
end