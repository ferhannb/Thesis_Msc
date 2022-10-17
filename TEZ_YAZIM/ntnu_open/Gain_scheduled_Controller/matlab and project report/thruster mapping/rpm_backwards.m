

%%%%% Extracting RPM values from bollard pull test [BACKWARDS] 16.05.2019 %%%%%

clear all
close all
clc

load bollard_data.csv;

%% 10% thrust
stb_10 = bollard_data(17238:17498,12);
port_10 = bollard_data(17238:17498,13);
stb_10_mean = mean(stb_10);
port_10_mean = mean(port_10);

meas_10 = (stb_10_mean+port_10_mean)/2;

%% 20% thrust
stb_20 = bollard_data(17520:17730,12);
port_20 = bollard_data(17520:17730,13);
stb_20_mean = mean(stb_20);
port_20_mean = mean(port_20);

meas_20 = (stb_20_mean+port_20_mean)/2;



%% 30% thrust
stb_30 = bollard_data(18050:18199,12);
port_30 = bollard_data(18050:18199,13);
stb_30_mean = mean(stb_30);
port_30_mean = mean(port_30);

meas_30 = (stb_30_mean+port_30_mean)/2;


%% 40% thrust
stb_40 = bollard_data(18302:18349,12);
port_40 = bollard_data(18302:18349,13);
stb_40_mean = mean(stb_40);
port_40_mean = mean(port_40);

meas_40 = (stb_40_mean+port_40_mean)/2;


%% 50% thrust
stb_50 = bollard_data(25245:25302,12);
port_50 = bollard_data(25245:25302,13);
stb_50_mean = mean(stb_50);
port_50_mean = mean(port_50);

meas_50 = (stb_50_mean+port_50_mean)/2;

%% 60% thrust
stb_60 = bollard_data(25010:25070,12);
port_60 = bollard_data(25010:25070,13);
stb_60_mean = mean(stb_60);
port_60_mean = mean(port_60);

meas_60 = (stb_60_mean+port_60_mean)/2;


%% 70% thrust
stb_70 = bollard_data(24587:24691,12);
port_70 = bollard_data(24587:24691,13);
stb_70_mean = mean(stb_70);
port_70_mean = mean(port_70);

meas_70 = (stb_70_mean+port_70_mean)/2;


%% 80% thrust
stb_80 = bollard_data(20970:21064,12);
port_80 = bollard_data(20970:21064,13);
stb_80_mean = mean(stb_80);
port_80_mean = mean(port_80);

meas_80 = (stb_80_mean+port_80_mean)/2;



%% 90% thrust
stb_90 = bollard_data(23507:23601,12);
port_90 = bollard_data(23507:23601,13);
stb_90_mean = mean(stb_90);
port_90_mean = mean(port_90);

meas_90 = (stb_90_mean+port_90_mean)/2;



%% 100% thrust
stb_100 = bollard_data(24145:24250,12);
port_100 = bollard_data(24145:24250,13);
stb_100_mean = mean(stb_100);
port_100_mean = mean(port_100);

meas_100 = (stb_100_mean+port_100_mean)/2;

rpm_b = [0 meas_10 meas_20 meas_30 meas_40 meas_50 meas_60 meas_70 meas_80 meas_90 meas_100];
