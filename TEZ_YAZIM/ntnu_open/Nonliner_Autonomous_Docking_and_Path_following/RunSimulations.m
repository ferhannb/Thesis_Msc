function [mean_e,cross_track,mean_chi_e,course_error,energy_consumption, mean_energy,finish_time, median_e,median_chi_e] = RunSimulations(N,num_sim,Controller_choise,num_obstacles)

%Select seed for RNG
rng('default')
% rng(3)

mean_e = zeros(num_sim,1);
mean_chi_e = zeros(num_sim,1);

energy_consumption = zeros(num_sim,1);
finish_time = zeros(num_sim,1);

median_e = zeros(num_sim,1);
median_chi_e = zeros(num_sim,1);

for i=1:num_sim
    [buffer,finish_time(i)]= MonteCarlo(N,num_obstacles,Controller_choise);
    buffer=buffer';
    
    mean_e(i) = mean(abs(buffer(21,:)));
    mean_chi_e(i) = mean(abs(buffer(20,:)));
    energy_consumption(i) = buffer(22,end)+buffer(23,end);
    
    median_e(i) = median(abs(buffer(21,:)));
    median_chi_e(i)=median(abs(buffer(20,:)));
end

cross_track=mean(mean_e);
course_error=mean(mean_chi_e);
mean_energy=mean(energy_consumption);

end