clearvars;
clear global variables;
clf

N  = 12500;

num_sim = 10000;
num_obstacles=3;

%Mean error values
track_e = zeros(3,num_sim);
course_e = zeros(3,num_sim);
energy = zeros(3,num_sim);
finish_time = zeros(3,num_sim);

%Median error values
median_track_e = zeros(3,num_sim);
median_course_e = zeros(3,num_sim);

%Grand mean error values
m_track_e = zeros(3,1);
m_course_e = zeros(3,1);
m_energy = zeros(3,1);

%% Main loop
for i=1:3
Controller_choise=i-1;

Current_iteration=i

[track_e(i,:),m_track_e(i),course_e(i,:),m_course_e(i),...
    energy(i,:),m_energy(i),finish_time(i,:),...
    median_track_e(i,:), median_course_e(i,:)] = RunSimulations(N,num_sim,Controller_choise,num_obstacles);

end

%% Plots
t = 1: num_sim;
controller=["STC","PID-SMC","PID"];

figure(1)
plot(t,track_e)
title("Cross track error")
xlabel("Iteration")
ylabel("cross-track error [m]")
legend(controller,'Location','northwest')
axis tight
grid

figure(2)
plot(t,(180/pi)*course_e)
title("Course error")
xlabel("Iteration")
ylabel("course error [deg]")
legend(controller,'Location','northwest')
axis tight
grid

figure(3)
plot(t,energy)
title("Energy consumption")
xlabel("Iteration")
legend(controller,'Location','northwest')
axis tight
grid

figure(4)
plot(t,finish_time)
title("Time used")
xlabel("Iteration")
ylabel("time [s]")
legend(controller,'Location','northwest')
axis tight
grid    

figure(5)
plot(t,sort(track_e,2))
title("Cross track error sorted")
xlabel("Iteration")
ylabel("Distance [m]")
legend(controller,'Location','northwest')
axis tight
grid

figure(6)
plot(t,sort((180/pi)*course_e,2))
title("Course error sorted")
xlabel("Iteration")
ylabel("angle [deg]")
legend(controller,'Location','northwest')
axis tight
grid

figure(7)
plot(t,sort(energy,2))
title("Energy consumption sorted")
xlabel("Iteration")
legend(controller,'Location','northwest')
axis tight
grid

figure(8)
plot(t,sort(finish_time,2))
title("Time used sorted")
xlabel("Iteration")
ylabel("time [s]")
legend(controller,'Location','northwest')
axis tight
grid    