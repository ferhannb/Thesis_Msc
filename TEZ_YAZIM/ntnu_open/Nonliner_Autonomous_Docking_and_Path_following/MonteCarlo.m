function [simdata,finish_time ] = MonteCarlo(N,num_obstacles,Controller_choise)
%% Init
h  = 0.02;        % sampling time [s]
global Xudot Nrdot Iz Xu m mp alpha v_stw ...
    tau_1_int tau_6_int wpt obstacle_list ;

%  x = [ u v w p q r x y z phi theta psi u_int psi_int]'
x = zeros(14,1);
x_dot=zeros(14,1);

% Surge speed
u_d=1;
u_r=1;

%docking speed
u_r_min=0.2;

%Course
chi_d = 0;
chi_d_ddot = h*chi_d;

chi_r=0;

%Course of otter
chi_otter=0;
chi_e = chi_d-x(12);

x(1)=u_d; %Set initial velocity to the desired velocity

cross_track_error = 0;
n=[0 0];

n_out=[0 0];
Thrust_calculated = n_out;

% Boat mass
m = 55;

% Load condition
mp = 0;                % payload mass (kg), max value 45 kg
rp = [0 0 -0.35]';      % location of payload (m)
rg = [0.2 0 -0.2]';     % CG for hull only (m)

T_yaw = 0.5;                            % time constant in yaw (s)
k_pos = 0.02216/2;                      % Positive Bollard, one propeller
k_neg = 0.01289/2;                      % Negative Bollard, one propeller

% Init (finding Xudot, Nrdot)
init(rg,rp,mp,T_yaw,k_pos,k_neg)

%Gain for PID controller
K = T_yaw/Iz;

% Current
V_c_min=0.;
V_c_max = 0.5;
V_c = randi(100*(V_c_max))/100;

beta_c_min = -180 * pi/180;   % current direction (rad)
beta_c_max = 180 * pi/180;   % current direction (rad)
beta_c = randi(round(2*beta_c_max))+beta_c_min;

%Time constants for time varying currents
T_v=1;
T_beta=1;

ZOH_current=100;

%% WAYPOINTS
wpt.pos.y=[30,30,20,20, 10, 10];
wpt.pos.x=[0,20,70,110, 130, 180];

obstacle_list=[0;0];

wp_LOSindex=1;

R_min=2;
R_max=3;

num_obstacles = randi(num_obstacles);

[RO, obstacle] = obstacle_generator(R_min,R_max,num_obstacles);

%Set position for the obstacle center
y_obstacle= obstacle(1,:);
x_obstacle=obstacle(2,:);

Rm=RO*4;

weight=2;

%Set initial position at the first waypoint
x(7)=wpt.pos.x(wp_LOSindex);
x(8)=wpt.pos.y(wp_LOSindex);

%% LOS
R=4;                                         %Radius for path-finding
Delta=4;                                     %Delta for lookahead path-finding

add_noise=1;         %0=no noise, 1=add noise
%% Reference model parameters

%Reference model course
r_d=0;
r_dmax=1;
a_d=0;
a_dmax= 0.5;
omega_n=1.3;
damp=0.9;

%Reference model surge
u_r_d=0;
u_r_dmax=1;
u_a_d=0;
u_a_dmax= 0.5;
u_omega_n=1.8;
u_damp=1;

%% Time varying current
alpha_c1=0.005;
alpha_c2=0.01;

%% Course STC constants
omega=17;
gamma=1;
epsilon=0.08;
alpha_m=0.005;

lambda_stc_1= 1.6;
lambda_stc_2= 8;
e_chi_stc=0.005;

%% Course PID SMC contstants
lambda_smc=1.2493;
e_smc=0.1;
k_s=72.241;

%% Surge PI constants
Kp_u = 239.3;
Ki_u =47.594;

%% Course PID constants
Kp_chi = 272.5044;
Kd_chi = 190.0829;
Ki_chi = 52.9263;

%% MAIN LOOP
simdata = zeros(N,31);                   % table for simulation data

for i=1:N+1
    t = (i-1) * h;                          % time (s)
    
    %Store simulation data in a table
    simdata(i,:) = [t x' n u_d chi_r chi_e cross_track_error tau_1_int tau_6_int...
        chi_otter chi_d n_out Thrust_calculated V_c beta_c];
    
    %% LOS Guidance law
    
    %Calculation of desired course using LOS
    
    [~,obstacle_index] = min((sqrt(((x_obstacle-x(7)).^2 + (y_obstacle-x(8)).^2))));
    
    sigma = sqrt((x(8) - y_obstacle(obstacle_index))^2+(x(7) - x_obstacle(obstacle_index))^2);  
    sigma_dot = ((2*(x(7)-x_obstacle(obstacle_index))*x_dot(7)) +(2*(x(8)-y_obstacle(obstacle_index))*x_dot(8)))/(2*sigma);
    
    T_c = in_T_C(sigma,sigma_dot,min(Rm(obstacle_index),max(sigma,RO(obstacle_index))),inf);
    
    %Obstacle avoidance
    if T_c~=true
        if ( ~any(all(obstacle_list ==[x_obstacle(obstacle_index);y_obstacle(obstacle_index)])))
            path_replanning(RO(obstacle_index),x_obstacle(obstacle_index), y_obstacle(obstacle_index),weight,wp_LOSindex,x(7:8));
        end
    end
    
    [chi_r,cross_track_error,wp_LOSindex]= lookahead_LOS(wp_LOSindex,x(7:8),wpt.pos.x,wpt.pos.y,R,Delta);
    
    %% Reference models
    [u_d_dot, u_r_d_dot, u_a_d_dot] = referencemodel(u_r,u_r_d,u_r_dmax,u_a_d,u_a_dmax,u_omega_n,u_damp,u_d);
    
    [chi_d_dot, r_d_dot, a_d_dot] = referencemodel(chi_r,r_d,r_dmax,a_d,a_dmax,omega_n,damp,chi_d);
    
    %% Calculate error
    chi_e=chi_otter - chi_d;
    chi_e_dot= x(6) - chi_d_dot;
    
    %% Control-laws
    %PI surge controller
    tau_1 = (m+mp - Xudot)*u_d_dot + Xu*u_d - Kp_u*(x(1)-u_d) - Ki_u*x(13);
    
    %Super-Twist controller
    if Controller_choise==0
        [tau_6,v_stw_dot,alpha_dot]=STC(lambda_stc_2,alpha_m,omega,gamma,epsilon,lambda_stc_1,chi_e,chi_e_dot,e_chi_stc);
    end
    
    %PID Sliding Mode Controller
    if Controller_choise==1
        tau_6 =PID_SMC(k_s,lambda_smc,e_smc,chi_e,chi_e_dot,chi_d_dot,chi_d_ddot,x(14));
    end
    
    %PID course controller
    if Controller_choise == 2
        tau_6 = (Iz - Nrdot)*r_d_dot + 1/K*chi_d_dot - Kp_chi*(chi_e)  - Kd_chi*(chi_e_dot) - Ki_chi * x(14);
    end
    
    %% Control allocation
    [n, Thrust_calculated]=calc_thrust(tau_1,tau_6,k_pos,k_neg);
    
    %% Calculate states
    [x_dot(1:12),n_out]= otter(x(1:12),n,mp, rp, V_c,beta_c);
    
    %Time varying current
    if mod(i,ZOH_current)==0
        V_c_dot= -alpha_c1 * V_c + (rand(1)-0.5)/2;
        beta_c_dot= -alpha_c2 * beta_c + (rand(1)-0.5)/0.5;
    else
        V_c_dot=0;
        beta_c_dot=0;
    end
    
    %% Euler integration (k+1)
    x(1:12)=x(1:12)+h*x_dot(1:12);
    
    % Calculate u_e_int x(13)
    x(13) = x(13)+ h*(x(1)-u_d);
    
    % Calculate saturated chi_e_int x(14)
    x(14) = sat(x(14)+ h*(x(12)-chi_d),0.01);
    
    
    
    % Calculate integral from STC
    if Controller_choise==0
        v_stw=v_stw + h* v_stw_dot;
        alpha = alpha + h*alpha_dot;
    end
    
    %Calculate energy consumption
    tau_1_int=tau_1_int + h*(tau_1^2);
    tau_6_int=tau_6_int + h*(tau_6^2);
    
    % Calculate integral from reference models
    chi_d = chi_d + h* chi_d_dot;
    r_d = r_d + h*r_d_dot;
    a_d = a_d + h* a_d_dot;
    
    u_d=u_d + h* u_d_dot;
    u_r_d=u_r_d + h * u_r_d_dot;
    u_a_d=u_a_d + h * u_a_d_dot;
    
    %Calculate stochastic time varying current values
    V_c = sat2(V_c + (h/T_v) * V_c_dot,V_c_min,V_c_max);
    
    beta_c =sat2(beta_c + (h/T_beta)*beta_c_dot,beta_c_min,beta_c_max);
    
    %% Calculate course and add noise
    chi_otter=atan2(x_dot(8),x_dot(7));
    
    if add_noise==1
        [x(7:8),x(1),x(10:11),chi_otter]= noiseGenerator(i,x(7:8),x(1),x(10:11),chi_otter);
    end
    
    %% Docking phase
    
    total_distance = sqrt((x(7)-wpt.pos.x(end))^2 + (x(8)-wpt.pos.y(end))^2);
    
    %Reduction of desired surge speed and current
    if total_distance <= 15
        xd=0.0001;
        u_r=sat2(u_r - xd*total_distance,u_r_min,inf);
        V_c = sat2(V_c - (xd*0.3)*total_distance,V_c_min,V_c_max);
        alpha_c1=0.2;
    end
    
end

eta  = simdata(:,8:13);
[~,time_index] = min(sqrt((wpt.pos.y(end)-eta(:,2)).^2 + (wpt.pos.x(end)-eta(:,1)).^2));
finish_time = (time_index-1)*h;

end