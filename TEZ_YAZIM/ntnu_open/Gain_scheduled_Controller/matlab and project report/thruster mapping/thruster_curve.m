clear all
% Convert rpm to rad/s
rpm2rads = 2*pi/60;

%% Measured pull
% Forward pull
Fp = [0 0.4 1.3 3.0 5.4 8.8 12.4 17.3 22.7 24.4 24.0]'*9.81;                                    % [N]
thruster_f = [0 121.4 244.6 358.7 490.5 623.2 726.1 844.6 968.0 972.0 970.7]'*rpm2rads;         % Measurements are in rpm, converts it to rad/s

% Backward pull
Bp = -[0 0 0.7 1.9 3.5 5.4 7.4 10.8 13.4 13.5 13.6]'*9.81;                                      % [N]
thruster_b = -[0 122.0 244.4 359.5 493.4 615.4 728.1 857.1 964.4 970.9 968.1]'*rpm2rads;        % Measurements are in rpm, converts it to rad/s

%% Setup for using fit() 
s = fitoptions('Method','NonlinearLeastSquares',...
               'Lower',min(Bp),...
               'Upper',max(Fp),...
               'Startpoint',0);
           
f = fittype('a*x^n','problem','n','options',s);

[nlsq_B,gof] = fit(thruster_b,Bp,f,'problem',2);

[nlsq_F,gof2] = fit(thruster_f,Fp,f,'problem',2);

figure(4)


y1 = 0.01289*abs(thruster_b).*thruster_b;                                                       % nlsq_B finds the thrust parameter in rad/s (0.01289)

hold on
plot(thruster_b,y1)
scatter(thruster_b, Bp)



y2 = 0.02216*abs(thruster_f).*thruster_f;                                                       % nlsq_F finds the thrust parameter in rad/s (0.02141) 
plot(thruster_f,y2)

scatter(thruster_f, Fp)
scatter([972.0 970.7]*rpm2rads, [24.4 24.0]*9.81)

xlabel("\omega [rad/s]")
ylabel("Thrust [N]")
title("Thrust curve for Otter USV")

hold off


