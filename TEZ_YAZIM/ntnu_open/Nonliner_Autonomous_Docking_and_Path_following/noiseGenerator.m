function [pos,vel,pitchroll,heading]= noiseGenerator(i,pos,vel,pitchroll,heading)

global pos_noise vel_noise pitchroll_noise heading_noise;

h=0.02;
T=1;

%Sample frequency of 1Hz
if mod(i,50)==0
    
    %Position
    pos_noise   = wgn(1,2,0.00003,'linear')';
    
    %Velocity
    vel_noise     = wgn(1,1,0.00006,'linear')';
    
    %Angle
    pitchroll_noise = deg2rad(wgn(1,2,0.0007,'linear'))' ;
    heading_noise    = deg2rad(wgn(1,1,0.001,'linear')) ;
end

%Position with raw noise
pos_in   = pos + pos_noise;

%Velocity with raw noise
vel_in     = vel + vel_noise;

%Angle with raw noise
pitchroll_in = pitchroll + pitchroll_noise;
heading_in    = heading   + heading_noise;

% Low-pass filtering
pos = (h/T).*(pos_in-pos)+pos;
vel = (h/T).*(vel_in-vel)+vel;
pitchroll = (h/T).*(pitchroll_in-pitchroll)+pitchroll;
heading = (h/T).*(heading_in-heading)+heading;

end