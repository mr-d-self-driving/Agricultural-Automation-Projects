function [gamma,error] = purePursuitController(q,L,Ld,path)
%This function will give the turning angle in order to follow a path. The
%primary tuning paramter in this function is the look-ahead distance Ld
%INPUTS:(state of the tractor (3x1 double), distance between wheels (double),
%lookahead distance (double), array representing path to follow (2xN array)
%OUTPUTS: turning angle (double), cross track error (double)

%% Find point closest to tractor in world frame

dist = ((path(:,1)-q(1)).^2 + (path(:,2)-q(2)).^2).^(1/2); %calculates distance to each point in path
[Mc,Ic] = min(dist); %find index and value of closest point
%closePoint = path(Ic,:); %closest point in path 

error = dist(Ic); %gives cross track error

%% Find the goal point
%find the index of the point that is closest to goal distance and ahead of 
%the close point

[Mg,Ig] = min( abs(Ld - dist(Ic:length(dist))) ); %finds index
goalPoint = path((Ig+Ic-1),:);%finds goal point in path

%% Transform the goal point to vehicle co-ordinates

w_T_r = transl2(q(1),q(2))*trot2(q(3)); %world to robot transformation
gP_w = [goalPoint 1]'; %putting goal point into homogeneous coord.
gP_R = (w_T_r)^-1 * gP_w; %gives goal point in robot frame

%% Calculates the Steering angle

k = 2 * gP_R(2)/Ld^2; %calculates kappa
gamma = atan(k*L); %calculates turning angle


