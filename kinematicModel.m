function [q] = kinematicModel(q, u, Umin, Umax, Qmin, Qmax, l, tau_g, tau_v, delta1, delta2, s)
% This function performs Euler integration to find the final state of the
% tractor based on input parameters. The function returns the 6
% element stae vector of the system
%INPUTS: (State vector (5x1 double), desired velocity and turning 
% angle (2x1 double), input constraints(2x1 double), state contraints(5x1 double),
% wheelbase (double), turning timelag (float), velocity time-lag (float),
% skid-angle (double) and slip ratio (double)
%OUTPUTS: New state vector (5x1 double)

%declaring integration and control timesteps as global
global dT DT
%'new' state vector for iteration
q_new = [0,0,0,0,0];

%Enforcing input constraints 
for k = 1:1:2
    if u(k) > Umax(k)
        u(k) = Umax(k);
    elseif u(k) < Umin(k)
        u(k) = Umin(k);
    end
end

%begin loop over dT. This loop integrates the state at every integration
%timestep.
for t = 0:dT:DT-dT
    
    %enforcing state constraints
    for k = 1:1:5
        if q(k) > Qmax(k)
            q(k) = Qmax(k);
        elseif q(k) < Qmin(k)
            q(k) = Qmin(k);
        end
    end
    
    %Calculates the linear velocity and tangential (skid) velocity
    vl = q(4); %linear velocity
    vy = vl*tan(delta2); %slip angular velocity

    %Calculates the final pose of the tractor in the world frame by integrating
    %the instantaneous velocites.
    q_new(1) = q(1) + dT*(vl * cos(q(3)) - vy * sin(q(3)) ); %final x position
    q_new(2) = q(2) + dT*(vl * sin(q(3)) + vy * cos(q(3)) ); %final y position
    q_new(3) = q(3) + dT*( (vl * tan(q(5) + delta1) - vy) / l); %final direction
    
    %Handles the two cases for tau_v (either tau_v is zero or it has some
    %positive value 
    if tau_v == 0
        q_new(4) = (1-s)*u(2); %setting desired velocity equal to velocity
    else
        q_new(4) = q(4)+(-q(4)+((1-s)*u(2)))*(dT/tau_v); %final velocity with time lag
    end
    
    %Handles the two cases for tau_g
    if tau_g == 0
        q_new(5) = u(1); %sets steering angle equal to desired steering angle
    else
        q_new(5) = q(5)+(-q(5)+u(1))*(dT/tau_g); %final steering angle with time lag
    end
    
    q = q_new;
end

