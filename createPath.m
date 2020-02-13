function [path] = createPath(q_start, type, length, alpha, spacing)
%This function creates either a circular or straight path at the given start
%point, with a length (or diameter) equal to the given length value and a
%point spacing given by spacing.
%INPUTS: starting pose, type of path segment, linear length or diameter,
%arc segment (rad), point spacing
%OUTPUTS: path 2xN array of points

%Creates paths of type linear
if (strcmp(type,'line')) 
    
    n = round(length/spacing); %number of points
    path = zeros(n,2); %initializes path array
    
    %builds path by adding points incremenally to the starting vector
    for i = 0:1:n
        path(i+1,:) = [(q_start(1)+ i*spacing*cos(q_start(3))) (q_start(2)+ i*spacing*sin(q_start(3)))];
    end

%creates paths of type circular   
elseif (strcmp(type,'circle'))
    
    r = length/2; %radius
    n = round((r*alpha)/spacing); %number of points
    path = zeros(n,2);%initializes path array
    beta = q_start(3)+(pi/2);%starting angle + 90deg
    gamma = q_start(3)-(pi/2);%starting angle - 90deg
    
    center = [(q_start(1)+r*cos(gamma)) (q_start(2)+r*sin(gamma))]; %center or circle
    
    %creates path incrementally 
    for i = 0:1:n
        path(i+1,:) = [(center(1) + r*cos(beta-i*(alpha/n))) (center(2) + r*sin(beta-i*(alpha/n)))];
    end
elseif (strcmp(type,'circleback'))
    
    r = length/2; %radius
    n = round((r*alpha)/spacing); %number of points
    path = zeros(n,2);%initializes path array
    beta = q_start(3)+(pi/2);%starting angle + 90deg
    gamma = q_start(3)-(pi/2);%starting angle - 90deg
    
    center = [(q_start(1)+r*cos(beta)) (q_start(2)+r*sin(beta))]; %center or circle
    
    %creates path incrementally 
    for i = 0:1:n
        path(i+1,:) = [(center(1) + r*cos(gamma+i*(alpha/n))) (center(2) + r*sin(gamma+i*(alpha/n)))];
    end    
else
    

end

