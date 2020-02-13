function [] = plotTractor(pTractor,T,color)
% This function takes the homogeneous array defining the points of a tractor
% and a trasformation matrix and animates a polygon moving to the pose 
% defined by the transformation T.


%Defines points of polygon
P = pTractor(1:2,:);

H = plot_poly(P, 'animate', color);
plot_poly(H, T);

end

