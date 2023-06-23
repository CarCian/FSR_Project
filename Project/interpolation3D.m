function [trajectory] = interpolation3D(q_in,q_fin,delta)
%INTERPOLATION Summary of this function goes here
%   Detailed explanation goes here
aplha=atan2(q_in(2)-q_fin(2),q_in(1)-q_fin(1));
dist_x_y=((q_in(1)-q_fin(1))^2 +(q_in(2)-q_fin(2))^2)^0.5;
dist_x_y_z=((q_in(1)-q_fin(1))^2 +(q_in(2)-q_fin(2))^2+(q_in(3)-q_fin(3))^2)^0.5;
theta=atan2(q_in(3)-q_fin(3),dist_x_y);
tree=q_in;
    for i=1:floor(dist_x_y_z/delta)
    q=[ floor(q_in(1)+delta*i*cos(aplha)) ,floor(q_in(2)+delta*i*sin(aplha)) ,floor(q_in(3)+delta*i*sin(theta)) ];
    tree=cat(1,tree,q);
    end
    tree=cat(1,tree,q_fin);
    trajectory=tree;
end

