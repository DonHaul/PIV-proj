function [ xyz] = get_xyz( coord,z,K )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Kx = K(1,1);
Cx = K(1,3);
Ky = K(2,2);
Cy = K(2,3);

xyz(1) = (z/Kx) * (coord(1)-Cx);
xyz(2) = (z/Ky) * (coord(2)-Cy);
xyz(3) = z;

end

