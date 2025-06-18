function R321 = RotationMatrix321(attitude)
% Braden Nelson
% Created: 3/7/25
%
% Inputs: attitude = 3x1 euler angles
% 
% Output: 3x3 rotation matrix in the from
%         R321 = [a1 a2 a3; b1 b2 b3; c1 c2 c3];
%
% Methodology: using the euler angles given in order to algebraically
%              compute a 3-2-1 rotation matrix parameterized by those
%              angles.

phi = attitude(1);
theta = attitude(2);
psi = attitude(3);

a1 = cos(theta)*cos(psi);
a2 = cos(theta)*sin(psi);
a3 = -sin(theta);

b1 = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
b2 = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
b3 = sin(phi)*cos(theta);

c1 = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
c2 = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
c3 = cos(phi)*cos(theta);

R321 = [a1 a2 a3; b1 b2 b3; c1 c2 c3];

end