%Mass of satellite
m = 9.061; %%kilograms

%Inertia of satellite in kg-m^2 (I estimated)
I = [0.0797 -0.00005 0.00125 ;-0.00005 0.1291 -0.00103;0.00125 -0.00103 0.1310];
invI = inv(I);
