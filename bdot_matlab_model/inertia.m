%Mass of satellite
m = 7.6; %%kilograms

%Inertia of satellite in kg-m^2 (I estimated)
I = [0.04 0 0 ;0 0.09 0;0 0 0.06];
invI = inv(I);
