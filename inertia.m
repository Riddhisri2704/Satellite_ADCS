%inertia and mass
m=2.6;                  %mass of satellite in kg

I= [0.9 0 0 ; 0 0.9 0; 0 0 0.3];

%Invert the matrix
invI = inv(I);