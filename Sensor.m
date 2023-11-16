function [BB,pqr]=Sensor(BB, pqr)

for idx=1:3
    sensor_params

    %pollute the data
    BB(idx) = BB(idx) + MagFieldBias + MagFieldNoise;
    pqr(idx) = pqr(idx) + AngFieldBias + AngFieldNoise;
end
