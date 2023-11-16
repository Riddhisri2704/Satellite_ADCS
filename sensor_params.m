%Update rate
nextSensorUpdate=1;

%bias and noise

MagscaleBias=(4e-7);    %Tesla
MagFieldBias=MagscaleBias*(2*rand()-1);  % -1 to 1

MagscaleNoise=(1e-7);   %Tesla
MagFieldNoise=MagscaleNoise*(2*rand()-1);  % -1 to 1

AngscaleBias=0.001;    %rad/s
AngFieldBias=AngscaleBias*(2*rand()-1);  % -1 to 1

AngscaleNoise=0.0005;  %rad/s
AngFieldNoise=AngscaleNoise*(2*rand()-1);  % -1 to 1