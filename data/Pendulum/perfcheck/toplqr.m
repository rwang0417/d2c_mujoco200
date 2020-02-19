%% system parameters
NUM_IN = 1;
NUM_SYS = 2;
STEP_MAX = 1;
%% Load from .txt file
fid = fopen('lnr_top.txt','r');
Ua  = fscanf(fid, '%f %f %f');
fclose(fid);
La = reshape(Ua, NUM_SYS + NUM_IN, NUM_SYS * STEP_MAX)';
%% dlqr
Q = [100 0;
     0 2];
R = 0.1;
[Kd,ss,e] = dlqr(La(:,1:NUM_SYS),La(:,NUM_SYS+1:NUM_SYS+NUM_IN),Q,R);
Kd
