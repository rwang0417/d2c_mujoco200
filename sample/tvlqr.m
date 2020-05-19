clear all;
% mex mexstep.c mujoco150.lib
%% Param
NUM_SYS             = 2;    %System state number
NUM_IN              = 1;    %Number of inputs
STEP_MAX            = 10; %Total timestep
%% Var
sig_q = 10^3;
sig_f = 10^4;
Ri = 10^0 * eye(NUM_IN);
Qi = sig_q * eye(NUM_SYS);
OS = zeros(NUM_SYS, NUM_SYS, STEP_MAX+1);
TK = zeros(NUM_IN, NUM_SYS, STEP_MAX);
OS(:, :, STEP_MAX+1) = sig_f * eye(NUM_SYS);
%% Load from .txt file
fid = fopen('lnr.txt','r');
Ua  = fscanf(fid, '%f %f %f');
fclose(fid);
La = reshape(Ua, NUM_SYS + NUM_IN, NUM_SYS * STEP_MAX);
for i = 1 : STEP_MAX
    OAk(:, :, i) = La(1: NUM_SYS, (i-1)*NUM_SYS + 1: i* NUM_SYS)';
    OBk(:, :, i) = La(NUM_SYS + 1 : NUM_SYS + NUM_IN, (i-1)*NUM_SYS + 1 : i * NUM_SYS)';
end
for i  = 1 : STEP_MAX
    OCk(:, :, i) = eye(NUM_SYS);
end
%% Calculate K  
tic
for i= STEP_MAX: -1 : 1  
    OS(:, :, i) = OAk(:, :, i)' * (OS(:, :, i +1) - OS(:, :, i + 1) * OBk(:, :, i) / (OBk(:, :, i)' * OS(:, :, i +1) * OBk(:, :, i) + Ri) * OBk(:, :, i)' * OS(:, :, i + 1)) * OAk(:, :, i) + Qi;
end
for i = 1 : STEP_MAX
    TK(:, :, i) = (Ri + OBk(:, :, i)' * OS(:, :, i+1) * OBk(:, :, i)) \ OBk(:, :, i)' * OS(:, :, i+1) * OAk(:, :, i);
end
toc
fidtk = fopen('TK.txt','wt');
for k = 1:1:STEP_MAX
    for i = 1:1:NUM_IN
		fprintf(fidtk,'%f ',TK(i,:,k));
        fprintf(fidtk,'\n');
    end
    fprintf(fidtk,'\n');
end
fclose(fidtk);