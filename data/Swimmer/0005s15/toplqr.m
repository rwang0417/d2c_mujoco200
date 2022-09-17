tic;
%% system parameters
NUM_IN = 5;
NUM_SYS = 16;
STEP_MAX = 1;
%% Load from .txt file
fid = fopen('lnr_top.txt','r');
Ua  = fscanf(fid, '%f %f %f');
fclose(fid);
La = reshape(Ua, NUM_SYS + NUM_IN, NUM_SYS * STEP_MAX)';
%% dlqr
Q = 300*eye(NUM_SYS);
Q(9:16,9:16)=Q(9:16,9:16)*0;
R = .001*eye(NUM_IN);
[Kd,ss,e] = dlqr(La(:,1:NUM_SYS),La(:,NUM_SYS+1:NUM_SYS+NUM_IN),Q,R);
fidtk = fopen('TK_top.txt','wt');
for i = 1 : NUM_IN
    for j = 1 : NUM_SYS
        fprintf(fidtk,'%f ',Kd(i,j));
    end
    fprintf(fidtk,'\n');
end
fclose(fidtk);
toc;