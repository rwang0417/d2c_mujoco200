tic;
%% system parameters
NUM_IN = 2;
NUM_SYS = 10;
STEP_MAX = 1;
%% Load from .txt file
fid = fopen('lnr_top.txt','r');
Ua  = fscanf(fid, '%f %f %f');
fclose(fid);
La = reshape(Ua, NUM_SYS + NUM_IN, NUM_SYS * STEP_MAX)';
%% dlqr
Q = 2.5*eye(10);
Q(6:10,6:10)=Q(6:10,6:10)*3;
R = .001*eye(2);
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