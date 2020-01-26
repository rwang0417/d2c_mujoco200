clear all;warning off;tic;
%% tower
SYS_NUM = 32;
POS_NUM = 16;
VEL_NUM = 16;
IN_NUM = 20;
STEP_NUM = 5;
SIM_STEP = 0.01;
CTRL_STEP = 0.01;
MODEL = 'tower.xml';
ID_STEP_NUM = STEP_NUM-1;
%% read control sequence
fid = fopen('result0.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
U_cut=U(1:IN_NUM*STEP_NUM,1);
u_norm = reshape(U_cut, IN_NUM, STEP_NUM);
u_max = max(max(abs(u_norm)));
%% nominal states with free states only
ACTUAL_DOF=24;
X_NORM = zeros(ACTUAL_DOF*2,STEP_NUM+1);
mexstep('load',MODEL); % load model
mexstep('reset');
mexstep('set','qpos',X_NORM(1:ACTUAL_DOF,1),ACTUAL_DOF);
mexstep('set','qvel',X_NORM(ACTUAL_DOF+1:ACTUAL_DOF*2,1),ACTUAL_DOF);
mexstep('forward');
for i = 1 : 1 : STEP_NUM
    mexstep('set','ctrl',u_norm(:,i),IN_NUM);
    mexstep('step',1);
    X_NORM(1:ACTUAL_DOF,i+1)=mexstep('get','qpos');
    X_NORM(ACTUAL_DOF+1:ACTUAL_DOF*2,i+1)=mexstep('get','qvel');
end
mexstep('exit');
%% check sysid result
TEST_NUM = 1;
TEST_NOISE_COEF = 0.001;%1
start=1;
u_test = TEST_NOISE_COEF*u_max*randn(IN_NUM,TEST_NUM,ID_STEP_NUM+1);
Y_SIM_ORIG = zeros(ACTUAL_DOF*2,ID_STEP_NUM+1,TEST_NUM);
Y_SIM = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
Y_EST_QMC = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
Y_EST_LS = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
%% simulation result
for j = 1 : 1 : TEST_NUM
    mexstep('load',MODEL); % load model
    mexstep('reset');
    mexstep('set','qpos',X_NORM(1:ACTUAL_DOF,start),ACTUAL_DOF);
    mexstep('set','qvel',X_NORM(ACTUAL_DOF+1:ACTUAL_DOF*2,start),ACTUAL_DOF);
    mexstep('forward');
    Y_SIM_ORIG(:,start,j)=zeros(ACTUAL_DOF*2,1);
    for i = start : 1 : ID_STEP_NUM
        mexstep('set','ctrl',u_norm(:,i)+u_test(:,j,i),IN_NUM);
        mexstep('step',1);
        Y_SIM_ORIG(1:ACTUAL_DOF,i+1,j)=mexstep('get','qpos')'-X_NORM(1:ACTUAL_DOF,i+1);
        Y_SIM_ORIG(ACTUAL_DOF+1:ACTUAL_DOF*2,i+1,j)=mexstep('get','qvel')'-X_NORM(ACTUAL_DOF+1:ACTUAL_DOF*2,i+1);
    end
end
mexstep('exit');
Y_SIM(:,:,:)=Y_SIM_ORIG(1:SYS_NUM,:,:);
Y_SIM(14,:,:)=Y_SIM_ORIG(19,:,:);
Y_SIM(15,:,:)=Y_SIM_ORIG(22,:,:);
Y_SIM(30,:,:)=Y_SIM_ORIG(43,:,:);
Y_SIM(31,:,:)=Y_SIM_ORIG(45,:,:);
%% qmc need nonzero initials
% for j = 1:1:TEST_NUM
%     x1=zeros(match_q,1);
%     for i = start:1:ID_STEP_NUM
%         x2 = A_ID(:,:,i)*x1+B_ID(:,:,i)*u_test(:,j,i);
%         if i==1||mod(i,BATCH_SIZE)~=1
%         Y_EST_QMC(:,i,j) = C_ID(:,:,i)*x1+D_ID(:,:,i)*u_test(:,j,i);
%         end
%         x1=x2;
%         if mod(i,BATCH_SIZE)==0
%             x1=T_CON(:,:,i/BATCH_SIZE)*x2;
%             Y_EST_QMC(:,i+1,j)=C_ID_END(:,:,ceil(i/BATCH_SIZE))*x2+D_ID_END(:,:,ceil(i/BATCH_SIZE))*u_test(:,j,i+1);
%         end
%     end
% end
%% least square estimation result
fid = fopen('lnr.txt','r');
Ua  = fscanf(fid, '%f %f %f');
fclose(fid);
Ua_cut=Ua(1:(SYS_NUM + IN_NUM)*SYS_NUM * STEP_NUM,1);
La = reshape(Ua_cut, SYS_NUM + IN_NUM, SYS_NUM * STEP_NUM);
for i = 1 : STEP_NUM
    OAk(:, :, i) = La(1: SYS_NUM, (i-1)*SYS_NUM + 1: i* SYS_NUM)';
    OBk(:, :, i) = La(SYS_NUM + 1 : SYS_NUM + IN_NUM, (i-1)*SYS_NUM + 1 : i * SYS_NUM)';
end
for j = 1:1:TEST_NUM
    x1=zeros(SYS_NUM,1);
    for i = start:1:ID_STEP_NUM+1
        x2 = OAk(:,:,i)*x1+OBk(:,:,i)*u_test(:,j,i);
        Y_EST_LS(:,i,j) = x1;
        x1=x2;
    end
end
toc
% compare%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% need to be rewritten
figure()
% plot([Y_SIM(3:5,:,1)' Y_EST_QMC(3:5,:,1)' Y_EST_LS(3:5,:,1)'])
% plot([Y_SIM(9:10,:,1)' Y_EST_LS(9:10,:,1)'])
plot([mean(Y_SIM(1:2,:,:),3)' mean(Y_EST_LS(1:2,:,:),3)'])
% plot([Y_SIM(9:10,:,1)' Y_EST_QMC(9:10,:,1)'])
legend('sim','sim2','ls','ls2')
ERR=mean(abs((Y_EST_LS-Y_SIM)./Y_SIM),3);
ERR_BASE = mean(ERR(:,2:end),3);
ERR_STATE = mean(ERR_BASE,2);
ERR_STEP = mean(ERR_BASE,1);
% figure()
% plot(1:size(ERR_STATE,1),ERR_STATE(:,1));
figure
plot(1:size(ERR_STEP,2),ERR_STEP);
