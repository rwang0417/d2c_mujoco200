%% pendulum
% SYS_NUM = 2;
% POS_NUM = 1;
% VEL_NUM = 1;
% IN_NUM = 1;
% DOF = 1;
% QUAT = 0;
% STEP_NUM = 200;
% TRIAL_NUM = 2000;
% SIM_STEP = 0.01;
% CTRL_STEP = 0.01;
% MODEL = 'pendulum.xml';
% PERT_COEF = 0.1;
% X_INIT = [pi, 0];
% % BATCH_SIZE=20;
%% cartpole
SYS_NUM = 4;
POS_NUM = 2;
VEL_NUM = 2;
IN_NUM = 1;
DOF = 2;
QUAT = 0;
STEP_NUM = 300;
TRIAL_NUM = 2000;
SIM_STEP = 0.01;
CTRL_STEP = 0.01;
MODEL = 'cartpole.xml';
PERT_COEF = 0.12;
X_INIT = [0 0 0 0];
% % BATCH_SIZE=60;
%% read control sequence
fid = fopen('result0.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_norm = reshape(U, IN_NUM, STEP_NUM);
u_max = max(max(abs(u_norm)));
%% nominal states
X_NORM = zeros(SYS_NUM,STEP_NUM+1);
X_NORM(:,1) = X_INIT(1,:)';
mexstep('load',MODEL); % load model
mexstep('reset');
mexstep('set','qpos',X_NORM(1:POS_NUM,1),POS_NUM);
mexstep('set','qvel',X_NORM(POS_NUM+1:SYS_NUM,1),VEL_NUM);
mexstep('forward');
for i = 1 : 1 : STEP_NUM
    mexstep('set','ctrl',u_norm(:,i),IN_NUM);
    mexstep('step',1);
    X_NORM(1:POS_NUM,i+1)=mexstep('get','qpos');
    X_NORM(POS_NUM+1:SYS_NUM,i+1)=mexstep('get','qvel');
end
%% LQG
load('closedloop.mat')
tic;
% cdc2017
Ri=10^0 * eye(IN_NUM);
Qi=10^2 * eye(match_q);
W=10^1 * eye(match_q);
V=10^1 * eye(SYS_NUM);
Si = zeros(match_q, match_q, ID_STEP_NUM+1);
Pi = zeros(match_q, match_q, ID_STEP_NUM);
L = zeros(IN_NUM, match_q, ID_STEP_NUM);
K = zeros(match_q, SYS_NUM, ID_STEP_NUM);
Si(:, :, ID_STEP_NUM+1) = 10^3 * eye(match_q);
for i= ID_STEP_NUM:-1 :1  
    Si(:,:,i) = A_ID(:,:,i)'*Si(:,:,i+1)*A_ID(:,:,i)+Qi-A_ID(:,:,i)*Si(:,:,i+1)*B_ID(:,:,i)*((B_ID(:,:,i)'*Si(:,:,i+1)*B_ID(:,:,i)+Ri)\B_ID(:,:,i)'*Si(:,:,i+1)*A_ID(:,:,i));
end
for i = 1:1:ID_STEP_NUM
    L(:,:,i) = (B_ID(:,:,i)'*Si(:,:,i+1)*B_ID(:,:,i)+Ri)\B_ID(:,:,i)'*Si(:,:,i+1)*A_ID(:,:,i);
end
for i = 1:1:ID_STEP_NUM
    Pi(:,:,i+1) = A_ID(:,:,i)*(Pi(:,:,i)-Pi(:,:,i)*C_ID(:,:,i)'*((C_ID(:,:,i)*Pi(:,:,i)*C_ID(:,:,i)'+V)\C_ID(:,:,i)*Pi(:,:,i)))*A_ID(:,:,i)'+W;
end
for i = 1:1:ID_STEP_NUM
    K(:,:,i) = Pi(:,:,i)*C_ID(:,:,i)'/(C_ID(:,:,i)*Pi(:,:,i)*C_ID(:,:,i)'+V);
end
MX1=zeros(size(A_ID,1),size(A_ID,2),STEP_NUM);
MU=zeros(size(B_ID,1),size(B_ID,2),STEP_NUM);
MY=zeros(size(K,1),size(K,2),STEP_NUM);
ML=zeros(size(L,1),size(L,2),STEP_NUM);
for i = 1:1:ID_STEP_NUM-1
    MX1(:,:,i) = A_ID(:,:,i)-K(:,:,i+1)*C_ID(:,:,i+1)*A_ID(:,:,i);
    MU(:,:,i) = B_ID(:,:,i)-K(:,:,i+1)*C_ID(:,:,i+1)*B_ID(:,:,i);
    MY(:,:,i) = K(:,:,i+1);
    ML(:,:,i) = -L(:,:,i);
end
save('feedback2c.mat','ML','MX1','MU','MY','T_CON','BATCH_SIZE')
%% policy state compare
TEST_NUM = 500;
TEST_COEF = 0.2;
start=1;
Y_OP = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
Y_CL = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
X_OP = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
X_CL = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
u_test = TEST_COEF*u_max*randn(IN_NUM,TEST_NUM,ID_STEP_NUM+1);
% openloop
for j = 1 : 1 : TEST_NUM
    mexstep('reset');
    mexstep('set','qpos',X_NORM(1:POS_NUM,start),POS_NUM);
    mexstep('set','qvel',X_NORM(POS_NUM+1:SYS_NUM,start),VEL_NUM);
    mexstep('forward');
    X_OP(:,start,j) = X_INIT';
    for i = start : 1 : ID_STEP_NUM
        mexstep('set','ctrl',u_norm(:,i)+u_test(:,j,i),IN_NUM);
        mexstep('step',1);
        Y_OP(1:POS_NUM,i+1,j)=mexstep('get','qpos')'-X_NORM(1:POS_NUM,i+1);
        Y_OP(POS_NUM+1:SYS_NUM,i+1,j)=mexstep('get','qvel')'-X_NORM(POS_NUM+1:SYS_NUM,i+1);
        X_OP(1:POS_NUM,i+1,j)=mexstep('get','qpos')';
        X_OP(POS_NUM+1:SYS_NUM,i+1,j)=mexstep('get','qvel')';
    end
end
% closedloop
for j = 1 : 1 : TEST_NUM
    mexstep('reset');
    mexstep('set','qpos',X_NORM(1:POS_NUM,start),POS_NUM);
    mexstep('set','qvel',X_NORM(POS_NUM+1:SYS_NUM,start),VEL_NUM);
    mexstep('forward');
    x1=zeros(match_q,1);
    X_CL(:,start,j) = X_INIT';
    u=0;
    for i = start : 1 : ID_STEP_NUM-1
        u=-L(:,:,i)*x1;% cdc2017
        mexstep('set','ctrl',u_norm(:,i)+u_test(:,j,i)+u,IN_NUM);
        mexstep('step',1);
        Y_CL(1:POS_NUM,i+1,j)=mexstep('get','qpos')'-X_NORM(1:POS_NUM,i+1);
        Y_CL(POS_NUM+1:SYS_NUM,i+1,j)=mexstep('get','qvel')'-X_NORM(POS_NUM+1:SYS_NUM,i+1);
        X_CL(1:POS_NUM,i+1,j)=mexstep('get','qpos')';
        X_CL(POS_NUM+1:SYS_NUM,i+1,j)=mexstep('get','qvel')';
        % cdc2017
%         x2=(A_ID(:,:,i)-K(:,:,i+1)*C_ID(:,:,i+1)*A_ID(:,:,i))*x1+(B_ID(:,:,i)-K(:,:,i+1)*C_ID(:,:,i+1)*B_ID(:,:,i))*u+K(:,:,i+1)*Y_CL(:,i+1,j);
        x2=MX1(:,:,i)*x1+MU(:,:,i)*u+MY(:,:,i)*Y_CL(:,i+1,j);
        x1=x2;
        if mod(i,BATCH_SIZE)==0
            x1=T_CON(:,:,i/BATCH_SIZE)*x2;
        end
    end
end
mexstep('exit');
toc
% compare
% figure()
% plot([Y_OP(1:2,:,1)' Y_CL(1:2,:,1)'])
% legend('op1','op2','cl1','cl2')
% figure()
% plot([mean(X_OP(1:2,1:160,:),3)' mean(X_CL(1:2,1:160,1),3)' X_NORM(1:2,1:160)'])
% legend('op1','op2','cl1','cl2','norm1','norm2')
% figure()
% plot([mean(X_OP(3:4,1:160,:),3)' mean(X_CL(3:4,1:160,1),3)' X_NORM(3:4,1:160)'])
% legend('op1','op2','cl1','cl2','norm1','norm2')
figure()
plot([mean(X_CL(1:2,1:160,1),3)' X_NORM(1:2,1:160)'])
legend('cl1','cl2','norm1','norm2')
% figure()
% plot([mean(X_CL(3:4,1:160,1),3)' X_NORM(3:4,1:160)'])
% legend('cl1','cl2','norm1','norm2')