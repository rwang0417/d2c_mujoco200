clear all;clc;warning off;tic;
% https://edoras.sdsu.edu/doc/matlab/toolbox/control/getstart/desig31a.html
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
%% cartpole
% SYS_NUM = 4;
% POS_NUM = 2;
% VEL_NUM = 2;
% IN_NUM = 1;
% DOF = 2;
% QUAT = 0;
% STEP_NUM = 300;
% TRIAL_NUM = 2000;
% SIM_STEP = 0.01;
% CTRL_STEP = 0.01;
% MODEL = 'cartpole.xml';
% PERT_COEF = 0.12;
% X_INIT = [0 0 0 0];
%% swimmer3
SYS_NUM = 10;
POS_NUM = 5;
VEL_NUM = 5;
IN_NUM = 2;
DOF = 5;
QUAT = 0;
STEP_NUM = 100;%600
TRIAL_NUM = 1500;
SIM_STEP = 0.001;
CTRL_STEP = 0.001;
MODEL = 'swimmer3.xml';
PERT_COEF = 0.05;%0.5
X_INIT = zeros(1,SYS_NUM);
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
%% loop for each timestep
q=10;
match_q=10; % 10,4 for pendulum 30,8 for cartpole 10,10 for swimmer3
BATCH_SIZE=80;
BATCH_NUM=floor((STEP_NUM-q-3)/BATCH_SIZE);
ID_STEP_NUM = BATCH_NUM*BATCH_SIZE; % identify till the end of the last full batch
A_ID=zeros(match_q,match_q,ID_STEP_NUM);
B_ID=zeros(match_q,IN_NUM,ID_STEP_NUM);
C_ID=zeros(SYS_NUM,match_q,ID_STEP_NUM+1);
D_ID=zeros(SYS_NUM,IN_NUM,ID_STEP_NUM+1);
C_ID_END=zeros(SYS_NUM,match_q,BATCH_NUM);
D_ID_END=zeros(SYS_NUM,IN_NUM,BATCH_NUM);
T=zeros(match_q,match_q,ID_STEP_NUM);
T_END=zeros(match_q,match_q,BATCH_NUM);
T_CON=zeros(match_q,match_q,BATCH_NUM);
% qmc
for k=1:BATCH_SIZE:ID_STEP_NUM
    % collect data
    u_pert = PERT_COEF*u_max*randn(IN_NUM,TRIAL_NUM,q+1+BATCH_SIZE); % need q+1 control for Ok, 1 more for Ok+1, 1 more for overlap
    Y = zeros(SYS_NUM,TRIAL_NUM,q+1+BATCH_SIZE);
    for j = 1:1:TRIAL_NUM
%             x1=X_NORM(:,k);
        x1=X_NORM(:,k)+0.0000005*randn(SYS_NUM,1);
        % init
        mexstep('reset');
        mexstep('set','qpos',x1(1:POS_NUM),POS_NUM);
        mexstep('set','qvel',x1(POS_NUM+1:SYS_NUM),VEL_NUM);
        mexstep('forward');
        Y(:,j,1)=x1-X_NORM(:,k);
        for i = 1 : 1 : q+1+BATCH_SIZE
            mexstep('set','ctrl',u_norm(:,k+i-1)+u_pert(:,j,i),IN_NUM);
            mexstep('step',1);
            Y(1:POS_NUM,j,i+1)=mexstep('get','qpos')'-X_NORM(1:POS_NUM,k+i);
            Y(POS_NUM+1:SYS_NUM,j,i+1)=mexstep('get','qvel')'-X_NORM(POS_NUM+1:SYS_NUM,k+i);
        end
    end
    % markov parameters h(i,j)
    h = zeros(SYS_NUM,IN_NUM,q+1+BATCH_SIZE,q+1+BATCH_SIZE);
    for j = 1:1:q+1+BATCH_SIZE
        for i = j:1:q+j
            if i <= q+1+BATCH_SIZE
                h(:,:,i,j) = Y(:,:,i)*u_pert(:,:,j)'./TRIAL_NUM./(PERT_COEF*u_max)^2;
            end
        end
    end
    % hankel matrix Hk
    H=zeros(SYS_NUM*(q+1),IN_NUM*(q+1),1+BATCH_SIZE);
    for m = 1:1:1+BATCH_SIZE
        for j = 1:1:1+q
            for i = j:1:1+q
                H(SYS_NUM*(i-1)+1:SYS_NUM*i,IN_NUM*(j-1)+1:IN_NUM*j,m)=h(:,:,m+i-1,m+j-1);
            end
        end
    end
    % covariance parameters r(i,j)
    r = zeros(SYS_NUM,SYS_NUM,q+1+BATCH_SIZE,q+1+BATCH_SIZE);
    for j = 1:1:q+1+BATCH_SIZE
        for i = j:1:q+j
            if i <= q+1+BATCH_SIZE
                r(:,:,i,j) = Y(:,:,i)*Y(:,:,j)'./TRIAL_NUM;
                r(:,:,j,i) = r(:,:,i,j)';
            end
        end
    end
    % covariance matrix Rk
    R=zeros(SYS_NUM*(q+1),SYS_NUM*(q+1),1+BATCH_SIZE);
    for m = 1:1:1+BATCH_SIZE
        for j = 1:1:q+1
            for i = 1:1:q+1
                R(SYS_NUM*(i-1)+1:SYS_NUM*i,SYS_NUM*(j-1)+1:SYS_NUM*j,m)=r(:,:,m+i-1,m+j-1);
            end
        end
    end
    % data matrix Dk, identified observability grammian Ok
    D=zeros(SYS_NUM*(q+1),SYS_NUM*(q+1),1+BATCH_SIZE);
    O=zeros(SYS_NUM*(q+1),match_q,1+BATCH_SIZE);
    O_blockshift=zeros(SYS_NUM*q,match_q,1+BATCH_SIZE);
    O_transform=zeros(SYS_NUM*q,match_q);
    O_inverse=zeros(match_q,SYS_NUM*q,1+BATCH_SIZE);
    for m = 1:1:1+BATCH_SIZE
        D(:,:,m) = R(:,:,m)-H(:,:,m)*H(:,:,m)'.*(PERT_COEF*u_max)^2;
%         [V_D_cal,sigma_D]  = eig(D(:,:,m));
%         sigma_D = diag(sigma_D);
%         sigma_D_pos = sigma_D(sigma_D>0);
%         ind_D_pos = sigma_D>0;
%         V_D_pos = [];
%     %     size(sigma_D_pos,1)
%         for i = 1:size(sigma_D,1)
%             if ind_D_pos(i,1) == 1
%                 V_D_pos = [V_D_pos V_D_cal(:,i)];
%             end
%         end
%         D_pos = V_D_pos*diag(sigma_D_pos)*V_D_pos';
%         [U,S,V] = svd(D_pos);
        [U,S,V] = svd(D(:,:,m));
%         Spos=S(:,sigma_D>0);
        O(:,:,m) = U*sqrt(S(:,1:match_q));
        O_blockshift(:,:,m) = O(SYS_NUM+1:end,:,m);
        O_inverse(:,:,m) = pinv(O(1:SYS_NUM*q,:,m));
        if m == 1 && k > 1
            T_CON(:,:,floor(k/BATCH_SIZE))=pinv(O(:,:,1))*O_END;
        end
    end
    O_END=O(:,:,1+BATCH_SIZE);
%     O_transform(:,:) = O(1:SYS_NUM*q,:,1+batch_size);
%     if k == 1
%         O_ref=O_transform(:,:);
%     end
%     % Tk
%     T(:,:,k) = pinv(O_transform)*O_ref;
    % Mk
    M = zeros(SYS_NUM*q,IN_NUM,BATCH_SIZE);
    for m = 1:1:BATCH_SIZE
        for i = 1:1:q
            M(SYS_NUM*(i-1)+1:SYS_NUM*i,:,m)=h(:,:,m+i,m);
        end
    end
    % ABCD
    for m = 1:1:BATCH_SIZE
        A_ID(:,:,m+k-1) = O_inverse(:,:,m+1)*O_blockshift(:,:,m);
        B_ID(:,:,m+k-1) = O_inverse(:,:,m+1)*M(:,:,m);
        C_ID(:,:,m+k-1) = O(1:SYS_NUM,:,m);
        D_ID(:,:,m+k-1) = h(:,:,m,m);
    end
    C_ID_END(:,:,ceil(k/BATCH_SIZE)) = O(1:SYS_NUM,:,BATCH_SIZE+1);
    D_ID_END(:,:,ceil(k/BATCH_SIZE)) = h(:,:,BATCH_SIZE+1,BATCH_SIZE+1);
end
mexstep('exit');
toc;
save('qmc_acc2018_result')
save('closedloop.mat','T_CON','A_ID','B_ID','C_ID','D_ID','ID_STEP_NUM','STEP_NUM','BATCH_SIZE','q','match_q')
%% check results
% simulation and estimation compare
TEST_NUM = 1;
TEST_COEF = 0.1;%1
u_test = TEST_COEF*u_max*randn(IN_NUM,TEST_NUM,ID_STEP_NUM+1);
Y_SIM = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
Y_EST_QMC = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
Y_EST_LS = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
% simulation result
start=1;
for j = 1 : 1 : TEST_NUM
    mexstep('load',MODEL); % load model
    mexstep('reset');
    mexstep('set','qpos',X_NORM(1:POS_NUM,start),POS_NUM);
    mexstep('set','qvel',X_NORM(POS_NUM+1:SYS_NUM,start),VEL_NUM);
    mexstep('forward');
    Y_SIM(:,start,j)=zeros(SYS_NUM,1);
    for i = start : 1 : ID_STEP_NUM
        mexstep('set','ctrl',u_norm(:,i)+u_test(:,j,i),IN_NUM);
        mexstep('step',1);
        Y_SIM(1:POS_NUM,i+1,j)=mexstep('get','qpos')'-X_NORM(1:POS_NUM,i+1);
        Y_SIM(POS_NUM+1:SYS_NUM,i+1,j)=mexstep('get','qvel')'-X_NORM(POS_NUM+1:SYS_NUM,i+1);
    end
end
mexstep('exit');
% % qmc estimation result
% add y
% for j = 1:1:TEST_NUM
%     y_add=zeros(SYS_NUM,1);
%     for k=1:BATCH_SIZE:ID_STEP_NUM
%         x1=zeros(match_q,1);
%         for m = 1:1:BATCH_SIZE
%             x2 = A_ID(:,:,m+k-1)*x1+B_ID(:,:,m+k-1)*u_test(:,j,m+k-1);
%             Y_EST_QMC(:,m+k-1,j) = y_add+C_ID(:,:,m+k-1)*x1+D_ID(:,:,m+k-1)*u_test(:,j,m+k-1);
%             x1=x2;
%         end
%         y_add=C_ID_END(:,:,ceil(k/BATCH_SIZE))*x2+D_ID_END(:,:,ceil(k/BATCH_SIZE))*u_test(:,j,BATCH_SIZE+k);
%     end
% end
% transform x
% for j = 1:1:TEST_NUM
%     x1=zeros(match_q,1);
%     for k=1:BATCH_SIZE:ID_STEP_NUM
%         for m = 1:1:BATCH_SIZE
%             x2 = A_ID(:,:,m+k-1)*x1+B_ID(:,:,m+k-1)*u_test(:,j,m+k-1);
%             Y_EST_QMC(:,m+k-1,j) = C_ID(:,:,m+k-1)*x1+D_ID(:,:,m+k-1)*u_test(:,j,m+k-1);
%             x1=x2;
%         end
%         x1=T_CON(:,:,ceil(k/BATCH_SIZE))*x1;
%     end
% end

% need nonzero initials
for j = 1:1:TEST_NUM
    x1=zeros(match_q,1);
    for i = start:1:ID_STEP_NUM
        x2 = A_ID(:,:,i)*x1+B_ID(:,:,i)*u_test(:,j,i);
%         if i==1||mod(i,BATCH_SIZE)~=1
        Y_EST_QMC(:,i,j) = C_ID(:,:,i)*x1+D_ID(:,:,i)*u_test(:,j,i);
%         end
        x1=x2;
        if mod(i,BATCH_SIZE)==0
            x1=T_CON(:,:,i/BATCH_SIZE)*x2;
%             Y_EST_QMC(:,i+1,j)=C_ID_END(:,:,ceil(i/BATCH_SIZE))*x2+D_ID_END(:,:,ceil(i/BATCH_SIZE))*u_test(:,j,i+1);
        end
    end
end

% for j = 1:1:TEST_NUM
%     x1=zeros(match_q,1);
%     for i = start:1:ID_STEP_NUM
%         x2 = A_ID(:,:,i)*x1+B_ID(:,:,i)*u_test(:,j,i);
%         Y_EST_QMC(:,i,j) = C_ID(:,:,i)*x1+D_ID(:,:,i)*u_test(:,j,i);
%         x1=x2;
%     end
% end

% % least square estimation result
% fid = fopen('lnr.txt','r');
% Ua  = fscanf(fid, '%f %f %f');
% fclose(fid);
% % STEP_NUM=1600;
% La = reshape(Ua, SYS_NUM + IN_NUM, SYS_NUM * STEP_NUM);
% for i = 1 : STEP_NUM
%     OAk(:, :, i) = La(1: SYS_NUM, (i-1)*SYS_NUM + 1: i* SYS_NUM)';
%     OBk(:, :, i) = La(SYS_NUM + 1 : SYS_NUM + IN_NUM, (i-1)*SYS_NUM + 1 : i * SYS_NUM)';
% end
% for j = 1:1:TEST_NUM
%     x1=zeros(SYS_NUM,1);
%     for i = start:1:ID_STEP_NUM+1
%         x2 = OAk(:,:,i)*x1+OBk(:,:,i)*u_test(:,j,i);
%         Y_EST_LS(:,i,j) = x1;
%         x1=x2;
%     end
% end

% compare
figure()
% plot([Y_SIM(1:2,:,1)' Y_EST_QMC(1:2,:,1)' Y_EST_LS(1:2,:,1)'])
plot([Y_SIM(9:10,:,1)' Y_EST_QMC(9:10,:,1)'])
legend('sim','sim2','qmc','qmc2')
ERR=abs((Y_EST_QMC-Y_SIM)./Y_SIM);
ERR_BASE = mean(ERR(:,2:end),3);
ERR_STATE = mean(ERR_BASE,2);
ERR_STEP = mean(ERR_BASE,1);
% figure()
% plot(1:size(ERR_STATE,1),ERR_STATE(:,1));
figure
plot(1:size(ERR_STEP,2),ERR_STEP);
