clear all;clc;warning off;tic;
%% system 1
% SYS_NUM = 2;
% POS_NUM = 1;
% VEL_NUM = 1;
% IN_NUM = 1;
% DOF = 1;
% QUAT = 0;
% STEP_NUM = 200;
% TRIAL_NUM = 1000;
% SIM_STEP = 1;
% CTRL_STEP = 1;
% PERT_COEF = 0.1;
% X_INIT = [0, 0];
% Bk=[0;1];
% Ck=[1 0;0 1];
% Dk=[1;1];
%% system 2
SYS_NUM = 3;
IN_NUM = 2;
STEP_NUM = 200;
TRIAL_NUM = 1000;
SIM_STEP = 1;
CTRL_STEP = 1;
PERT_COEF = 0.1;
X_INIT = zeros(SYS_NUM,1);
Bk=[0.4*eye(IN_NUM);0.2*ones(SYS_NUM-IN_NUM,IN_NUM)];
Ck=eye(SYS_NUM);
Dk=0.5*ones(SYS_NUM,IN_NUM);
%% read control sequence
% fid = fopen('result0.txt','r');
% U = fscanf(fid, '%f');
% fclose(fid);
% u_norm = reshape(U, IN_NUM, STEP_NUM);
% u_max = max(max(abs(u_norm)));
u_max=0.4;
u_norm=0.4*ones(IN_NUM,STEP_NUM);
%% nominal states
Ak=zeros(SYS_NUM,SYS_NUM,STEP_NUM);
X_NORM = zeros(SYS_NUM,STEP_NUM+1);
Y_NORM = zeros(SYS_NUM,STEP_NUM);
X_NORM(:,1) = X_INIT(1,:)';
for i = 1 : 1 : STEP_NUM
%     Ak(:,:,i)=expm([0 1;sin(3*(i-1))-1 -0.1]);
%     Ak(:,:,i)=[expm([0 1;sin(3*(i-1))-1 -0.1]), zeros(2,8);zeros(8,2),-0.6*eye(8)];
    Ak(:,:,i)=[0.2 0 0;0 0.3 0;0 0 0.6];
    X_NORM(:,i+1)=Ak(:,:,i)*X_NORM(:,i)+Bk*u_norm(:,i);
    Y_NORM(:,i)=Ck*X_NORM(:,i)+Dk*u_norm(:,i);
end
%% loop for each timestep
q=6;
match_q=3;
BATCH_SIZE=100;
BATCH_NUM=floor((STEP_NUM-q-3)/BATCH_SIZE);
ID_STEP_NUM = BATCH_NUM*BATCH_SIZE; % identify till the end of the last full batch
A_ID=zeros(match_q,match_q,ID_STEP_NUM);
B_ID=zeros(match_q,IN_NUM,ID_STEP_NUM);
C_ID=zeros(SYS_NUM,match_q,ID_STEP_NUM+1);
D_ID=zeros(SYS_NUM,IN_NUM,ID_STEP_NUM+1);
C_ID_END=zeros(SYS_NUM,match_q,BATCH_NUM);
D_ID_END=zeros(SYS_NUM,IN_NUM,BATCH_NUM);
O_true = zeros(SYS_NUM*(q+1),SYS_NUM,ID_STEP_NUM);
T=zeros(match_q,SYS_NUM,ID_STEP_NUM);%.............................
T_END=zeros(match_q,SYS_NUM,BATCH_NUM);
T_CON=zeros(match_q,SYS_NUM,BATCH_NUM);
% true system Ok
for k = 1:1:ID_STEP_NUM+1
    O_true(1:SYS_NUM,:,k) = Ck;
    for i = k:k+q-1
        O_anal1 = eye(SYS_NUM);
        for j = k:i
            O_anal1 = Ak(:,:,j)*O_anal1;
        end
        O_true(SYS_NUM*(i-k+1)+1:SYS_NUM*(i-k+2),:,k) = Ck*O_anal1;
    end
end
% qmc
for k=1:BATCH_SIZE:ID_STEP_NUM
    % collect data
    u_pert = PERT_COEF*u_max*randn(IN_NUM,TRIAL_NUM,q+1+BATCH_SIZE); % need q+1 control for Ok, 1 more for Ok+1, 1 more for overlap
    Y = zeros(SYS_NUM,TRIAL_NUM,q+1+BATCH_SIZE);
    for j = 1:1:TRIAL_NUM
%         x1=X_NORM(:,k);
        x1=X_NORM(:,k)+0.2*randn(SYS_NUM,1);
        for i = 1 : 1 : q+1+BATCH_SIZE
            x2=Ak(:,:,k+i-1)*x1+Bk*(u_norm(:,k+i-1)+u_pert(:,j,i));
            Y(:,j,i)=Ck*x1+Dk*(u_norm(:,k+i-1)+u_pert(:,j,i))-Y_NORM(:,k+i-1);
            x1=x2;
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
        [U,S,V] = svd(D(:,:,m));
        O(:,:,m) = U*sqrt(S(:,1:match_q));
        O_blockshift(:,:,m) = O(SYS_NUM+1:end,:,m);
        O_inverse(:,:,m) = pinv(O(1:SYS_NUM*q,:,m));
        T(:,:,k+m-1) = pinv(O(:,:,m))*O_true(:,:,k+m-1);
        if m == 1 && k > 1
            T_CON(:,:,floor(k/BATCH_SIZE))=pinv(O(:,:,1))*O_END;
        end
    end
    T_END(:,:,ceil(k/BATCH_SIZE))=pinv(O(:,:,1+BATCH_SIZE))*O_true(:,:,k+BATCH_SIZE);
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
toc;
save('qmc_acc2018_result')
%% check results
% system transformation to true system coordinate
for k = 1:1:ID_STEP_NUM
    A_tf = T(:,:,k+1)\A_ID(:,:,k)*T(:,:,k);
    B_tf = T(:,:,k+1)\B_ID(:,:,k);
    C_tf = C_ID(:,:,k)*T(:,:,k);
    if mod(k,BATCH_SIZE)==0
        A_tf=T_END(:,:,ceil(k/BATCH_SIZE))\A_ID(:,:,k)*T(:,:,k);
        B_tf=T_END(:,:,ceil(k/BATCH_SIZE))\B_ID(:,:,k);
    end
    norm(A_tf-Ak(:,:,k))
%     norm(B_tf-Bk)
%     norm(C_tf-Ck)
%     norm(D_ID(:,:,k)-Dk)
end
% simulation and estimation compare
TEST_NUM = 1;
TEST_COEF = 0.1;
u_test = TEST_COEF*u_max*randn(IN_NUM,TEST_NUM,ID_STEP_NUM+1);
Y_SIM = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
Y_EST_QMC = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
Y_EST_LS = zeros(SYS_NUM,ID_STEP_NUM+1,TEST_NUM);
% simulation result
start=1;
for j = 1 : 1 : TEST_NUM
    x1=X_NORM(:,start);
    for i = start : 1 : ID_STEP_NUM
        x2=Ak(:,:,i)*x1+Bk*(u_norm(:,i)+u_test(:,j,i));
        Y_SIM(:,i,j)=Ck*x1+Dk*(u_norm(:,i)+u_test(:,j,i))-Y_NORM(:,i);
        x1=x2;
    end
end
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
for j = 1:1:TEST_NUM
    x1=zeros(match_q,1);
    for k=1:BATCH_SIZE:ID_STEP_NUM
        for m = 1:1:BATCH_SIZE
            x2 = A_ID(:,:,m+k-1)*x1+B_ID(:,:,m+k-1)*u_test(:,j,m+k-1);
            Y_EST_QMC(:,m+k-1,j) = C_ID(:,:,m+k-1)*x1+D_ID(:,:,m+k-1)*u_test(:,j,m+k-1);
            x1=x2;
        end
        x1=T_CON(:,:,ceil(k/BATCH_SIZE))*x1;
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
plot([Y_SIM(3:4,:,1)' Y_EST_QMC(3:4,:,1)'])

figure()
% plot([Y_SIM(1:2,:,1)' Y_EST_QMC(1:2,:,1)' Y_EST_LS(1:2,:,1)'])
plot([Y_SIM(5:6,:,1)' Y_EST_QMC(5:6,:,1)'])

figure()
% plot([Y_SIM(1:2,:,1)' Y_EST_QMC(1:2,:,1)' Y_EST_LS(1:2,:,1)'])
plot([Y_SIM(7:8,:,1)' Y_EST_QMC(7:8,:,1)'])

figure()
% plot([Y_SIM(1:2,:,1)' Y_EST_QMC(1:2,:,1)' Y_EST_LS(1:2,:,1)'])
plot([Y_SIM(1:2,:,1)' Y_EST_QMC(1:2,:,1)'])
legend('sim','sim2','qmc','qmc2')
ERR=abs((Y_EST_QMC-Y_SIM)./Y_SIM);
ERR_BASE = mean(ERR(:,2:end),3);
ERR_STATE = mean(ERR_BASE,2);
ERR_STEP = mean(ERR_BASE,1);
% figure()
% plot(1:size(ERR_STATE,1),ERR_STATE(:,1));
figure
plot(1:size(ERR_STEP,2),ERR_STEP);
