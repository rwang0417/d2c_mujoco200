clear all;clc;tic;
%% dbar3d
% NODE_NUM = 4;
% POS_NUM = 3 * NODE_NUM;
% VEL_NUM = 3 * NODE_NUM;
% SYS_NUM = POS_NUM + VEL_NUM;
% IN_NUM = 7;
% OUT_NUM= 24;
% MODEL = 'dbar3d.xml';
% STEP_NUM = 200;
% TRIAL_NUM = 400;
% PERT_COEF = 2;
% X_INIT = zeros(SYS_NUM,1);
% Ck=eye(SYS_NUM);%[eye(6) zeros(6,18)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%

%% t1d13d
% NODE_NUM = 11;
% POS_NUM = 3 * NODE_NUM;
% VEL_NUM = 3 * NODE_NUM;
% SYS_NUM = POS_NUM + VEL_NUM;
% IN_NUM = 20;
% OUT_NUM= 66;
% MODEL = 't1d1_3d.xml';
% STEP_NUM = 200;
% TRIAL_NUM = 500;
% PERT_COEF = .1;
% X_INIT = zeros(SYS_NUM,1);
% Ck=eye(SYS_NUM);%[eye(12) zeros(12,54) ];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%

%% t2d13d
NODE_NUM = 25;
POS_NUM = 3 * NODE_NUM;
VEL_NUM = 3 * NODE_NUM;
SYS_NUM = POS_NUM + VEL_NUM;
IN_NUM = 46;
OUT_NUM = 24;
MODEL = 't2d1_3d.xml';
STEP_NUM = 200;
TRIAL_NUM = 800;
PERT_COEF = 0.4;
X_INIT = zeros(SYS_NUM,1);
Ck=[eye(6) zeros(6,144);zeros(6,50) eye(6) zeros(6,94);zeros(6,100) eye(6) zeros(6,44);zeros(6,144) eye(6)];%[eye(3) zeros(3,147);zeros(3,28) eye(3) zeros(3,119);zeros(3,56) eye(3) zeros(3,91);zeros(3,84) eye(3) zeros(3,63);zeros(3,112) eye(3) zeros(3,35);zeros(3,147) eye(3)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM)];%eye(SYS_NUM);%

%% tuning
% arma fitting parameters
q = 3;
qu = 3;

% lqr cost parameters
sig_q = 10^4;
sig_f = 10^6;
sig_r = 10^0;

%% read nominal control sequence
fid = fopen('result0_t2d1.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_norm = reshape(U, IN_NUM, STEP_NUM);
u_max = max(max(abs(u_norm)));

%% generate nominal state trajectory
Y_NORM = zeros(OUT_NUM,STEP_NUM+1);
mexstep('load',MODEL); % load model
mexstep('reset'); % reset all states and controls
mexstep('forward');
N_array=mexstep('get','site_xpos');
Nd_array=mexstep('get','sensordata');
x2(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM); % the site_xpos data read from MuJoCo is 3 by NODE_NUM
x2(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
Y_NORM(:,1)=Ck*x2;
for i = 1 : 1 : STEP_NUM
    mexstep('set','ctrl',u_norm(:,i),IN_NUM);
    mexstep('step',1);
    N_array=mexstep('get','site_xpos');
    Nd_array=mexstep('get','sensordata');
    x2(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
    x2(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
    Y_NORM(:,i+1)=Ck*x2;
end

%% collect data for arma model fitting
delta_u=PERT_COEF*u_max*randn(IN_NUM*(STEP_NUM+1),TRIAL_NUM); % generate random inputs du
delta_y=zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
for j=1:1:TRIAL_NUM
    mexstep('reset');
    mexstep('forward');
    for i=1:1:STEP_NUM
        mexstep('set','ctrl',u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        mexstep('step',1);
        N_array=mexstep('get','site_xpos');
        Nd_array=mexstep('get','sensordata');
        x2(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
        x2(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
        delta_y(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
    end
end 

%% least square fitting for the arma model
fitcoef=zeros(OUT_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM); % M1 * fitcoef = delta_y
for i=max(q,qu)+2:1:STEP_NUM % skip the first few steps to wait for enough data
    M1=[delta_y(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
    fitcoef(:,:,i)=delta_y(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)*M1'/(M1*M1');
    r(i)=sqrt(mean(mean((delta_y(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)-fitcoef(:,:,i)*M1).^2,1))); % residual
end

%% prediction check with rolling window
TEST_NUM=1; % number of monte-carlo runs to verify the fitting result
ucheck=0.01*u_max*randn(IN_NUM*(STEP_NUM+1),TEST_NUM); % input used for checking
y_sim=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM); % output from real system
y_pred=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM); % output from arma model
for j=1:1:TEST_NUM
    mexstep('reset');
    mexstep('forward');
    for i=1:1:STEP_NUM
        mexstep('set','ctrl',u_norm(:,i)+ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        mexstep('step',1);
        N_array=mexstep('get','site_xpos');
        Nd_array=mexstep('get','sensordata');
        x2(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
        x2(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
        y_sim(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
    end
end
y_pred(OUT_NUM*(STEP_NUM-q-1)+1:OUT_NUM*(STEP_NUM+1),:)=y_sim(OUT_NUM*(STEP_NUM-q-1)+1:OUT_NUM*(STEP_NUM+1),:);
for i=max(q,qu)+2:1:STEP_NUM
    M2=[y_pred(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
    y_pred(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)=fitcoef(:,:,i)*M2;
end

% plot y_sim and y_pred to check if they match
rpred=reshape(y_pred,OUT_NUM,STEP_NUM+1,TEST_NUM);
rsim=reshape(y_sim,OUT_NUM,STEP_NUM+1,TEST_NUM);
figure()
plot([fliplr(rpred(1,:,1))' fliplr(rsim(1,:,1))']); % plot only the first state
legend('pred1','sim1');

%% time-varying LQR
% construct augmented Ak, Bk
A_aug = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM);
B_aug = zeros(OUT_NUM*q+IN_NUM*(qu-1),IN_NUM,STEP_NUM);
for i=max(q,qu)+2:1:STEP_NUM 
    A_aug(:,:,i)=[fitcoef(:,1:OUT_NUM*q,i),fitcoef(:,OUT_NUM*q+IN_NUM+1:end,i);
      eye((q-1)*OUT_NUM),zeros((q-1)*OUT_NUM,OUT_NUM+IN_NUM*(qu-1));
      zeros(IN_NUM*(qu-1),OUT_NUM*q),[zeros(IN_NUM,IN_NUM*(qu-1));eye(IN_NUM*(qu-2)) zeros(IN_NUM*(qu-2),IN_NUM)]];
    B_aug(:,:,i)=[fitcoef(:,OUT_NUM*q+1:OUT_NUM*q+IN_NUM,i);zeros(OUT_NUM*(q-1),IN_NUM);eye(IN_NUM*min(qu-1,1));zeros(IN_NUM*(qu-2),IN_NUM)];
end

% LQR cost matrices
Ri = sig_r *1* eye(IN_NUM);
Qi = sig_q * eye(OUT_NUM*q+IN_NUM*(qu-1));Qi(OUT_NUM+1:end,OUT_NUM+1:end)=0*Qi(OUT_NUM+1:end,OUT_NUM+1:end);
OS = zeros(OUT_NUM*q+IN_NUM*(qu-1), OUT_NUM*q+IN_NUM*(qu-1), STEP_NUM+1);
TK = zeros(IN_NUM, OUT_NUM*q+IN_NUM*(qu-1), STEP_NUM);
OS(:, :, STEP_NUM+1) = sig_f * eye(OUT_NUM*q+IN_NUM*(qu-1));OS(OUT_NUM+1:end,OUT_NUM+1:end)=0*OS(OUT_NUM+1:end,OUT_NUM+1:end);
for i= STEP_NUM:-1:max(q,qu)+2  
    OS(:, :, i) = A_aug(:, :, i)' * (OS(:, :, i +1) - OS(:, :, i + 1) * B_aug(:, :, i) / (B_aug(:, :, i)' * OS(:, :, i +1) * B_aug(:, :, i) + Ri) * B_aug(:, :, i)' * OS(:, :, i + 1)) * A_aug(:, :, i) + Qi;
end
for i = max(q,qu)+2:1:STEP_NUM
    TK(:, :, i) = (Ri + B_aug(:, :, i)' * OS(:, :, i+1) * B_aug(:, :, i)) \ B_aug(:, :, i)' * OS(:, :, i+1) * A_aug(:, :, i);
end

%% LQG Addition
Wi = 1e0*eye(IN_NUM);
Vi = 1e-2*eye(OUT_NUM);

D_aug = zeros(OUT_NUM*q+IN_NUM*(qu-1),IN_NUM);
PS = zeros(OUT_NUM*q+IN_NUM*(qu-1), OUT_NUM*q+IN_NUM*(qu-1), STEP_NUM+1);
LK = zeros(OUT_NUM*q+IN_NUM*(qu-1), OUT_NUM, STEP_NUM);
for i=max(q,qu)+2:1:STEP_NUM
    C_aug = [eye(OUT_NUM) zeros(OUT_NUM,OUT_NUM*(q-1)+IN_NUM*(qu-1))];
    D_aug1 = zeros(OUT_NUM,IN_NUM);
    for j = 1:qu
        D_aug1 = D_aug1 + fitcoef(:,OUT_NUM*q+IN_NUM*(qu-1)+1:OUT_NUM*q+IN_NUM*qu,i);
    end
    D_aug(:,:,i) = [D_aug1;zeros(OUT_NUM*(q-1)+IN_NUM*(qu-1),IN_NUM)];
end

for i= 1:1:STEP_NUM
    PS(:, :, i+1) = A_aug(:,:,i) * (PS(:, :, i) - PS(:, :, i) * C_aug' / (C_aug * PS(:, :, i) * C_aug' + Vi) * C_aug * PS(:, :, i)) * A_aug(:,:,i)' + D_aug(:,:,i)*Wi*D_aug(:,:,i)';
    LK(:, :, i) = PS(:, :, i)*C_aug'/ (C_aug * PS(:, :, i) * C_aug' + Vi);
end

%% save data to .mat file for closed-loop testing and creating animation in MuJoCo C++ software
MYE=zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM-1);
MU1=zeros(OUT_NUM*q+IN_NUM*(qu-1),IN_NUM,STEP_NUM-1);
MYM=zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM,STEP_NUM-1);
MTK=TK;
MCK=Ck;
MQ=q;
MQU=qu;
for i= 1:1:STEP_NUM-1
    MYE(:, :, i) = A_aug(:,:,i)-LK(:,:,i+1)*C_aug*A_aug(:,:,i);
    MU1(:,:,i) = B_aug(:,:,i)-LK(:,:,i+1)*C_aug*B_aug(:,:,i);
    MYM(:, :, i) = LK(:,:,i+1);
end
save('feedbackiolqg.mat','MTK','MCK','MQ','MQU','MYE','MU1','MYM');

%% closed-loop performance test
vk = .01*randn(OUT_NUM,STEP_NUM+1,TEST_NUM);
delta_u_test=[0.4*u_max*randn(IN_NUM*(STEP_NUM+1),TEST_NUM);zeros(IN_NUM*(qu+1),TEST_NUM)];
y_open=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM);
y_closed=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM);
y_est = zeros(OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM+1,TEST_NUM);
u_feedback=zeros(IN_NUM*(STEP_NUM+1),TEST_NUM);
for j=1:1:TEST_NUM
    % open-loop
    mexstep('reset');
    mexstep('forward');
    for i=1:1:STEP_NUM
        if i >= max(q,qu)+3
            mexstep('set','ctrl',u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        else
            mexstep('set','ctrl',u_norm(:,i),IN_NUM);
        end
        mexstep('step',1);
        N_array=mexstep('get','site_xpos');
        Nd_array=mexstep('get','sensordata');
        x2(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
        x2(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
        y_open(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1)+vk(:,i+1,j);
    end
    % closed-loop
    mexstep('reset');
    mexstep('forward');
    for i=1:1:STEP_NUM
        N_array=mexstep('get','site_xpos');
        Nd_array=mexstep('get','sensordata');
        x1(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
        x1(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
        y_closed(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+2),j)=Ck*x1-Y_NORM(:,i)+vk(:,i,j);
        if i >= max(q,qu)+3
            % LQG
            y_est(:,i,j)=A_aug(:,:,i-1)*y_est(:,i-1,j)+B_aug(:,:,i-1)*u_feedback(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+3),j)+LK(:,:,i)*(y_closed(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+2),j)-C_aug*(A_aug(:,:,i-1)*y_est(:,i-1,j)+B_aug(:,:,i-1)*u_feedback(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+3),j)));
            u_feedback(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)=-TK(:,:,i)*y_est(:,i,j); 
            
            % LQR
%             u_feedback(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)=-TK(:,:,i)*[y_closed(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),j);u_feedback(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+1+qu),j)];
            mexstep('set','ctrl',u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)+u_feedback(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        else
            mexstep('set','ctrl',u_norm(:,i),IN_NUM);
        end
        mexstep('step',1);
    end
    N_array=mexstep('get','site_xpos');
    Nd_array=mexstep('get','sensordata');
    x1(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
    x1(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
    y_closed(1:OUT_NUM,j)=Ck*x1-Y_NORM(:,STEP_NUM+1)+vk(:,STEP_NUM+1,j);
end
% mexstep('exit')

% plot open-loop closed-loop comparison
y_closed_avg=mean((y_closed),2);
y_open_avg=mean((y_open),2);
ropenavg=reshape(y_open_avg,OUT_NUM,STEP_NUM+1);
rclosedavg=reshape(y_closed_avg,OUT_NUM,STEP_NUM+1);
figure()
plot([fliplr(ropenavg(1,1:STEP_NUM-max(q,qu)))' fliplr(rclosedavg(1,1:STEP_NUM-max(q,qu)))']);
legend('openloop1','closedloop1');
closedloop=mean(abs(y_closed_avg))
toc;