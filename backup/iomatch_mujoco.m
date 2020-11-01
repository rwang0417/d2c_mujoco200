clear all;clc;tic;
%% pendulum
% POS_NUM = 1;
% VEL_NUM = 1;
% SYS_NUM = 2;
% IN_NUM = 1;
% OUT_NUM= 2;
% MODEL = 'pendulum.xml';
% STEP_NUM = 200;
% TRIAL_NUM = 100;
% PERT_COEF = .2;
% X_INIT = [pi;0];
% Ck=eye(SYS_NUM);%[1 0];%[0 1];%

%% cartpole
% POS_NUM = 2;
% VEL_NUM = 2;
% SYS_NUM = 4;
% IN_NUM = 1;
% OUT_NUM= 4;
% MODEL = 'cartpole.xml';
% STEP_NUM = 30;
% TRIAL_NUM = 600;
% PERT_COEF = .2;
% X_INIT = zeros(SYS_NUM,1);
% Ck=eye(SYS_NUM);%[1 0 0 0;0 1 0 0];%[1 0 0 0];%[0 0 1 0;0 0 0 1];%[0.3 0.4 0.5 0.6];%[0.5 1 0.2 0.4; 0 0.2 0.5 0.9;0 0.3 0.7 0.2;0.2 0.3 0.1 0.4];%

%% swimmer3
% POS_NUM = 5;
% VEL_NUM = 5;
% SYS_NUM = 10;
% IN_NUM = 2;
% OUT_NUM = 5;
% MODEL = 'swimmer3.xml';
% STEP_NUM = 1600;
% TRIAL_NUM = 400;
% PERT_COEF = .01;
% X_INIT = zeros(SYS_NUM,1);
% Ck=[eye(POS_NUM) zeros(POS_NUM,VEL_NUM) ];%eye(SYS_NUM);%[zeros(3,2) eye(3) zeros(3,5)];%[zeros(POS_NUM,POS_NUM) eye(VEL_NUM)];%

%% swimmer6
% POS_NUM = 8;
% VEL_NUM = 8;
% SYS_NUM = 16;
% IN_NUM = 5;
% OUT_NUM = 3;
% MODEL = 'swimmer6.xml';
% STEP_NUM = 1500;
% TRIAL_NUM = 400;
% PERT_COEF = .01;
% X_INIT = zeros(SYS_NUM,1);
% Ck=[zeros(3,2) [1;0;0] zeros(3,1) [0;1;0] zeros(3,1) [0;0;1] zeros(3,1) zeros(3,8)];%[zeros(4,2) [1;0;0;0] zeros(4,1) [0;1;0;0] [0;0;1;0] zeros(4,1) [0;0;0;1] zeros(4,8)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%eye(SYS_NUM);%[zeros(POS_NUM,POS_NUM) eye(VEL_NUM)];%

%% swimmer15
POS_NUM = 17;
VEL_NUM = 17;
SYS_NUM = 34;
IN_NUM = 14;
OUT_NUM = 8;
MODEL = 'swimmer15.xml';
STEP_NUM = 2400;
TRIAL_NUM = 600;
PERT_COEF = .2;
X_INIT = zeros(SYS_NUM,1);
Ck=[zeros(8,2) [1;zeros(7,1)] zeros(8,1) [0;1;zeros(6,1)] zeros(8,1) [0;0;1;zeros(5,1)] zeros(8,1) [0;0;0;1;zeros(4,1)] zeros(8,1) [zeros(4,1);1;zeros(3,1)] zeros(8,1) [zeros(5,1);1;zeros(2,1)] zeros(8,1) [zeros(6,1);1;0] zeros(8,1) [zeros(7,1);1] zeros(8,17)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%eye(SYS_NUM);%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%[zeros(POS_NUM,POS_NUM) eye(VEL_NUM)];%

%% fish
% POS_NUM = 14;
% VEL_NUM = 13;
% SYS_NUM = 27;
% IN_NUM = 6;
% OUT_NUM = 7;
% MODEL = 'fish.xml';
% STEP_NUM = 1200;
% TRIAL_NUM = 600;
% PERT_COEF = .001;
% X_INIT = [0;0;0;1;0;0;0;zeros(SYS_NUM-7,1)];
% Ck=[zeros(7,7) eye(7) zeros(7,13)];%[eye(POS_NUM) zeros(POS_NUM,VEL_NUM) ];%[eye(7) zeros(7,20); zeros(7,14) eye(7) zeros(7,6)];%[eye(7) zeros(7,20)];%eye(SYS_NUM);%

%% tuning
% arma fitting parameters
q=5;
qu=5;

% lqr cost parameters
sig_q = 10^2;
sig_f = 10^6;
sig_r = 10^0;

%% read nominal control sequence
fid = fopen('result0_s15.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_norm = reshape(U, IN_NUM, STEP_NUM);
u_max = max(max(abs(u_norm)));

%% generate nominal state trajectory
Y_NORM = zeros(OUT_NUM,STEP_NUM+1);
x2 = zeros(SYS_NUM, 1);
mexstep('load',MODEL); % load model
mexstep('reset'); % reset all states and controls
mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM); % set initial states
mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
mexstep('forward'); % update sites and sensors
Y_NORM(:,1)=Ck*X_INIT(:,1);
for i = 1 : 1 : STEP_NUM
    mexstep('set','ctrl',u_norm(:,i),IN_NUM);
    mexstep('step',1);
    x2(1:POS_NUM,1)=mexstep('get','qpos')';
    x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel')';
    Y_NORM(:,i+1)=Ck*x2;
end

%% collect data for arma model fitting
delta_u=PERT_COEF*u_max*randn(IN_NUM*(STEP_NUM+1),TRIAL_NUM); % generate random inputs du
delta_y=zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
for j=1:1:TRIAL_NUM
    mexstep('reset');
    mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
    mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
    mexstep('forward');
    for i=1:1:STEP_NUM
        mexstep('set','ctrl',u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        mexstep('step',1);
        x2(1:POS_NUM,1)=mexstep('get','qpos');
        x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
        delta_y(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1); % record output dy
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
    mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
    mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
    mexstep('forward');
    for i=1:1:STEP_NUM
        mexstep('set','ctrl',u_norm(:,i)+ucheck(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        mexstep('step',1);
        x2(1:POS_NUM,1)=mexstep('get','qpos');
        x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
        y_sim(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
    end
end

y_pred(OUT_NUM*(STEP_NUM-q-1)+1:OUT_NUM*(STEP_NUM+1),:)=y_sim(OUT_NUM*(STEP_NUM-q-1)+1:OUT_NUM*(STEP_NUM+1),:); % manually match the first few steps
for i=max(q,qu)+2:1:STEP_NUM % start to apply input after having enough data
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

%% save data to .mat file for closed-loop testing and creating animation in MuJoCo C++ software
MTK=TK;
MCK=Ck;
MQ=q;
MQU=qu;
save('feedbackioid.mat','MTK','MCK','MQ','MQU');

%% closed-loop performance test
delta_u_test=[0.2*u_max*randn(IN_NUM*(STEP_NUM+1),TEST_NUM);zeros(IN_NUM*(qu+1),TEST_NUM)];
y_open=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM); % open-loop output, no feedback
y_closed=zeros(OUT_NUM*(STEP_NUM+1),TEST_NUM); % closed-loop output, with the above feedback
u_feedback=zeros(IN_NUM*(STEP_NUM+1),TEST_NUM); % input feedback term
for j=1:1:TEST_NUM
    % open-loop
    mexstep('reset');
    mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
    mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
    mexstep('forward');
    for i=1:1:STEP_NUM
        if i >= max(q,qu)+2 % start to apply perturbation after certain steps
            mexstep('set','ctrl',u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        else
            mexstep('set','ctrl',u_norm(:,i),IN_NUM);
        end
        mexstep('step',1);
        x2(1:POS_NUM,1)=mexstep('get','qpos');
        x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
        y_open(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Y_NORM(:,i+1);
    end
    % closed-loop
    mexstep('reset');
    mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
    mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
    mexstep('forward');
    for i=1:1:STEP_NUM
        x1(1:POS_NUM,1)=mexstep('get','qpos');
        x1(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
        y_closed(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+2),j)=Ck*x1-Y_NORM(:,i);
        if i >= max(q,qu)+2 % start to apply perturbation after certain steps
            u_feedback(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)=-TK(:,:,i)*[y_closed(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),j);u_feedback(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+1+qu),j)];
            mexstep('set','ctrl',u_norm(:,i)+delta_u_test(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)+u_feedback(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
        else
            mexstep('set','ctrl',u_norm(:,i),IN_NUM);
        end
        mexstep('step',1);
    end
    x1(1:POS_NUM,1)=mexstep('get','qpos');
    x1(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel');
    y_closed(1:OUT_NUM,j)=Ck*x1-Y_NORM(:,STEP_NUM+1);
end
% mexstep('exit')

% plot open-loop closed-loop comparison
y_closed_avg=mean((y_closed),2);
y_open_avg=mean((y_open),2);
ropenavg=reshape(y_open_avg,OUT_NUM,STEP_NUM+1);
rclosedavg=reshape(y_closed_avg,OUT_NUM,STEP_NUM+1);
figure()
plot([fliplr(ropenavg(1,1:STEP_NUM-max(q,qu)))' fliplr(rclosedavg(1,1:STEP_NUM-max(q,qu)))']); % plot only the first state
legend('openloop1','closedloop1');
closedloop=mean(abs(y_closed_avg))
toc;