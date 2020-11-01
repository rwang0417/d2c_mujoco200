clear all;clc;warning off;tic;
%% dbar3d
% NODE_NUM = 4;
% POS_NUM = 3 * NODE_NUM;
% VEL_NUM = 3 * NODE_NUM;
% SYS_NUM = POS_NUM + VEL_NUM;
% IN_NUM = 7;
% OUT_NUM= 24;
% MODEL = 'dbar3d.xml';
% STEP_NUM = 200;
% lb=0;ub=inf;
% C_s_in = [2 5;1 3;3 4;1 4;2 6;2 7;2 8];
% X_TARGET = [zeros(3,1);0;0;2.5;zeros(SYS_NUM-6,1)];
% Ck=eye(SYS_NUM);%[eye(6) zeros(6,18)];%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM)];%
% q=qu=1,2,3
% alpha=0.5;
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 60;
% ITE_NUM = 20;
% STEP_DIV = 1; SKIPo = 4; SKIP = 300; 
% Ri = 1*10^-2 * eye(IN_NUM);
% cx = 8*10^0; cv = 2*10^0;
% ctx = 10*10^2; ctv = 1*10^3;
% Qi(4:6,4:6) = cx * [1 0 0;0 1 0;0 0 1];
% Qf(4:6,4:6) = ctx * [1 0 0;0 1 0;0 0 1];

%% t1d13d
NODE_NUM = 11;
POS_NUM = 3 * NODE_NUM;
VEL_NUM = 3 * NODE_NUM;
SYS_NUM = POS_NUM + VEL_NUM;
IN_NUM = 20;
OUT_NUM = 66;
MODEL = 't1d1_3dsmall.xml';
STEP_NUM = 200;
lb=0;ub=inf;
C_s_in = [2 12;1 9;1 6;9 6;2 5;4 11;4 8;8 11;3 12;10 12;7 12;3 5;5 10;5 7;3 10;3 7;10 7;5 13;5 15;5 14];
X_TARGET = [zeros(12,1);0;0;13;zeros(SYS_NUM-15,1)];
Ck=eye(SYS_NUM);%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM) ];%[eye(12) zeros(12,54)];%
% q=qu=1
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 150;
% ITE_NUM = 20;
% STEP_DIV = 1; SKIPo = 5; SKIP = 300; 
% Ri = 2*10^-2 * eye(IN_NUM);
% cx = 1*10^0; cv = 2*10^0;
% ctx = 10*10^2; ctv = 1*10^3;
% Qi(13:15,13:15) = cx * [1 0 0;0 2 0;0 0 3];
% Qf(13:15,13:15) = ctx * [1 0 0;0 2 0;0 0 3];
% q=qu=2
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 150;
% ITE_NUM = 20;
% STEP_DIV = 1; SKIPo = 5; SKIP = 300; 
% Ri = 2*10^-2 * eye(IN_NUM);
% cx = 4*10^0; cv = 2*10^0;
% ctx = 1*10^2; ctv = 1*10^3;
% Qi(13:15,13:15) = cx * [1 0 0;0 2 0;0 0 3];
% Qf(13:15,13:15) = ctx * [1 0 0;0 2 0;0 0 3];

%% t2d13d
% NODE_NUM = 25;
% POS_NUM = 3 * NODE_NUM;
% VEL_NUM = 3 * NODE_NUM;
% SYS_NUM = POS_NUM + VEL_NUM;
% IN_NUM = 46;
% OUT_NUM = 150;
% MODEL = 't2d1_3d.xml';
% STEP_NUM = 200;
% X_TARGET = [zeros(48,1);0.2;0.2;7;zeros(SYS_NUM-51,1)];
% Ck=eye(SYS_NUM);%[eye(POS_NUM) zeros(VEL_NUM,VEL_NUM)];%[eye(6) zeros(6,144);zeros(6,50) eye(6) zeros(6,94);zeros(6,100) eye(6) zeros(6,44);zeros(6,144) eye(6)];%
% q=qu=1
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 300;
% ITE_NUM = 15;
% STEP_DIV = 1; SKIPo = 5; SKIP = 300; 
% Ri = 4*10^-2 * eye(IN_NUM);
% cx = 1*10^0; cv = 1*10^-2;
% ctx = 20*10^2; ctv = 1*10^-1;
% Qi(49:51,49:51) = cx * [1 0 0;0 1 0;0 0 1];
% Qf(49:51,49:51) = ctx * [1 0 0;0 1 0;0 0 1];
% Qi(76:end,76:end) = cv * eye(75);
% Qf(76:end,76:end) = ctv * eye(75);
% q=qu=2
% ID_PERT_COEF = 0.01;
% TRIAL_NUM = 300;
% ITE_NUM = 6;
% STEP_DIV = 1; SKIPo = 5; SKIP = 300; 
% Ri = 1*10^-1 * eye(IN_NUM);
% cx = 1*10^0; cv = 1*10^2;
% ctx = 80*10^2; ctv = 1*10^4;
% Qi(49:51,49:51) = cx * [1 0 0;0 1 0;0 0 2];
% Qf(49:51,49:51) = ctx * [1 0 0;0 1 0;0 0 2];

%% tuning 
ID_PERT_COEF = 0.001;
TRIAL_NUM = 150;
ITE_NUM = 4;
STEP_DIV = 1; SKIPo = 4; SKIP = 300; 
q = 1;
qu = 1;
Ri = 2*10^-2 * eye(IN_NUM);
cx = 1*10^0; cv = 1*10^-2;
ctx = 10*10^2; ctv = 1*10^-1;
Qi = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1));
Qf = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1));
% dbar3d
% Qi(4:6,4:6) = cx * [1 0 0;0 1 0;0 0 1];
% Qf(4:6,4:6) = ctx * [1 0 0;0 1 0;0 0 1];
% t1d13d
Qi(13:15,13:15) = cx * [1 0 0;0 2 0;0 0 3];
Qf(13:15,13:15) = ctx * [1 0 0;0 2 0;0 0 3];
% t2d13d
% Qi(49:51,49:51) = cx * [1 0 0;0 1 0;0 0 2];
% Qf(49:51,49:51) = ctx * [1 0 0;0 1 0;0 0 2];
% Qi(76:end,76:end) = cv * eye(75);
% Qf(76:end,76:end) = ctv * eye(75);

%% variables
dz_seq=zeros(OUT_NUM*(STEP_NUM+1),1);
dzt_seq=zeros(OUT_NUM*(STEP_NUM+1),1);
u_norm = [zeros(IN_NUM,SKIPo) 0.1*randn(IN_NUM,STEP_NUM-SKIPo)];%zeros(IN_NUM,STEP_NUM);
du_seq = zeros(IN_NUM*(STEP_NUM+1),1);
Sk = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM+1); 
Sk(:,:,STEP_NUM+1) = Qf;
vk = zeros(OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM+1);
K = zeros(IN_NUM,OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM);
Kv = zeros(IN_NUM,OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM);
Ku = zeros(IN_NUM,IN_NUM,STEP_NUM);
Quu = zeros(IN_NUM,IN_NUM,STEP_NUM);
dufc = zeros(IN_NUM,STEP_NUM);
uf = zeros(IN_NUM,STEP_NUM);
kt  = zeros(IN_NUM,STEP_NUM);
cost = zeros(ITE_NUM,1);
fitcoef = zeros(OUT_NUM,OUT_NUM*q+IN_NUM*qu,STEP_NUM); 
A_aug = zeros(OUT_NUM*q+IN_NUM*(qu-1),OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM);
B_aug = zeros(OUT_NUM*q+IN_NUM*(qu-1),IN_NUM,STEP_NUM);
delta_z = zeros(OUT_NUM*(STEP_NUM+1),TRIAL_NUM);
delta_u = zeros(IN_NUM*(STEP_NUM+1),TRIAL_NUM);
ids = floor(linspace(1,STEP_NUM,STEP_DIV+1)); ID_START = ids(1:end-1); ID_END = [ID_START(2:end)+SKIP,STEP_NUM];

%% prestress & initial guess
fid = fopen('length0.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_norm = reshape(U, IN_NUM, STEP_NUM);
u_init = u_norm;
umax = max(max(abs(u_norm)));

%% forward pass
alpha = 0.5; z = 1;
mexstep('load',MODEL);
mexstep('reset');
mexstep('forward');
N_array=mexstep('get','site_xpos');
Nd_array=mexstep('get','sensordata');
% N=N_array(:,1:end-1);
% save('.\Tensegrity Files\N.mat','N');
x_new(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM); % the site_xpos data read from MuJoCo is 3 by NODE_NUM
x_new(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
Z_NORM=repmat(Ck*x_new(:,1),1,STEP_NUM+1);
for ite=1:1:ITE_NUM
% nominal
forward_flag = true; ti = 0;%alpha = .3; 
while forward_flag
    mexstep('reset');
    mexstep('forward');
    cost_new = 0; u_new = u_norm; %delta_j = 0; 
    for i = 1 : 1 : STEP_NUM
        if i >= SKIPo+1%max(q,qu)+2
            % no control limit
%             du_seq(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1)=-K(:,:,i)*[dz_seq(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),1);du_seq(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+1+qu),1)] + alpha*kt(:,i);
%             u_new(:,i) = u_norm(:,i) + du_seq(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1);
            % control limit
            N_array=mexstep('get','site_xpos');
            C_s = tenseg_ind2C(C_s_in,N_array(:,1:end-1));
            S = N_array(:,1:end-1)*C_s';
            len_str = diag(sqrt(S'*S));
            du_seq(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1)=max(lb-u_norm(:,i),min(len_str-u_norm(:,i),-K(:,:,i)*[dz_seq(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),1);du_seq(IN_NUM*(STEP_NUM-i+2)+1:IN_NUM*(STEP_NUM-i+1+qu),1)] + alpha*(kt(:,i)+dufc(:,i)-uf(:,i))));  
            u_new(:,i) = u_norm(:,i) + du_seq(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),1);
            cost_new = cost_new + 0.5*[reshape(Ck*(x_new(:,i:-1:i-q+1)-repmat(X_TARGET,1,q)),OUT_NUM*q,1);reshape(u_new(:,i-1:-1:i-qu+1)-u_init(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)]'*Qi*[reshape(Ck*(x_new(:,i:-1:i-q+1)-repmat(X_TARGET,1,q)),OUT_NUM*q,1);reshape(u_new(:,i-1:-1:i-qu+1)-u_init(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)]+0.5*(u_new(:,i)-u_init(:,i))'*Ri*(u_new(:,i)-u_init(:,i));
        end        
        mexstep('set','tendon_lengthspring',u_new(:,i),IN_NUM);
        mexstep('step',1);
        N_array=mexstep('get','site_xpos');
        Nd_array=mexstep('get','sensordata');
        xt(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
        xt(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
        dz_seq(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),1)=Ck*xt-Z_NORM(:,i+1);
        x_new(:,i+1) = xt;
    end
    cost_new = cost_new + 0.5*[reshape(Ck*(x_new(:,STEP_NUM+1:-1:STEP_NUM+2-q)-repmat(X_TARGET,1,q)),OUT_NUM*q,1);reshape(u_new(:,STEP_NUM:-1:STEP_NUM+2-qu)-u_init(:,STEP_NUM:-1:STEP_NUM+2-qu),IN_NUM*(qu-1),1)]'*Qf*[reshape(Ck*(x_new(:,STEP_NUM+1:-1:STEP_NUM+2-q)-repmat(X_TARGET,1,q)),OUT_NUM*q,1);reshape(u_new(:,STEP_NUM:-1:STEP_NUM+2-qu)-u_init(:,STEP_NUM:-1:STEP_NUM+2-qu),IN_NUM*(qu-1),1)];
    if ite > 1
        z = (cost(ite-1) - cost_new)/delta_j;
    end
    
    if z >= -0.6 || alpha < 10^-5
        forward_flag = false;
        cost(ite) = cost_new
        Z_NORM = Ck*x_new;
        X_NORM = x_new;
        u_norm = u_new;
        vk(:,STEP_NUM+1) = Qf*[reshape(Z_NORM(:,STEP_NUM+1:-1:STEP_NUM+2-q)-Ck*repmat(X_TARGET,1,q),OUT_NUM*q,1);reshape(u_norm(:,STEP_NUM:-1:STEP_NUM+2-qu)-u_init(:,STEP_NUM:-1:STEP_NUM+2-qu),IN_NUM*(qu-1),1)];
    else
        alpha = 0.99*alpha;
    end
    ti = ti + 1;
end

% alpha = alpha * 0.92;
% if alpha < 0.02
%     alpha = 0.02;
% end
alpha
umax = max(1,max(max(abs(u_norm))))

% sysid - arma
% collect data
% batch
% delta_u=ID_PERT_COEF*1*randn(IN_NUM*(STEP_NUM+1),TRIAL_NUM);
skipu = 0;
delta_u=[ID_PERT_COEF*1*randn(IN_NUM*(STEP_NUM+1-skipu),TRIAL_NUM);0.01*randn(IN_NUM*skipu,TRIAL_NUM)];
for k=1:1:STEP_DIV
    for j=1:1:TRIAL_NUM
        mexstep('reset');
        mexstep('forward');
        for i=ID_START(k):1:ID_END(k)
            % control limit
            delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)=max(lb-u_norm(:,i),min(ub-u_norm(:,i),delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j)));
            mexstep('set','tendon_lengthspring',u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
            mexstep('step',1);
            N_array=mexstep('get','site_xpos');
            Nd_array=mexstep('get','sensordata');
            x2(1:POS_NUM,1)=reshape(N_array(:,1:NODE_NUM),1,3*NODE_NUM);
            x2(POS_NUM+1:SYS_NUM,1)=Nd_array(1:3*NODE_NUM);
            delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Z_NORM(:,i+1);
        end
    end 

    % arma fitting - least square: M1 * fitcoef = delta_y
    for i=ID_START(k)+SKIPo:1:ID_END(k) % skip the first few steps to wait for enough data
        M1=[delta_z(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
        [Q,R]=qr(M1');
        fitcoef(:,:,i)=(R\Q'*delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)')';
%         fitcoef(:,:,i)=delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)*M1'/(M1*M1');
        r(i)=sqrt(mean(mean((delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)-fitcoef(:,:,i)*M1).^2,1))); % residual
    end
end

% single
% for j=1:1:TRIAL_NUM
%     mexstep('reset');
%     mexstep('set','qpos',X_INIT(1:POS_NUM,1),POS_NUM);
%     mexstep('set','qvel',X_INIT(POS_NUM+1:SYS_NUM,1),VEL_NUM);
%     mexstep('forward');
%     for i=1:1:STEP_NUM
%         mexstep('set','ctrl',u_norm(:,i)+delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+2),j),IN_NUM);
%         mexstep('step',1);
%         x2(1:POS_NUM,1)=mexstep('get','qpos')';
%         x2(POS_NUM+1:SYS_NUM,1)=mexstep('get','qvel')';
%         delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),j)=Ck*x2-Z_NORM(:,i+1);
%     end
% end 
% 
% % arma fitting - least square: M1 * fitcoef = delta_y
% for i=SKIPo+1:1:STEP_NUM % skip the first few steps to wait for enough data
%     M1=[delta_z(OUT_NUM*(STEP_NUM-i+1)+1:OUT_NUM*(STEP_NUM-i+1+q),:);delta_u(IN_NUM*(STEP_NUM-i+1)+1:IN_NUM*(STEP_NUM-i+qu+1),:)];
%     fitcoef(:,:,i)=delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)*M1'/(M1*M1');
%     r(i)=sqrt(mean(mean((delta_z(OUT_NUM*(STEP_NUM-i)+1:OUT_NUM*(STEP_NUM-i+1),:)-fitcoef(:,:,i)*M1).^2,1))); % residual
% end
% rmax = max(r)

% construct augmented Ak, Bk
for i=SKIPo+1:1:STEP_NUM 
    A_aug(:,:,i)=[fitcoef(:,1:OUT_NUM*q,i),fitcoef(:,OUT_NUM*q+IN_NUM+1:end,i);
      eye((q-1)*OUT_NUM),zeros((q-1)*OUT_NUM,OUT_NUM+IN_NUM*(qu-1));
      zeros(IN_NUM*(qu-1),OUT_NUM*q),[zeros(IN_NUM,IN_NUM*(qu-1));eye(IN_NUM*(qu-2)),zeros(IN_NUM*(qu-2),IN_NUM)]];
    B_aug(:,:,i)=[fitcoef(:,OUT_NUM*q+1:OUT_NUM*q+IN_NUM,i);zeros(OUT_NUM*(q-1),IN_NUM);eye(IN_NUM*min(qu-1,1));zeros(IN_NUM*(qu-2),IN_NUM)];
end

%% backpass
delta_j = 0; uf = zeros(IN_NUM,STEP_NUM); kt  = zeros(IN_NUM,STEP_NUM); K = zeros(IN_NUM,OUT_NUM*q+IN_NUM*(qu-1),STEP_NUM); dufc = zeros(IN_NUM,STEP_NUM);
for i = STEP_NUM:-1:SKIPo+1%max(q,qu)+2
    Quu(:,:,i) = B_aug(:,:,i)'*Sk(:,:,i+1)*B_aug(:,:,i)+Ri; miu = 0; delta = 0;
    if min(eig(Quu(:,:,i))) <= 0
        disp('Quu is not positive definite')
    end
%     while min(eig(Quu(:,:,i))) <= 0
%         disp('Quu is not positive definite')
%         delta = max(2, 2*delta);
%         miu = min(max(10^-3, miu*delta),10^8);
%         Quu(:,:,i) = B_aug(:,:,i)'*(Sk(:,:,i+1)+miu*eye(OUT_NUM*q+IN_NUM*(qu-1)))*B_aug(:,:,i)+Ri;
%     end

    kpreinv = inv(Quu(:,:,i));
    Qux=B_aug(:,:,i)'*Sk(:,:,i+1)*A_aug(:,:,i);
    K(:,:,i) = kpreinv*Qux;
    Kv(:,:,i) = kpreinv*B_aug(:,:,i)';
    Ku(:,:,i) = kpreinv*Ri;
    Sk(:,:,i) = A_aug(:,:,i)'*Sk(:,:,i+1)*(A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))+Qi;
    vk(:,i) = (A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))'*vk(:,i+1)-K(:,:,i)'*Ri*(u_norm(:,i)-u_init(:,i))+Qi*[reshape(Z_NORM(:,i:-1:i-q+1)-Ck*repmat(X_TARGET,1,q),OUT_NUM*q,1);reshape(u_norm(:,i-1:-1:i-qu+1)-u_init(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)];
%     kt(:,i) = - Kv(:,:,i)*vk(:,i+1) - Ku(:,:,i)*u_norm(:,i); 
    Qu = Ri*(u_norm(:,i)-u_init(:,i))+B_aug(:,:,i)'*vk(:,i+1); kt(:,i) = -kpreinv*Qu;
    delta_j = delta_j - (alpha*kt(:,i)'*Qu+alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));

% % with IROS12 control limit
%     fidx=[]; cidx=[];
%     for j=1:1:IN_NUM
%         if u_norm(j,i) < ub && u_norm(j,i) > lb
%             fidx=[fidx,j];
%         else
%             cidx=[cidx,j];
%         end
%     end
%     uf(fidx,i) = u_norm(fidx,i);
%     kpreinv = inv(Quu(fidx,fidx,i));
%     Qux=B_aug(:,:,i)'*Sk(:,:,i+1)*A_aug(:,:,i);
%     K(fidx,:,i) = kpreinv*Qux(fidx,:);
%     Sk(:,:,i) = A_aug(:,:,i)'*Sk(:,:,i+1)*(A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))+Qi;
%     vk(:,i) = (A_aug(:,:,i)-B_aug(:,:,i)*K(:,:,i))'*vk(:,i+1)-K(:,:,i)'*Ri*(u_norm(:,i)-u_init(:,i))+Qi*[reshape(Z_NORM(:,i:-1:i-q+1)-Ck*repmat(X_TARGET,1,q),OUT_NUM*q,1);reshape(u_norm(:,i-1:-1:i-qu+1)-u_init(:,i-1:-1:i-qu+1),IN_NUM*(qu-1),1)];
%     dufc(fidx,i) = -kpreinv*Quu(fidx,cidx,i)*u_norm(cidx,i);
%     Qu = Ri*(u_norm(:,i)-u_init(:,i))+B_aug(:,:,i)'*vk(:,i+1); kt(fidx,i) = -kpreinv*Qu(fidx,1);
%     delta_j = delta_j - (alpha*kt(:,i)'*Qu+alpha^2/2*kt(:,i)'*Quu(:,:,i)*kt(:,i));
end
end
mexstep('exit');

%% output result
fid = fopen('result0.txt','wt');
for i = 1 : STEP_NUM
    for j = 1 : IN_NUM
        fprintf(fid,'%.10f ',u_norm(j,i));
    end
end
fclose(fid);

save('results.mat','cost','u_norm');

%% plot
figure;
% plot([X_NORM(4,:)', X_NORM(5,:)', X_NORM(6,:)'])
plot([X_NORM(13,:)', X_NORM(14,:)', X_NORM(15,:)'])
% plot([X_NORM(49,:)', X_NORM(50,:)', X_NORM(51,:)'])
legend('x','y','z')
xlabel('step')

figure;
plot(0:1:size(cost)-1, cost);
xlabel('iteration')
ylabel('cost')

% figure;
% plot(u_norm)
% legend('u')
% xlabel('step')