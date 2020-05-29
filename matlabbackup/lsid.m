clear all;warning off;tic;
%% tower
SYS_NUM = 28;
POS_NUM = 14;
VEL_NUM = 14;
ACTUAL_DOF=24;
IN_NUM = 20;
STEP_NUM=5;
SIM_STEP = 0.001;
CTRL_STEP = 0.001;
TRIAL_NUM=2000;
MODEL = 'tower.xml';
%% read control sequence
fid = fopen('result0.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
U_cut=U(1:IN_NUM*STEP_NUM,1);
u_norm = reshape(U_cut, IN_NUM, STEP_NUM);
u_max = max(max(abs(u_norm)));
u_norm=zeros(IN_NUM, STEP_NUM);
%% equilibrium point id
PERT_STD=.0000001;
delta_x1=0.00000*u_max*randn(SYS_NUM,TRIAL_NUM);
% delta_x1(POS_NUM+1:SYS_NUM,:)=0.00000001*delta_x1(POS_NUM+1:SYS_NUM,:);
delta_u=0*u_max*randn(IN_NUM,TRIAL_NUM);
X_NORM_INIT = repmat(zeros(ACTUAL_DOF*2,1),1,TRIAL_NUM);
x_pert_init=zeros(ACTUAL_DOF*2,TRIAL_NUM);
x2_orig = zeros(ACTUAL_DOF*2,TRIAL_NUM);
delta_x2 = zeros(SYS_NUM,TRIAL_NUM);
% plus
% x_pert_init(1:POS_NUM,:)=X_NORM_INIT(1:POS_NUM,:)+delta_x1(1:POS_NUM,:);
% x_pert_init(ACTUAL_DOF+1:ACTUAL_DOF+VEL_NUM,:)=X_NORM_INIT(ACTUAL_DOF+1:ACTUAL_DOF+VEL_NUM,:)+delta_x1(POS_NUM+1:SYS_NUM,:);
% tower
% pos
x_pert_init(1:2,:)=PERT_STD*u_max*randn(2,TRIAL_NUM);
x_pert_init(3,:)=-2*x_pert_init(2,:);
x_pert_init(4:7,:)=PERT_STD*u_max*randn(4,TRIAL_NUM);
x_pert_init(8,:)=-2*(x_pert_init(7,:)-x_pert_init(2,:));
x_pert_init(9:13,:)=PERT_STD*u_max*randn(5,TRIAL_NUM);
x_pert_init(14:15,:)=-x_pert_init(2:3,:);
x_pert_init(16,:)=PERT_STD*u_max*randn(1,TRIAL_NUM);
x_pert_init(17:18,:)=-x_pert_init(7:8,:);
x_pert_init(19,:)=PERT_STD*u_max*randn(1,TRIAL_NUM);
x_pert_init(20:21,:)=-x_pert_init(2:3,:);
x_pert_init(22,:)=PERT_STD*u_max*randn(1,TRIAL_NUM);
x_pert_init(23:24,:)=-x_pert_init(7:8,:);
% vel
x_pert_init(25:26,:)=PERT_STD*u_max*randn(2,TRIAL_NUM);
x_pert_init(27,:)=-2*x_pert_init(26,:);
x_pert_init(28:31,:)=PERT_STD*u_max*randn(4,TRIAL_NUM);
x_pert_init(32,:)=-2*(x_pert_init(31,:)-x_pert_init(26,:));
x_pert_init(33:37,:)=PERT_STD*u_max*randn(5,TRIAL_NUM);
x_pert_init(38:39,:)=-x_pert_init(26:27,:);
x_pert_init(40,:)=PERT_STD*u_max*randn(1,TRIAL_NUM);
x_pert_init(41:42,:)=-x_pert_init(31:32,:);
x_pert_init(43,:)=PERT_STD*u_max*randn(1,TRIAL_NUM);
x_pert_init(44:45,:)=-x_pert_init(26:27,:);
x_pert_init(46,:)=PERT_STD*u_max*randn(1,TRIAL_NUM);
x_pert_init(47:48,:)=-x_pert_init(31:32,:);
delta_x1(1:2,:)=x_pert_init(1:2,:);
delta_x1(3:6,:)=x_pert_init(4:7,:);
delta_x1(7:11,:)=x_pert_init(9:13,:);
delta_x1(12,:)=x_pert_init(16,:);
delta_x1(13,:)=x_pert_init(19,:);
delta_x1(14,:)=x_pert_init(22,:);
delta_x1(15:16,:)=x_pert_init(25:26,:);
delta_x1(17:20,:)=x_pert_init(28:31,:);
delta_x1(21:25,:)=x_pert_init(33:37,:);
delta_x1(26,:)=x_pert_init(40,:);
delta_x1(27,:)=x_pert_init(43,:);
delta_x1(28,:)=x_pert_init(46,:);
% x_pert_init(19,:)=X_NORM_INIT(19,:)+delta_x1(14,:);
% x_pert_init(22,:)=X_NORM_INIT(22,:)+delta_x1(15,:);
% x_pert_init(43,:)=X_NORM_INIT(43,:)+delta_x1(30,:);
% x_pert_init(46,:)=X_NORM_INIT(46,:)+delta_x1(31,:);
% % pos
% x_pert_init(14,:)=-x_pert_init(2,:);
% x_pert_init(20,:)=-x_pert_init(2,:);
% x_pert_init(15,:)=-x_pert_init(3,:);
% x_pert_init(21,:)=-x_pert_init(3,:);
% x_pert_init(17,:)=-x_pert_init(7,:);
% x_pert_init(23,:)=-x_pert_init(7,:);
% x_pert_init(18,:)=-x_pert_init(8,:);
% x_pert_init(24,:)=-x_pert_init(8,:);
% % vel
% x_pert_init(38,:)=-x_pert_init(26,:);
% x_pert_init(44,:)=-x_pert_init(26,:);
% x_pert_init(39,:)=-x_pert_init(27,:);
% x_pert_init(45,:)=-x_pert_init(27,:);
% x_pert_init(41,:)=-x_pert_init(31,:);
% x_pert_init(47,:)=-x_pert_init(31,:);
% x_pert_init(42,:)=-x_pert_init(32,:);
% x_pert_init(48,:)=-x_pert_init(32,:);
% collect data
mexstep('load',MODEL); % load model
mexstep('reset');
for i=1:1:TRIAL_NUM
    mexstep('set','qpos',x_pert_init(1:ACTUAL_DOF,i),ACTUAL_DOF);
    mexstep('set','qvel',x_pert_init(ACTUAL_DOF+1:ACTUAL_DOF*2,i),ACTUAL_DOF);
    mexstep('forward');
    mexstep('set','ctrl',u_norm(:,1)+delta_u(:,i),IN_NUM);
    mexstep('step',1);
    x2_orig(1:ACTUAL_DOF,i)=mexstep('get','qpos')';
    x2_orig(ACTUAL_DOF+1:ACTUAL_DOF*2,i)=mexstep('get','qvel')';
end
delta_x2(1:POS_NUM,:)=x2_orig(1:POS_NUM,:);
delta_x2(POS_NUM+1:SYS_NUM,:)=x2_orig(ACTUAL_DOF+1:ACTUAL_DOF+VEL_NUM,:);
% tower
delta_x2(1:2,:)=x2_orig(1:2,:);
delta_x2(3:6,:)=x2_orig(4:7,:);
delta_x2(7:11,:)=x2_orig(9:13,:);
delta_x2(12,:)=x2_orig(16,:);
delta_x2(13,:)=x2_orig(19,:);
delta_x2(14,:)=x2_orig(22,:);
delta_x2(15:16,:)=x2_orig(25:26,:);
delta_x2(17:20,:)=x2_orig(28:31,:);
delta_x2(21:25,:)=x2_orig(33:37,:);
delta_x2(26,:)=x2_orig(40,:);
delta_x2(27,:)=x2_orig(43,:);
delta_x2(28,:)=x2_orig(46,:);
% delta_x2(14,:)=x2_orig(19,:);
% delta_x2(15,:)=x2_orig(22,:);
% delta_x2(30,:)=x2_orig(43,:);
% delta_x2(31,:)=x2_orig(46,:);
% minus
% x_pert_init(1:POS_NUM,:)=X_NORM_INIT(1:POS_NUM,:)-delta_x1(1:POS_NUM,:);
% x_pert_init(ACTUAL_DOF+1:ACTUAL_DOF+VEL_NUM,:)=X_NORM_INIT(ACTUAL_DOF+1:ACTUAL_DOF+VEL_NUM,:)-delta_x1(POS_NUM+1:SYS_NUM,:);
% x_pert_init=0.001*u_max*randn(2*ACTUAL_DOF,TRIAL_NUM);
% tower
% x_pert_init(19,:)=X_NORM_INIT(19,:)-delta_x1(14,:);
% x_pert_init(22,:)=X_NORM_INIT(22,:)-delta_x1(15,:);
% x_pert_init(43,:)=X_NORM_INIT(43,:)-delta_x1(30,:);
% x_pert_init(46,:)=X_NORM_INIT(46,:)-delta_x1(31,:);
% % pos
% x_pert_init(14,:)=-x_pert_init(2,:);
% x_pert_init(20,:)=-x_pert_init(2,:);
% x_pert_init(15,:)=-x_pert_init(3,:);
% x_pert_init(21,:)=-x_pert_init(3,:);
% x_pert_init(17,:)=-x_pert_init(7,:);
% x_pert_init(23,:)=-x_pert_init(7,:);
% x_pert_init(18,:)=-x_pert_init(8,:);
% x_pert_init(24,:)=-x_pert_init(8,:);
% % vel
% x_pert_init(38,:)=-x_pert_init(26,:);
% x_pert_init(44,:)=-x_pert_init(26,:);
% x_pert_init(39,:)=-x_pert_init(27,:);
% x_pert_init(45,:)=-x_pert_init(27,:);
% x_pert_init(41,:)=-x_pert_init(31,:);
% x_pert_init(47,:)=-x_pert_init(31,:);
% x_pert_init(42,:)=-x_pert_init(32,:);
% x_pert_init(48,:)=-x_pert_init(32,:);
for i=1:1:TRIAL_NUM
    mexstep('set','qpos',-x_pert_init(1:ACTUAL_DOF,i),ACTUAL_DOF,1);
    mexstep('set','qvel',-x_pert_init(ACTUAL_DOF+1:ACTUAL_DOF*2,i),ACTUAL_DOF,1);
    mexstep('forward');
%     x2_orig(1:ACTUAL_DOF,i)=mexstep('get','qpos')';
%     x2_orig(ACTUAL_DOF+1:ACTUAL_DOF*2,i)=mexstep('get','qvel')';
    mexstep('set','ctrl',u_norm(:,1)-delta_u(:,i),IN_NUM);
    mexstep('step',1);
    y=mexstep('get','site_xpos');
    x2_orig(1:ACTUAL_DOF,i)=mexstep('get','qpos')';
    x2_orig(ACTUAL_DOF+1:ACTUAL_DOF*2,i)=mexstep('get','qvel')';
end
mexstep('exit');
% x2_orig-x_pert_init
% delta_x2(1:POS_NUM,:)=delta_x2(1:POS_NUM,:)-x2_orig(1:POS_NUM,:);
% delta_x2(POS_NUM+1:SYS_NUM,:)=delta_x2(POS_NUM+1:SYS_NUM,:)-x2_orig(ACTUAL_DOF+1:ACTUAL_DOF+VEL_NUM,:);
% tower
delta_x2(1:2,:)=delta_x2(1:2,:)-x2_orig(1:2,:);
delta_x2(3:6,:)=delta_x2(3:6,:)-x2_orig(4:7,:);
delta_x2(7:11,:)=delta_x2(7:11,:)-x2_orig(9:13,:);
delta_x2(12,:)=delta_x2(12,:)-x2_orig(16,:);
delta_x2(13,:)=delta_x2(13,:)-x2_orig(19,:);
delta_x2(14,:)=delta_x2(14,:)-x2_orig(22,:);
delta_x2(15:16,:)=delta_x2(15:16,:)-x2_orig(25:26,:);
delta_x2(17:20,:)=delta_x2(17:20,:)-x2_orig(28:31,:);
delta_x2(21:25,:)=delta_x2(21:25,:)-x2_orig(33:37,:);
delta_x2(26,:)=delta_x2(26,:)-x2_orig(40,:);
delta_x2(27,:)=delta_x2(27,:)-x2_orig(43,:);
delta_x2(28,:)=delta_x2(28,:)-x2_orig(46,:);
% delta_x2(14,:)=delta_x2(14,:)+x2_orig(14,:)-x2_orig(19,:);
% delta_x2(15,:)=delta_x2(15,:)+x2_orig(15,:)-x2_orig(22,:);
% delta_x2(30,:)=delta_x2(30,:)+x2_orig(30,:)-x2_orig(43,:);
% delta_x2(31,:)=delta_x2(31,:)+x2_orig(31,:)-x2_orig(46,:);
% ls x1'*AB'=delta_x2'
% x1=[delta_x1;delta_u];
x1=delta_x1;
[Q,R]=qr(x1');
A=(R\Q'*delta_x2')'/2;
A1=delta_x2*x1'/(x1*x1')/2;
r=sqrt(mean(mean((delta_x2-2*A*x1).^2,1)))
fid = fopen('lnr_equ.txt','wt');
for i = 1:1:POS_NUM+VEL_NUM
    for j = 1:1:POS_NUM+VEL_NUM
        fprintf(fid,'%g ',A(i,j));
    end
    fprintf(fid,'\n');
end
fclose(fid);
%% nominal states with free states only
% X_NORM = zeros(ACTUAL_DOF*2,STEP_NUM+1);
% mexstep('load',MODEL); % load model
% mexstep('reset');
% mexstep('set','qpos',X_NORM(1:ACTUAL_DOF,1),ACTUAL_DOF);
% mexstep('set','qvel',X_NORM(ACTUAL_DOF+1:ACTUAL_DOF*2,1),ACTUAL_DOF);
% mexstep('forward');
% for i = 1 : 1 : STEP_NUM
%     mexstep('set','ctrl',u_norm(:,i),IN_NUM);
%     mexstep('step',1);
%     X_NORM(1:ACTUAL_DOF,i+1)=mexstep('get','qpos');
%     X_NORM(ACTUAL_DOF+1:ACTUAL_DOF*2,i+1)=mexstep('get','qvel');
% end
% mexstep('exit');
%% check sysid result

