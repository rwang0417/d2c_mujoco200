%%
% x2 is the state after 1 step
% x1 is the initial state
% u_init is the control value NUM_IN * TIME
% t_step is the timestep in seconds
% mex mexstep.c mujoco200.lib mujoco200nogl.lib
% clear all; clc;
tic;
%% Dbar
% SYS_NUM = 4;
% IN_NUM = 4;
% STEP_NUM = 200;
% NODE_NUM = 6;
% SIM_STEP = 0.01;
% CTRL_STEP = 0.01;
% MODEL = 'dbar.xml';
% Nfinal_vec = [0.0000
% 0.0000
% 0.0000
% -0.9272
% 0.0000
% 1.0679
% -0.4486
% 0.0000
% 2.3987
% 0.4786
% 0.0000
% 1.3308
% -2.0000
% 0.0000
% 0.0000
% 2.0000
% 0.0000
% 0.0000];
% % N_final = [0 -sqrt(2)*cosd(60) 0 sqrt(2)*cosd(60) -2 2;
% %            0 0 0 0 0 0;
% %            0 sqrt(2)*sind(60) 2*sqrt(2)*sind(60) sqrt(2)*sind(60) 0 0];
% N = [0 -1 0 1 -2 2;
%            0 0 0 0 0 0;
%            0 1 2 1 0 0];
% C_b_com = [-1 1 0 0 0 0; 0 -1 1 0 0 0;0 0 -1 1 0 0;1 0 0 -1 0 0];
% C_s_com = [-1 0 1 0 0 0;0 -1 0 1 0 0;0 0 0 -1 0 1;0 -1 0 0 1 0];
%% T1D1
% SYS_NUM = 12;
% IN_NUM = 10;
% STEP_NUM = 300;
% NODE_NUM = 11;
% SIM_STEP = 0.01;
% CTRL_STEP = 0.01;
% MODEL = 't1d1.xml';
% Nfinal_vec = [0.0000
% 0.0000
% 0.0000
% -0.8173
% 0.0000
% 0.5763
% 0.0111
% 0.0000
% 1.1364
% -1.4879
% 0.0000
% 1.1911
% -0.7550
% 0.0000
% 1.7791
% -0.2614
% 0.0000
% 2.6487
% 1.4888
% 0.0000
% 0.8784
% 0.8284
% 0.0000
% 0.5601
% 0.5047
% 0.0000
% 2.0061
% -3.0000
% 0.0000
% 0.0000
% 3.0000
% 0.0000
% 0.0000];
% N_vec=[0.0000
% 0.0000
% 0.0000
% -0.8000
% 0.0000
% 0.6000
% 0.0000
% 0.0000
% 1.2000
% -1.5000
% 0.0000
% 1.2000
% -0.8000
% 0.0000
% 1.8000
% 0.0000
% 0.0000
% 2.4000
% 1.5000
% 0.0000
% 1.2000
% 0.8000
% 0.0000
% 0.6000
% 0.8000
% 0.0000
% 1.8000
% -3.0000
% 0.0000
% 0.0000
% 3.0000
% 0.0000
% 0.0000];
% C_b_com = zeros(IN_NUM,NODE_NUM);
% C_b_com(1,1) = -1; C_b_com(1,2) = 1;
% C_b_com(2,2) = -1; C_b_com(2,3) = 1;
% C_b_com(3,3) = -1; C_b_com(3,4) = 1;
% C_b_com(4,3) = -1; C_b_com(4,5) = 1;
% C_b_com(5,5) = -1; C_b_com(5,6) = 1;
% C_b_com(6,3) = -1; C_b_com(6,7) = 1;
% C_b_com(7,1) = -1; C_b_com(7,8) = 1;
% C_b_com(8,8) = -1; C_b_com(8,3) = 1;
% C_b_com(9,3) = -1; C_b_com(9,9) = 1;
% C_b_com(10,9) = -1; C_b_com(10,6) = 1;
%% T2D1
SYS_NUM = 28;
IN_NUM = 22;
STEP_NUM = 400;
NODE_NUM = 21;
SIM_STEP = 0.01;
CTRL_STEP = 0.01;
MODEL = 't2d1.xml';
N_vec=[0.0000
0.0000
0.0000
-0.8000
0.0000
0.6000
0.0000
0.0000
1.2000
-1.0000
0.0000
1.2000
-0.8000
0.0000
1.8000
0.0000
0.0000
2.4000
-2.0000
0.0000
2.4000
-0.8000
0.0000
3.0000
0.0000
0.0000
3.6000
-1.0000
0.0000
3.6000
-0.8000
0.0000
4.2000
0.0000
0.0000
4.8000
1.0000
0.0000
3.6000
2.0000
0.0000
2.4000
1.0000
0.0000
1.2000
0.8000
0.0000
0.6000
0.8000
0.0000
1.8000
0.8000
0.0000
3.0000
0.8000
0.0000
4.2000
-4.0000
0.0000
0.0000
4.0000
0.0000
0.0000];
Nfinal_vec=[0.0000
0.0000
0.0000
-0.8051
0.0000
0.5931
0.0025
0.0000
1.1829
-0.9975
0.0000
1.1869
-0.7726
0.0000
1.8148
0.0567
0.0000
2.3736
-1.9295
0.0000
2.6080
-0.8170
0.0000
2.8601
0.0599
0.0000
3.3408
-0.9371
0.0000
3.4183
-0.5617
0.0000
4.1241
-0.3069
0.0000
5.0911
1.0551
0.0000
3.2432
1.9625
0.0000
1.7671
1.0013
0.0000
1.2323
0.8076
0.0000
0.5898
0.8318
0.0000
1.7417
0.9336
0.0000
2.8543
0.3147
0.0000
4.3078
-4.0000
0.0000
0.0000
4.0000
0.0000
0.0000];
C_b_com = zeros(22,21);
C_b_com(1,1) = -1; C_b_com(1,2) = 1;
C_b_com(2,2) = -1; C_b_com(2,3) = 1;
C_b_com(3,3) = -1; C_b_com(3,4) = 1;
C_b_com(4,3) = -1; C_b_com(4,5) = 1;
C_b_com(5,5) = -1; C_b_com(5,6) = 1;
C_b_com(6,6) = -1; C_b_com(6,7) = 1;
C_b_com(7,6) = -1; C_b_com(7,8) = 1;
C_b_com(8,8) = -1; C_b_com(8,9) = 1;
C_b_com(9,9) = -1; C_b_com(9,10) = 1;
C_b_com(10,9) = -1; C_b_com(10,11) = 1;
C_b_com(11,11) = -1; C_b_com(11,12) = 1;
C_b_com(12,9) = -1; C_b_com(12,13) = 1;
C_b_com(13,6) = -1; C_b_com(13,14) = 1;
C_b_com(14,3) = -1; C_b_com(14,15) = 1;
C_b_com(15,1) = -1; C_b_com(15,16) = 1;
C_b_com(16,16) = -1; C_b_com(16,3) = 1;
C_b_com(17,3) = -1; C_b_com(17,17) = 1;
C_b_com(18,17) = -1; C_b_com(18,6) = 1;
C_b_com(19,6) = -1; C_b_com(19,18) = 1;
C_b_com(20,18) = -1; C_b_com(20,9) = 1;
C_b_com(21,9) = -1; C_b_com(21,19) = 1;
C_b_com(22,19) = -1; C_b_com(22,12) = 1;

%% independent variables
N = reshape(N_vec,3,NODE_NUM);
Nd = zeros(size(N));
gamma_hat = zeros(IN_NUM,IN_NUM);
N_final = reshape(Nfinal_vec,3,NODE_NUM);
x1 = zeros(1,2*SYS_NUM);
u = zeros(IN_NUM,STEP_NUM);
integration_per_step = CTRL_STEP/SIM_STEP;
%% read control sequence
fid = fopen('resulta.txt','r');
U = fscanf(fid, '%f');
fclose(fid);
u_opt = reshape(U, IN_NUM, STEP_NUM);
% load('control','u');
%% main loop
mexstep('load',MODEL);
mexstep('set','qpos',x1(1,1:SYS_NUM),SYS_NUM);
mexstep('set','qvel',x1(1,SYS_NUM+1:2*SYS_NUM),SYS_NUM);
for i = 1 : 1 : STEP_NUM
%     [u(:,i),MyControlGamma] = Analytical_Control(N,Nd,gamma_hat,N_final);
    mexstep('set','ctrl',u_opt(:,i),IN_NUM);
    mexstep('step',integration_per_step);
    mexstep('forward');
    N_array=mexstep('get','site_xpos');
    Nd_array=mexstep('get','sensordata');
	N = N_array(:,1:NODE_NUM);
    Nd = reshape(Nd_array,3,NODE_NUM);
    gamma_hat = diag(MyControlGamma);
    Nhist(:,:,i)= N;
end
mexstep('exit');
toc;
% save('control','u');
%% save control sequence
% fidu = fopen('resulta.txt','wt');
% for i = 1 : STEP_NUM
%     for j = 1 : IN_NUM
%         fprintf(fidu,'%.15f ',-u(j,i));
%     end
% end
% fprintf(fidu,'\n');
% fclose(fidu);
%% video
% vid = VideoWriter('video1');
% vid.Quality = 100;
% open(vid);
% fig = figure('units','normalized','position',[0 0 0.8 0.8]);
% % fig = tenseg_plot(N,C_b,C_s,fig);%,highlight_nodes,view_vec,[],R3Ddata);
% for i=1:5:size(Nhist,3)
% 	N = Nhist(:,:,i);
% 	fig = tenseg_plot(N,C_b_com,[],fig,[],[],[],[]);
% 	axis equal
% % 	axis(axis_vec)
% 	
% 	% Get frame width/height and write frame (including axes) to file
% 	%position = get(gcf,'Position'); % do this to include axes in video
% 	%writeVideo(vid,getframe(gcf,[0 0 position(3:4)]));
% 
% 	drawnow
% 	SelectEntireFig = getframe(fig);
% 	writeVideo(vid,SelectEntireFig);    
% 	clf
% end
% close(vid);