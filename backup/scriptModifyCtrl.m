%% trim control sequence
% IN_NUM=2;
% DISIRED_STEP_NUM=600;
% fid = fopen('result.txt','r');
% u = fscanf(fid, '%f');
% fclose(fid);
% u_cut=u(1:IN_NUM*DISIRED_STEP_NUM);%step_num*actuator_num
% fid = fopen('result0.txt','wt');
% fprintf(fid,'%f ',u_cut);
% fclose(fid);

%% control interpolation
ORIG_TIMESTEP=0.01;
NEW_TIMESTEP=0.005;
IN_NUM=14;
multi=ORIG_TIMESTEP/NEW_TIMESTEP;
if NEW_TIMESTEP>ORIG_TIMESTEP
    fprintf("new timestep is larger than original timestep...");
end
fid = fopen('result0.txt','r');
u = fscanf(fid, '%f');
fclose(fid);
row=floor(size(u,1)/IN_NUM);
u_reshape=reshape(u,IN_NUM,row);
u_extend=zeros(IN_NUM,multi*row);
for i=1:1:row
    u_extend(:,(i-1)*multi+1:i*multi)=repmat(u_reshape(:,i),1,multi);
end
u_new=reshape(u_extend,1,IN_NUM*multi*row);
fid = fopen('result0_s15.txt','wt');
fprintf(fid,'%.12g ',u_new);
fclose(fid);