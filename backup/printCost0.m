load('results.mat');
ite = size(cost, 1);
fid = fopen('cost0.txt','wt');
for i = 1 : 1 : ite
    fprintf(fid,'%.4f ',cost(i));
end
fprintf(fid,'\n');
fclose(fid);