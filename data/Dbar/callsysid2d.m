clc;clear all; close all;
%% system parameters
ExeFileName='sysid2d.exe';
ModelFileName='dbarload.xml';
NoiseLevel='0.001';
IterationNum='100';
ThreadNum='4';
StepNum=1000;
PreStressVec=[
4.14632533186679
173.344035351719
4.14632533186679
4.14632533186679
];
PreStressMat=repmat(-PreStressVec',1,StepNum);
%% save nominal control to .txt file
fidtk = fopen('result0.txt','wt');
fprintf(fidtk,'%f ',PreStressMat);
fprintf(fidtk,'\n');
fclose(fidtk);
%% call sysid2d.exe
% ExeFilePath=fullfile('.\',ExeFileName);
% Param1=[' ',ModelFileName];%第一个参数，一定要有' '
% Param2=[' ',NoiseLevel];
% Param3=[' ',IterationNum];
% Param4=[' ',ThreadNum];
% Cmd=[ExeFilePath ,Param1 ,Param2 ,Param3,Param4];
% system(Cmd);