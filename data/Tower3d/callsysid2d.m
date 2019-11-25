clc;clear all; close all;
%% system parameters
ExeFileName='sysid2d.exe';
ModelFileName='t1d1_3d.xml';
NoiseLevel='0.001';
IterationNum='100';
ThreadNum='4';
StepNum=300;
PreStressVec=[
260.7235543
260.7235543
260.7235543
260.7235543
260.7235543
260.7235543
260.7343464
260.7343464
260.7343464
260.7235543
260.7235543
260.7235543
782.6167724
782.6167724
782.6167724
260.7235543
782.5877967
782.5877967
782.5877967
260.7235543
];
PreStressMat=repmat(-PreStressVec',1,StepNum);
%% save nominal control to .txt file
fidtk = fopen('result0.txt','wt');
fprintf(fidtk,'%f ',PreStressMat);
fprintf(fidtk,'\n');
fclose(fidtk);
%% call sysid2d.exe
ExeFilePath=fullfile('.\',ExeFileName);
Param1=[' ',ModelFileName];%第一个参数，一定要有' '
Param2=[' ',NoiseLevel];
Param3=[' ',IterationNum];
Param4=[' ',ThreadNum];
Cmd=[ExeFilePath ,Param1 ,Param2 ,Param3,Param4];
system(Cmd);