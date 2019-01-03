%% 設定參數
clc;clear;close all;
sampling_rate = 0.002;
tacc=0.2;                                        
d = [   0,    0,    0,    0,   0,   0];
a= [0.12, 0.25, 0.26,    0,   0,   0];
alpha = [ -90,    0,    0,  -90,  90,   0];
lower_bound = [-150,  -30, -120, -110,-180,-180];
upper_bound = [150 ,  100,    0,  110, 180, 180];
Euler=zeros(2,3); %初始euler angle
A   =  [ [0 0 1 0.1] ; [-1 0 0 0.5] ; [0 -1 0 0.3] ; [0 0 0 1]  ] ;     %A點的NOAP
		     
B   =  [ [0 0 1 0.3] ; [0 1 0 0.3] ; [-1 0 0 0.2] ; [0 0 0 1]   ]  ;    %B點的NOAP
			 
C   =  [ [0 1 0 -0.3] ; [0 0 -1 -0.25] ; [-1 0 0 0.25] ; [0 0 0 1]];    %C點的NOAP
%% 執行project2			           
joint_move(sampling_rate,tacc,A,B,C,d,a,alpha,Euler,lower_bound,upper_bound)    %軸座標路徑規劃

cartesian_move(sampling_rate,tacc,A,B,C)                                        %卡式座標路徑規劃

