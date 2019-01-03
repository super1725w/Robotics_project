function joint_move(sampling_rate,tacc,A,B,C,d,a,alpha,euler,lower_limit,upper_limit)
%inverse kiniematic 求出A、B、C點的各軸角度
  angle_A =ik(A,a,lower_limit,upper_limit);
  angle_B =ik(B,a,lower_limit,upper_limit);
  angle_C =ik(C,a,lower_limit,upper_limit);
%% 路徑A到A'各軸的角度
point=1;%記錄每一步
 for t=-0.5:sampling_rate:-tacc
    h=(t+0.5)/0.5;
    theta_pred(:,point)=(angle_B-angle_A)*h+angle_A;            % 從A到A'角度q變化
    vel_pred(:,point)=(angle_B-angle_A)/0.5;                  % 從A到A'速度q'變化
    acc_pred(:,point)=[0;0;0;0;0;0];                          % 從A到A'加速度q''變化
    point=point+1;
 end
path(1)=point-1;   %紀錄第一段之終點，即A'

 %%  路徑A' 到C'各軸角度
angleA_prime=theta_pred(:,point-1)';     % A'的各軸角度位置
delta_B = angleA_prime-angle_B ;
delta_C = angle_C-angle_B ;
for t=(-tacc+sampling_rate):sampling_rate:(tacc-sampling_rate)
    h=(t+tacc)/(2*tacc);
    theta_pred(:,point)=((delta_C*tacc/0.5+delta_B)*(2-h)*h^2-2*delta_B)*h+angle_B+delta_B;   % 從A'到C'各軸角度變化
    vel_pred(:,point)=((delta_C*tacc/0.5+delta_B)*(1.5-h)*2*h^2-delta_B)/tacc;              % 從A'到C'各軸角速度變化
    acc_pred(:,point)=( delta_C*tacc/0.5+delta_B)*(1-h)*3*h/tacc^2;                         % 從A'到C'各軸加速度變化
    point=point+1;
end
path(2)=point-1;   %紀錄第二段之終點，即C'

%%  路徑C' 到C各軸角度變化情形
for t=tacc:sampling_rate:0.5;
    h=t/0.5;
    theta_pred(:,point)=delta_C*h+angle_B; % 從C'到C各軸角度變化
    vel_pred(:,point)=delta_C/0.5;       % 從C'到C各軸速度變化
    acc_pred(:,point)=[0;0;0;0;0;0];     % 從C'到C各軸加速度變化
    point=point+1;
end
    point=point-1;
path(3)=point;      %紀錄第三段之終點，即C
%% 繪出速度、角度、加速度結果
%--------------------------------角度--------------------------------------
angle_plot=figure('Name','Joint1~6 Theta');
t=-0.5:sampling_rate:0.5;
subplot(3,2,1);plot(t,theta_pred(1,:));title('joint1');grid on;xlabel('t');%繪製joint_1
subplot(3,2,2);plot(t,theta_pred(2,:));title('joint2');grid on;xlabel('t');%繪製joint_2
subplot(3,2,3);plot(t,theta_pred(3,:));title('joint3');grid on;xlabel('t');ylabel('Angle(°)');%繪製joint_3
subplot(3,2,4);plot(t,theta_pred(4,:));title('joint4');grid on;xlabel('t');%繪製joint_4
subplot(3,2,5);plot(t,theta_pred(5,:));title('joint5');grid on;xlabel('t');%繪製joint_5
subplot(3,2,6);plot(t,theta_pred(6,:));title('joint6');grid on;xlabel('t');%繪製joint_6
%---------------------------------速度--------------------------------------
vel_plot=figure('Name','Joint1~6 Angular Velocity');
subplot(3,2,1);plot(t,vel_pred(1,:));title('joint1');grid on;xlabel('t');%繪製joint_1
subplot(3,2,2);plot(t,vel_pred(2,:));title('joint2');grid on;xlabel('t');%繪製joint_2
subplot(3,2,3);plot(t,vel_pred(3,:));title('joint3');grid on;xlabel('t');ylabel('Angular Velocity(°/sec)');%%繪製joint_3
subplot(3,2,4);plot(t,vel_pred(4,:));title('joint4');grid on;xlabel('t');%繪製joint_4
subplot(3,2,5);plot(t,vel_pred(5,:));title('joint5');grid on;xlabel('t');%繪製joint_5
subplot(3,2,6);plot(t,vel_pred(6,:));title('joint6');grid on;xlabel('t');%繪製joint_6
%---------------------------------加速度-------------------------------------
acc_plot=figure('Name','Joint1~6 Angular Acceleration');
subplot(3,2,1);plot(t,acc_pred(1,:));title('joint1');grid on;xlabel('t');%繪製joint_1
subplot(3,2,2);plot(t,acc_pred(2,:));title('joint2');grid on;xlabel('t');%繪製joint_2
subplot(3,2,3);plot(t,acc_pred(3,:));title('joint3');grid on;xlabel('t');ylabel('Angular Acceleration(°/sec)');%繪製joint_3
subplot(3,2,4);plot(t,acc_pred(4,:));title('joint4');grid on;xlabel('t');%繪製joint_4
subplot(3,2,5);plot(t,acc_pred(5,:));title('joint5');grid on;xlabel('t');%繪製joint_5
subplot(3,2,6);plot(t,acc_pred(6,:));title('joint6');grid on;xlabel('t');%繪製joint_6
%----------------------------------------------------------------------------------------------------
%% 紀錄路徑上每一點的 Cartestion 座標
point=1;
for tt=-0.5:sampling_rate:0.5
[px(point), py(point), pz(point), phi(point),theta(point),psi(point),ax(point),ay(point),az(point)]=fk(theta_pred(:,point)',d,alpha,a,euler);
 px(point)=100*px(point);%公尺換公分
 py(point)=100*py(point);
 pz(point)=100*pz(point);
point=point+1;
end
%% 繪製立體路線圖，without direction
Cartesian_without_direction=figure('Name','Joint Move without Direction ');
A_CM=100*A(1:3,4); %公尺轉公分
B_CM=100*B(1:3,4);
C_CM=100*C(1:3,4);
scatter3(A_CM(1),A_CM(2),A_CM(3),'k','filled');% 畫出A
hold on;
scatter3(B_CM(1),B_CM(2),B_CM(3),'k','filled');% 畫出B
hold on;
scatter3(C_CM(1),C_CM(2),C_CM(3),'k','filled');% 畫出C

plot3([A_CM(1),B_CM(1)],[A_CM(2),B_CM(2)],[A_CM(3),B_CM(3)],'k:');   % 畫出A_B線段
plot3([B_CM(1),C_CM(1)],[B_CM(2),C_CM(2)],[B_CM(3),C_CM(3)],'k:');   % 畫出B_C線段
plot3(px(1:path(1)),py(1:path(1)),pz(1:path(1)),'r-');% 畫出A~A'
plot3(px(path(1)+1:path(2)),py(path(1)+1:path(2)),pz(path(1)+1:path(2)),'g-');%畫出A'~C'
plot3(px(path(2)+1:path(3)),py(path(2)+1:path(3)),pz(path(2)+1:path(3)),'b-');%畫出C'~C

text(A_CM(1)+1,A_CM(2)+1,A_CM(3)+1,strcat('A(',num2str(A_CM(1)),',',num2str(A_CM(2)),',',num2str(A_CM(3)),')'));% 顯示A座標
text(B_CM(1)+1,B_CM(2)+1,B_CM(3)+1,strcat('B(',num2str(B_CM(1)),',',num2str(B_CM(2)),',',num2str(B_CM(3)),')'));% 顯示B座標
text(C_CM(1)+1,C_CM(2)+1,C_CM(3)+1,strcat('C(',num2str(C_CM(1)),',',num2str(C_CM(2)),',',num2str(C_CM(3)),')'));% 顯示C座標
text(px(path(1)),py(path(1)),pz(path(1)),strcat('A''(',num2str(px(path(1))),',',num2str(py(path(1))),',',num2str(pz(path(1))),')'));% 顯示A'座標
text(px(path(2)),py(path(2)),pz(path(2)),strcat('C''(',num2str(px(path(2))),',',num2str(py(path(2))),',',num2str(pz(path(2))),')'));% 顯示C'座標

plot3([A_CM(1),A_CM(1)+10*A(1,1)],[A_CM(2),A_CM(2)+10*A(2,1)],[A_CM(3),A_CM(3)+10*A(3,1)],'r','LineWidth',2);% 畫出A之n方向
plot3([A_CM(1),A_CM(1)+10*A(1,2)],[A_CM(2),A_CM(2)+10*A(2,2)],[A_CM(3),A_CM(3)+10*A(3,2)],'g','LineWidth',2);% 畫出A之o方向
plot3([A_CM(1),A_CM(1)+10*A(1,3)],[A_CM(2),A_CM(2)+10*A(2,3)],[A_CM(3),A_CM(3)+10*A(3,3)],'b','LineWidth',2);% 畫出A之a方向
plot3([B_CM(1),B_CM(1)+10*B(1,1)],[B_CM(2),B_CM(2)+10*B(2,1)],[B_CM(3),B_CM(3)+10*B(3,1)],'r','LineWidth',2);% 畫出B之n方向
plot3([B_CM(1),B_CM(1)+10*B(1,2)],[B_CM(2),B_CM(2)+10*B(2,2)],[B_CM(3),B_CM(3)+10*B(3,2)],'g','LineWidth',2);% 畫出B之o方向
plot3([B_CM(1),B_CM(1)+10*B(1,3)],[B_CM(2),B_CM(2)+10*B(2,3)],[B_CM(3),B_CM(3)+10*B(3,3)],'b','LineWidth',2);% 畫出B之a方向
plot3([C_CM(1),C_CM(1)+10*C(1,1)],[C_CM(2),C_CM(2)+10*C(2,1)],[C_CM(3),C_CM(3)+10*C(3,1)],'r','LineWidth',2);% 畫出C之n方向
plot3([C_CM(1),C_CM(1)+10*C(1,2)],[C_CM(2),C_CM(2)+10*C(2,2)],[C_CM(3),C_CM(3)+10*C(3,2)],'g','LineWidth',2);% 畫出C之o方向
plot3([C_CM(1),C_CM(1)+10*C(1,3)],[C_CM(2),C_CM(2)+10*C(2,3)],[C_CM(3),C_CM(3)+10*C(3,3)],'b','LineWidth',2);% 畫出C之a方向

axis([-50 50 -50 50 10 35]);grid on;xlabel('X(cm)');ylabel('Y(cm)');zlabel('Z(cm)');
title('3D path of Joint Motion');hold off;

%% 繪製立體路線圖，with direction
Cartesian_with_direction=figure('Name','Joint Move with Direction ');

scatter3(A_CM(1),A_CM(2),A_CM(3),'k','filled');% 畫出A
hold on;
scatter3(B_CM(1),B_CM(2),B_CM(3),'k','filled');% 畫出B
hold on;
scatter3(C_CM(1),C_CM(2),C_CM(3),'k','filled');% 畫出C

plot3([A_CM(1),B_CM(1)],[A_CM(2),B_CM(2)],[A_CM(3),B_CM(3)],'k:');   % 畫出A_B線段
plot3([B_CM(1),C_CM(1)],[B_CM(2),C_CM(2)],[B_CM(3),C_CM(3)],'k:');   % 畫出B_C線段
plot3(px(1:path(1)),py(1:path(1)),pz(1:path(1)),'r-');% 畫出A~A'
plot3(px(path(1)+1:path(2)),py(path(1)+1:path(2)),pz(path(1)+1:path(2)),'g-');%畫出A'~C'
plot3(px(path(2)+1:path(3)),py(path(2)+1:path(3)),pz(path(2)+1:path(3)),'b-');%畫出C'~C

text(A_CM(1)+1,A_CM(2)+1,A_CM(3)+1,strcat('A(',num2str(A_CM(1)),',',num2str(A_CM(2)),',',num2str(A_CM(3)),')'));% 顯示A座標
text(B_CM(1)+1,B_CM(2)+1,B_CM(3)+1,strcat('B(',num2str(B_CM(1)),',',num2str(B_CM(2)),',',num2str(B_CM(3)),')'));% 顯示B座標
text(C_CM(1)+1,C_CM(2)+1,C_CM(3)+1,strcat('C(',num2str(C_CM(1)),',',num2str(C_CM(2)),',',num2str(C_CM(3)),')'));% 顯示C座標
text(px(path(1)),py(path(1)),pz(path(1)),strcat('A''(',num2str(px(path(1))),',',num2str(py(path(1))),',',num2str(pz(path(1))),')'));% 顯示A'座標
text(px(path(2)),py(path(2)),pz(path(2)),strcat('C''(',num2str(px(path(2))),',',num2str(py(path(2))),',',num2str(pz(path(2))),')'));% 顯示C'座標

plot3([A_CM(1),A_CM(1)+10*A(1,1)],[A_CM(2),A_CM(2)+10*A(2,1)],[A_CM(3),A_CM(3)+10*A(3,1)],'r','LineWidth',2);% 畫出A之n方向
plot3([A_CM(1),A_CM(1)+10*A(1,2)],[A_CM(2),A_CM(2)+10*A(2,2)],[A_CM(3),A_CM(3)+10*A(3,2)],'g','LineWidth',2);% 畫出A之o方向
plot3([A_CM(1),A_CM(1)+10*A(1,3)],[A_CM(2),A_CM(2)+10*A(2,3)],[A_CM(3),A_CM(3)+10*A(3,3)],'b','LineWidth',2);% 畫出A之a方向
plot3([B_CM(1),B_CM(1)+10*B(1,1)],[B_CM(2),B_CM(2)+10*B(2,1)],[B_CM(3),B_CM(3)+10*B(3,1)],'r','LineWidth',2);% 畫出B之n方向
plot3([B_CM(1),B_CM(1)+10*B(1,2)],[B_CM(2),B_CM(2)+10*B(2,2)],[B_CM(3),B_CM(3)+10*B(3,2)],'g','LineWidth',2);% 畫出B之o方向
plot3([B_CM(1),B_CM(1)+10*B(1,3)],[B_CM(2),B_CM(2)+10*B(2,3)],[B_CM(3),B_CM(3)+10*B(3,3)],'b','LineWidth',2);% 畫出B之a方向
plot3([C_CM(1),C_CM(1)+10*C(1,1)],[C_CM(2),C_CM(2)+10*C(2,1)],[C_CM(3),C_CM(3)+10*C(3,1)],'r','LineWidth',2);% 畫出C之n方向
plot3([C_CM(1),C_CM(1)+10*C(1,2)],[C_CM(2),C_CM(2)+10*C(2,2)],[C_CM(3),C_CM(3)+10*C(3,2)],'g','LineWidth',2);% 畫出C之o方向
plot3([C_CM(1),C_CM(1)+10*C(1,3)],[C_CM(2),C_CM(2)+10*C(2,3)],[C_CM(3),C_CM(3)+10*C(3,3)],'b','LineWidth',2);% 畫出C之a方向

for i = 1 : 3 :path(3) %每三點畫一次 避免圖型太密
    plot3([px(i),px(i)+5*ax(i)],[py(i),py(i)+5*ay(i)],[pz(i),pz(i)+5*az(i)],'-','LineWidth',0.5);
end
axis([-50 50 -50 50 10 35]);grid on;xlabel('X(cm)');ylabel('Y(cm)');zlabel('Z(cm)');
title('3D path of Joint Motion');hold off;
end

