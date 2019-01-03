function joint_move(sampling_rate,tacc,A,B,C,d,a,alpha,euler,lower_limit,upper_limit)
%inverse kiniematic �D�XA�BB�BC�I���U�b����
  angle_A =ik(A,a,lower_limit,upper_limit);
  angle_B =ik(B,a,lower_limit,upper_limit);
  angle_C =ik(C,a,lower_limit,upper_limit);
%% ���|A��A'�U�b������
point=1;%�O���C�@�B
 for t=-0.5:sampling_rate:-tacc
    h=(t+0.5)/0.5;
    theta_pred(:,point)=(angle_B-angle_A)*h+angle_A;            % �qA��A'����q�ܤ�
    vel_pred(:,point)=(angle_B-angle_A)/0.5;                  % �qA��A'�t��q'�ܤ�
    acc_pred(:,point)=[0;0;0;0;0;0];                          % �qA��A'�[�t��q''�ܤ�
    point=point+1;
 end
path(1)=point-1;   %�����Ĥ@�q�����I�A�YA'

 %%  ���|A' ��C'�U�b����
angleA_prime=theta_pred(:,point-1)';     % A'���U�b���צ�m
delta_B = angleA_prime-angle_B ;
delta_C = angle_C-angle_B ;
for t=(-tacc+sampling_rate):sampling_rate:(tacc-sampling_rate)
    h=(t+tacc)/(2*tacc);
    theta_pred(:,point)=((delta_C*tacc/0.5+delta_B)*(2-h)*h^2-2*delta_B)*h+angle_B+delta_B;   % �qA'��C'�U�b�����ܤ�
    vel_pred(:,point)=((delta_C*tacc/0.5+delta_B)*(1.5-h)*2*h^2-delta_B)/tacc;              % �qA'��C'�U�b���t���ܤ�
    acc_pred(:,point)=( delta_C*tacc/0.5+delta_B)*(1-h)*3*h/tacc^2;                         % �qA'��C'�U�b�[�t���ܤ�
    point=point+1;
end
path(2)=point-1;   %�����ĤG�q�����I�A�YC'

%%  ���|C' ��C�U�b�����ܤƱ���
for t=tacc:sampling_rate:0.5;
    h=t/0.5;
    theta_pred(:,point)=delta_C*h+angle_B; % �qC'��C�U�b�����ܤ�
    vel_pred(:,point)=delta_C/0.5;       % �qC'��C�U�b�t���ܤ�
    acc_pred(:,point)=[0;0;0;0;0;0];     % �qC'��C�U�b�[�t���ܤ�
    point=point+1;
end
    point=point-1;
path(3)=point;      %�����ĤT�q�����I�A�YC
%% ø�X�t�סB���סB�[�t�׵��G
%--------------------------------����--------------------------------------
angle_plot=figure('Name','Joint1~6 Theta');
t=-0.5:sampling_rate:0.5;
subplot(3,2,1);plot(t,theta_pred(1,:));title('joint1');grid on;xlabel('t');%ø�sjoint_1
subplot(3,2,2);plot(t,theta_pred(2,:));title('joint2');grid on;xlabel('t');%ø�sjoint_2
subplot(3,2,3);plot(t,theta_pred(3,:));title('joint3');grid on;xlabel('t');ylabel('Angle(�X)');%ø�sjoint_3
subplot(3,2,4);plot(t,theta_pred(4,:));title('joint4');grid on;xlabel('t');%ø�sjoint_4
subplot(3,2,5);plot(t,theta_pred(5,:));title('joint5');grid on;xlabel('t');%ø�sjoint_5
subplot(3,2,6);plot(t,theta_pred(6,:));title('joint6');grid on;xlabel('t');%ø�sjoint_6
%---------------------------------�t��--------------------------------------
vel_plot=figure('Name','Joint1~6 Angular Velocity');
subplot(3,2,1);plot(t,vel_pred(1,:));title('joint1');grid on;xlabel('t');%ø�sjoint_1
subplot(3,2,2);plot(t,vel_pred(2,:));title('joint2');grid on;xlabel('t');%ø�sjoint_2
subplot(3,2,3);plot(t,vel_pred(3,:));title('joint3');grid on;xlabel('t');ylabel('Angular Velocity(�X/sec)');%%ø�sjoint_3
subplot(3,2,4);plot(t,vel_pred(4,:));title('joint4');grid on;xlabel('t');%ø�sjoint_4
subplot(3,2,5);plot(t,vel_pred(5,:));title('joint5');grid on;xlabel('t');%ø�sjoint_5
subplot(3,2,6);plot(t,vel_pred(6,:));title('joint6');grid on;xlabel('t');%ø�sjoint_6
%---------------------------------�[�t��-------------------------------------
acc_plot=figure('Name','Joint1~6 Angular Acceleration');
subplot(3,2,1);plot(t,acc_pred(1,:));title('joint1');grid on;xlabel('t');%ø�sjoint_1
subplot(3,2,2);plot(t,acc_pred(2,:));title('joint2');grid on;xlabel('t');%ø�sjoint_2
subplot(3,2,3);plot(t,acc_pred(3,:));title('joint3');grid on;xlabel('t');ylabel('Angular Acceleration(�X/sec)');%ø�sjoint_3
subplot(3,2,4);plot(t,acc_pred(4,:));title('joint4');grid on;xlabel('t');%ø�sjoint_4
subplot(3,2,5);plot(t,acc_pred(5,:));title('joint5');grid on;xlabel('t');%ø�sjoint_5
subplot(3,2,6);plot(t,acc_pred(6,:));title('joint6');grid on;xlabel('t');%ø�sjoint_6
%----------------------------------------------------------------------------------------------------
%% �������|�W�C�@�I�� Cartestion �y��
point=1;
for tt=-0.5:sampling_rate:0.5
[px(point), py(point), pz(point), phi(point),theta(point),psi(point),ax(point),ay(point),az(point)]=fk(theta_pred(:,point)',d,alpha,a,euler);
 px(point)=100*px(point);%���ش�����
 py(point)=100*py(point);
 pz(point)=100*pz(point);
point=point+1;
end
%% ø�s������u�ϡAwithout direction
Cartesian_without_direction=figure('Name','Joint Move without Direction ');
A_CM=100*A(1:3,4); %�����ऽ��
B_CM=100*B(1:3,4);
C_CM=100*C(1:3,4);
scatter3(A_CM(1),A_CM(2),A_CM(3),'k','filled');% �e�XA
hold on;
scatter3(B_CM(1),B_CM(2),B_CM(3),'k','filled');% �e�XB
hold on;
scatter3(C_CM(1),C_CM(2),C_CM(3),'k','filled');% �e�XC

plot3([A_CM(1),B_CM(1)],[A_CM(2),B_CM(2)],[A_CM(3),B_CM(3)],'k:');   % �e�XA_B�u�q
plot3([B_CM(1),C_CM(1)],[B_CM(2),C_CM(2)],[B_CM(3),C_CM(3)],'k:');   % �e�XB_C�u�q
plot3(px(1:path(1)),py(1:path(1)),pz(1:path(1)),'r-');% �e�XA~A'
plot3(px(path(1)+1:path(2)),py(path(1)+1:path(2)),pz(path(1)+1:path(2)),'g-');%�e�XA'~C'
plot3(px(path(2)+1:path(3)),py(path(2)+1:path(3)),pz(path(2)+1:path(3)),'b-');%�e�XC'~C

text(A_CM(1)+1,A_CM(2)+1,A_CM(3)+1,strcat('A(',num2str(A_CM(1)),',',num2str(A_CM(2)),',',num2str(A_CM(3)),')'));% ���A�y��
text(B_CM(1)+1,B_CM(2)+1,B_CM(3)+1,strcat('B(',num2str(B_CM(1)),',',num2str(B_CM(2)),',',num2str(B_CM(3)),')'));% ���B�y��
text(C_CM(1)+1,C_CM(2)+1,C_CM(3)+1,strcat('C(',num2str(C_CM(1)),',',num2str(C_CM(2)),',',num2str(C_CM(3)),')'));% ���C�y��
text(px(path(1)),py(path(1)),pz(path(1)),strcat('A''(',num2str(px(path(1))),',',num2str(py(path(1))),',',num2str(pz(path(1))),')'));% ���A'�y��
text(px(path(2)),py(path(2)),pz(path(2)),strcat('C''(',num2str(px(path(2))),',',num2str(py(path(2))),',',num2str(pz(path(2))),')'));% ���C'�y��

plot3([A_CM(1),A_CM(1)+10*A(1,1)],[A_CM(2),A_CM(2)+10*A(2,1)],[A_CM(3),A_CM(3)+10*A(3,1)],'r','LineWidth',2);% �e�XA��n��V
plot3([A_CM(1),A_CM(1)+10*A(1,2)],[A_CM(2),A_CM(2)+10*A(2,2)],[A_CM(3),A_CM(3)+10*A(3,2)],'g','LineWidth',2);% �e�XA��o��V
plot3([A_CM(1),A_CM(1)+10*A(1,3)],[A_CM(2),A_CM(2)+10*A(2,3)],[A_CM(3),A_CM(3)+10*A(3,3)],'b','LineWidth',2);% �e�XA��a��V
plot3([B_CM(1),B_CM(1)+10*B(1,1)],[B_CM(2),B_CM(2)+10*B(2,1)],[B_CM(3),B_CM(3)+10*B(3,1)],'r','LineWidth',2);% �e�XB��n��V
plot3([B_CM(1),B_CM(1)+10*B(1,2)],[B_CM(2),B_CM(2)+10*B(2,2)],[B_CM(3),B_CM(3)+10*B(3,2)],'g','LineWidth',2);% �e�XB��o��V
plot3([B_CM(1),B_CM(1)+10*B(1,3)],[B_CM(2),B_CM(2)+10*B(2,3)],[B_CM(3),B_CM(3)+10*B(3,3)],'b','LineWidth',2);% �e�XB��a��V
plot3([C_CM(1),C_CM(1)+10*C(1,1)],[C_CM(2),C_CM(2)+10*C(2,1)],[C_CM(3),C_CM(3)+10*C(3,1)],'r','LineWidth',2);% �e�XC��n��V
plot3([C_CM(1),C_CM(1)+10*C(1,2)],[C_CM(2),C_CM(2)+10*C(2,2)],[C_CM(3),C_CM(3)+10*C(3,2)],'g','LineWidth',2);% �e�XC��o��V
plot3([C_CM(1),C_CM(1)+10*C(1,3)],[C_CM(2),C_CM(2)+10*C(2,3)],[C_CM(3),C_CM(3)+10*C(3,3)],'b','LineWidth',2);% �e�XC��a��V

axis([-50 50 -50 50 10 35]);grid on;xlabel('X(cm)');ylabel('Y(cm)');zlabel('Z(cm)');
title('3D path of Joint Motion');hold off;

%% ø�s������u�ϡAwith direction
Cartesian_with_direction=figure('Name','Joint Move with Direction ');

scatter3(A_CM(1),A_CM(2),A_CM(3),'k','filled');% �e�XA
hold on;
scatter3(B_CM(1),B_CM(2),B_CM(3),'k','filled');% �e�XB
hold on;
scatter3(C_CM(1),C_CM(2),C_CM(3),'k','filled');% �e�XC

plot3([A_CM(1),B_CM(1)],[A_CM(2),B_CM(2)],[A_CM(3),B_CM(3)],'k:');   % �e�XA_B�u�q
plot3([B_CM(1),C_CM(1)],[B_CM(2),C_CM(2)],[B_CM(3),C_CM(3)],'k:');   % �e�XB_C�u�q
plot3(px(1:path(1)),py(1:path(1)),pz(1:path(1)),'r-');% �e�XA~A'
plot3(px(path(1)+1:path(2)),py(path(1)+1:path(2)),pz(path(1)+1:path(2)),'g-');%�e�XA'~C'
plot3(px(path(2)+1:path(3)),py(path(2)+1:path(3)),pz(path(2)+1:path(3)),'b-');%�e�XC'~C

text(A_CM(1)+1,A_CM(2)+1,A_CM(3)+1,strcat('A(',num2str(A_CM(1)),',',num2str(A_CM(2)),',',num2str(A_CM(3)),')'));% ���A�y��
text(B_CM(1)+1,B_CM(2)+1,B_CM(3)+1,strcat('B(',num2str(B_CM(1)),',',num2str(B_CM(2)),',',num2str(B_CM(3)),')'));% ���B�y��
text(C_CM(1)+1,C_CM(2)+1,C_CM(3)+1,strcat('C(',num2str(C_CM(1)),',',num2str(C_CM(2)),',',num2str(C_CM(3)),')'));% ���C�y��
text(px(path(1)),py(path(1)),pz(path(1)),strcat('A''(',num2str(px(path(1))),',',num2str(py(path(1))),',',num2str(pz(path(1))),')'));% ���A'�y��
text(px(path(2)),py(path(2)),pz(path(2)),strcat('C''(',num2str(px(path(2))),',',num2str(py(path(2))),',',num2str(pz(path(2))),')'));% ���C'�y��

plot3([A_CM(1),A_CM(1)+10*A(1,1)],[A_CM(2),A_CM(2)+10*A(2,1)],[A_CM(3),A_CM(3)+10*A(3,1)],'r','LineWidth',2);% �e�XA��n��V
plot3([A_CM(1),A_CM(1)+10*A(1,2)],[A_CM(2),A_CM(2)+10*A(2,2)],[A_CM(3),A_CM(3)+10*A(3,2)],'g','LineWidth',2);% �e�XA��o��V
plot3([A_CM(1),A_CM(1)+10*A(1,3)],[A_CM(2),A_CM(2)+10*A(2,3)],[A_CM(3),A_CM(3)+10*A(3,3)],'b','LineWidth',2);% �e�XA��a��V
plot3([B_CM(1),B_CM(1)+10*B(1,1)],[B_CM(2),B_CM(2)+10*B(2,1)],[B_CM(3),B_CM(3)+10*B(3,1)],'r','LineWidth',2);% �e�XB��n��V
plot3([B_CM(1),B_CM(1)+10*B(1,2)],[B_CM(2),B_CM(2)+10*B(2,2)],[B_CM(3),B_CM(3)+10*B(3,2)],'g','LineWidth',2);% �e�XB��o��V
plot3([B_CM(1),B_CM(1)+10*B(1,3)],[B_CM(2),B_CM(2)+10*B(2,3)],[B_CM(3),B_CM(3)+10*B(3,3)],'b','LineWidth',2);% �e�XB��a��V
plot3([C_CM(1),C_CM(1)+10*C(1,1)],[C_CM(2),C_CM(2)+10*C(2,1)],[C_CM(3),C_CM(3)+10*C(3,1)],'r','LineWidth',2);% �e�XC��n��V
plot3([C_CM(1),C_CM(1)+10*C(1,2)],[C_CM(2),C_CM(2)+10*C(2,2)],[C_CM(3),C_CM(3)+10*C(3,2)],'g','LineWidth',2);% �e�XC��o��V
plot3([C_CM(1),C_CM(1)+10*C(1,3)],[C_CM(2),C_CM(2)+10*C(2,3)],[C_CM(3),C_CM(3)+10*C(3,3)],'b','LineWidth',2);% �e�XC��a��V

for i = 1 : 3 :path(3) %�C�T�I�e�@�� �קK�ϫ��ӱK
    plot3([px(i),px(i)+5*ax(i)],[py(i),py(i)+5*ay(i)],[pz(i),pz(i)+5*az(i)],'-','LineWidth',0.5);
end
axis([-50 50 -50 50 10 35]);grid on;xlabel('X(cm)');ylabel('Y(cm)');zlabel('Z(cm)');
title('3D path of Joint Motion');hold off;
end

