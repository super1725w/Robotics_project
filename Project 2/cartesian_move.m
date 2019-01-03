function cartesian_move(sampling_rate,tacc,A,B,C)
 nA=[A(1:3,1)]'; oA=[A(1:3,2)]'; aA=[A(1:3,3)]' ; A_CM=[A(1:3,4)]';
 nB=[B(1:3,1)]'; oB=[B(1:3,2)]'; aB=[B(1:3,3)]' ; B_CM=[B(1:3,4)]';
 nC=[C(1:3,1)]'; oC=[C(1:3,2)]'; aC=[C(1:3,3)]' ; C_CM=[C(1:3,4)]';
 %----------------------------------------------------------------------------------------------------
 %% ���|A��A'cartesian_motion�ܤ�
point=1;
 for t=-0.5:sampling_rate:-tacc
    h=(t+0.5)/0.5;
    dx = dot(nA, (B_CM - A_CM))*h; 
    dy = dot(oA, (B_CM - A_CM))*h; 
    dz = dot(aA, (B_CM - A_CM))*h;
    dpsi   = atan2(dot(oA,aB),  dot(nA, aB));
    dtheta = atan2(sqrt(dot(nA, aB)^2 + dot(oA, aB)^2),dot(aA, aB))*h;
    V_theta=1-cos(dtheta);   
    sin_phi = -sin(dpsi)*cos(dpsi)*V_theta*dot(nA, nB) + (cos(dpsi)^2*V_theta+cos(dtheta))*dot(oA, nB) - sin(dpsi)*sin(dtheta)*dot(aA, nB);
    cos_phi = -sin(dpsi)*cos(dpsi)*V_theta*dot(nA, oB) + (cos(dpsi)^2*V_theta+cos(dtheta))*dot(oA, oB) - sin(dpsi)*sin(dtheta)*dot(aA, oB);
    dphi = atan2(sin_phi, cos_phi)*h;
    
    S_psi=sin(dpsi);       
    C_psi=cos(dpsi);     
    S_theta=sin(dtheta);    
    C_theta=cos(dtheta);   
    S_phi=sin(dphi);    
    C_phi=cos(dphi);
    
    T = [1 0 0 dx;
          0 1 0 dy; 
          0 0 1 dz; 
          0 0 0 1 ];
    Ra = [(S_psi^2)*V_theta+C_theta ,     -S_psi*C_psi*V_theta , C_psi*S_theta , 0;
              -S_psi*C_psi*V_theta  ,  C_psi^2*V_theta+C_theta , S_psi*S_theta , 0;
                    -C_psi*S_theta  ,           -S_psi*S_theta ,       C_theta , 0;
                                 0  ,                        0 ,             0 , 1];
    Ro = [ C_phi , -S_phi , 0 , 0;
           S_phi ,  C_phi , 0 , 0;
               0 ,      0 , 1 , 0;
               0 ,      0 , 0 , 1];
    D = T*Ra*Ro;
    position(:,:,point)=A*D;
    px(point)=position(1,4,point)*100;    py(point)=position(2,4,point)*100;    pz(point)=position(3,4,point)*100;%����p�A���ର����
    ax(point)=position(1,3,point);    ay(point)=position(2,3,point);    az(point)=position(3,3,point);%����a
    point=point+1;
 end
path(1)=point-1;  %�����Ĥ@�q�����I�A�YA'
A_prime=position(:,:,path(1));% ����A'�I
A_prime_n=[A_prime(1:3,1)]'; A_prime_o=[A_prime(1:3,2)]'; A_prime_a=[A_prime(1:3,3)]' ; A_prime_p=[A_prime(1:3,4)]';

 %% ���|A'��C'�U�b�����ܤ�
    delta_Bx = dot(nB, (A_prime_p - B_CM)); delta_By =dot(oB, (A_prime_p - B_CM)); delta_Bz = dot(aB, (A_prime_p - B_CM));
    delta_Cx = dot(nB, (C_CM - B_CM)); delta_Cy =dot(oB, (C_CM - B_CM)); delta_Cz = dot(aB, (C_CM - B_CM));
    delta_psiB   = atan2(dot(oB,A_prime_a),  dot(nB, A_prime_a));
    delta_psiC   = atan2(dot(oB,aC),  dot(nB, aC));
    delta_thetaB = atan2(sqrt(dot(nB, A_prime_a)^2 + dot(oB, A_prime_a)^2),dot(aB, A_prime_a));
    delta_thetaC = atan2(sqrt(dot(nB, aC)^2 + dot(oB, aC)^2),dot(aB, aC));
    V_thetaB=1-cos(delta_thetaB);   V_thetaC=1-cos(delta_thetaC);
    sin_phiB = -sin(delta_psiB)*cos(delta_psiB)*V_thetaB*dot(nB, A_prime_n) + (cos(delta_psiB)^2*V_thetaB+cos(delta_thetaB))*dot(oB, A_prime_n) - sin(delta_psiB)*sin(delta_thetaB)*dot(aB, A_prime_n);
    cos_phiB = -sin(delta_psiB)*cos(delta_psiB)*V_thetaB*dot(nB, A_prime_o) + (cos(delta_psiB)^2*V_thetaB+cos(delta_thetaB))*dot(oB, A_prime_o) - sin(delta_psiB)*sin(delta_thetaB)*dot(aB, A_prime_o);
    sin_phiC = -sin(delta_psiC)*cos(delta_psiC)*V_thetaC*dot(nB, nC) + (cos(delta_psiC)^2*V_thetaC+cos(delta_thetaC))*dot(oB, nC) - sin(delta_psiC)*sin(delta_thetaC)*dot(aB, nC);
    cos_phiC = -sin(delta_psiC)*cos(delta_psiC)*V_thetaC*dot(nB, oC) + (cos(delta_psiC)^2*V_thetaC+cos(delta_thetaC))*dot(oB, oC) - sin(delta_psiC)*sin(delta_thetaC)*dot(aB, oC);
    delta_phiB = atan2(sin_phiB, cos_phiB);
    delta_phiC = atan2(sin_phiC, cos_phiC);
    if abs(delta_psiC-delta_psiB)>pi/2
        delta_psiB=delta_psiB+pi;
        delta_thetaB=-delta_thetaB;
    end
for t=(-tacc+sampling_rate):sampling_rate:(tacc-sampling_rate)
    h=(t+0.2)/(0.4);
    dx =((delta_Cx*tacc/0.5+delta_Bx)*(2-h)*h^2-2*delta_Bx)*h+delta_Bx;
    dy =((delta_Cy*tacc/0.5+delta_By)*(2-h)*h^2-2*delta_By)*h+delta_By;
    dz =((delta_Cz*tacc/0.5+delta_Bz)*(2-h)*h^2-2*delta_Bz)*h+delta_Bz;
    dpsi=(delta_psiC-delta_psiB)*h+delta_psiB;
    dtheta=((delta_thetaC*tacc/0.5+delta_thetaB)*(2-h)*h^2-2*delta_thetaB)*h+delta_thetaB;
    dphi=((delta_phiC*tacc/0.5+delta_phiB)*(2-h)*h^2-2*delta_phiB)*h+delta_phiB;
    S_psi=sin(dpsi);       C_psi=cos(dpsi);     S_theta=sin(dtheta);    C_theta=cos(dtheta);   S_phi=sin(dphi);    C_phi=cos(dphi);
    V_theta=1-C_theta;   
    T  = [1 0 0 dx;
          0 1 0 dy; 
          0 0 1 dz; 
          0 0 0 1 ];
    Ra = [(S_psi^2)*V_theta+C_theta ,     -S_psi*C_psi*V_theta , C_psi*S_theta , 0;
              -S_psi*C_psi*V_theta  ,  C_psi^2*V_theta+C_theta , S_psi*S_theta , 0;
                    -C_psi*S_theta  ,           -S_psi*S_theta ,       C_theta , 0;
                                 0  ,                        0 ,             0 , 1];
    Ro =  [C_phi ,  -S_phi, 0 , 0;
           S_phi ,  C_phi , 0 , 0;
               0 ,      0 , 1 , 0;
               0 ,      0 , 0 , 1 ];
    D = T*Ra*Ro;
    position(:,:,point)=B*D;
    px(point)=position(1,4,point)*100;    py(point)=position(2,4,point)*100;    pz(point)=position(3,4,point)*100; %����p�A���ର����
    ax(point)=position(1,3,point);    ay(point)=position(2,3,point);    az(point)=position(3,3,point);
    point=point+1;
end
path(2)=point-1 ;  %�����ĤG�q�����I�A�YC'
C_prime=position(:,:,path(2));% C���x�}
C_prime_n=[C_prime(1:3,1)]'; C_prime_o=[C_prime(1:3,2)]'; C_prime_a=[C_prime(1:3,3)]' ; C_prime_p=[C_prime(1:3,4)]';
%% ���|C'��C�U�b�����ܤ�
for t=tacc:sampling_rate:0.5
    h=t/0.5;
    dx = dot(nB, (C_CM - B_CM))*h; dy =dot(oB, (C_CM - B_CM))*h; dz = dot(aB, (C_CM - B_CM))*h; 
    dpsi   = atan2(dot(oB,aC),  dot(nB, aC));
    dtheta =  atan2(sqrt(dot(nB, aC)^2 + dot(oB, aC)^2),dot(aB, aC))*h;
    V_theta=1-cos(dtheta);   
    sin_phi = -sin(dpsi)*cos(dpsi)*V_theta*dot(nB, nC) + (cos(dpsi)^2*V_theta+cos(dtheta))*dot(oB, nC) - sin(dpsi)*sin(dtheta)*dot(aB, nC);
    cos_phi = -sin(dpsi)*cos(dpsi)*V_theta*dot(nB, oC) + (cos(dpsi)^2*V_theta+cos(dtheta))*dot(oB, oC) - sin(dpsi)*sin(dtheta)*dot(aB, oC);
    dphi = atan2(sin_phi, cos_phi);
  
    S_psi=sin(dpsi);       C_psi=cos(dpsi);     S_theta=sin(dtheta);    
    C_theta=cos(dtheta);   S_phi=sin(dphi);     C_phi=cos(dphi);
    T = [1 0 0 dx;
          0 1 0 dy; 
          0 0 1 dz; 
          0 0 0 1 ];
    Ra = [S_psi^2*V_theta+C_theta ,     -S_psi*C_psi*V_theta, C_psi*S_theta, 0;
              -S_psi*C_psi*V_theta,  C_psi^2*V_theta+C_theta, S_psi*S_theta, 0;
                    -C_psi*S_theta,           -S_psi*S_theta,       C_theta, 0;
                                 0,                        0,             0, 1];
    Ro = [C_phi, -S_phi, 0, 0;
           S_phi,  C_phi, 0, 0;
               0,      0, 1, 0;
               0,      0, 0, 1];
    D = T*Ra*Ro;
    position(:,:,point)=B*D;
    px(point)=position(1,4,point)*100;    py(point)=position(2,4,point)*100;    pz(point)=position(3,4,point)*100; %����p�A���ର����
    ax(point)=position(1,3,point);    ay(point)=position(2,3,point);    az(point)=position(3,3,point);
    point=point+1;
 end
    point=point-1;
    path(3)=point;     %�����ĤT�q�����I�A�YC
%----------------------------------------------------------------------------------------------------
%% ø�sA~C��x,y,z�U�b�ܤ�
px;py;pz;%Position
px_v=diff(px)/sampling_rate;   py_v=diff(py)/sampling_rate;   pz_v=diff(pz)/sampling_rate;% Velocity
px_acc=diff(px_v)/sampling_rate;  py_acc=diff(py_v)/sampling_rate;  pz_acc=diff(pz_v)/sampling_rate;% Accelaration 
t=-0.5:sampling_rate:0.5;dt=t(2:point);ddt=t(3:point);% �ɶ��H�۷L���j���Y�p
end_xyz_plot=figure('Name','�����I(x,y,z) ��m�B�t�סB�[�t�פ��ܤ� ');
subplot(3,3,1);grid on;plot(t,px)    ;title('position of x')    ;axis([-0.5 0.5 -30 30]);grid on;                                                 
subplot(3,3,4);grid on;plot(t,py)    ;title('position of y')    ;ylabel('Position(cm)');axis([-0.5 0.5 -40 60]);grid on;            
subplot(3,3,7);grid on;plot(t,pz)    ;title('position of z')    ;xlabel('Time(s)');axis([-0.5 0.5 20 30]);grid on;                               
subplot(3,3,2);grid on;plot(dt,px_v)  ;title('velocity of x')    ;axis([-0.5 0.5 -150 50]);grid on;
subplot(3,3,5);grid on;plot(dt,py_v)  ;title('velocity of y')    ;ylabel('Velocity(cm/s)');axis([-0.5 0.5 -120 -20]);grid on;
subplot(3,3,8);grid on;plot(dt,pz_v)  ;title('velocity of z')    ;xlabel('Time(s)');axis([-0.5 0.5 -20 20]);grid on;
subplot(3,3,3);grid on;plot(ddt,px_acc);title('acceleration of x');axis([-0.5 0.5 -600 200]);grid on;
subplot(3,3,6);grid on;plot(ddt,py_acc);title('acceleration of y');ylabel('Acceleration(cm/s^2)');axis([-0.5 0.5 -300 100]);grid on;
subplot(3,3,9);grid on;plot(ddt,pz_acc);title('acceleration of z');xlabel('Time(s)');axis([-0.5 0.5 -50 150]);grid on;

%% ø�s������u�Awithout direction
Cartesian_no_direction_plot=figure('Name','Cartesian Move without Direction  ');
A_CM=100*A(1:3,4);  %�����ऽ��
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

axis([-40 40 -40 60 10 30]);grid on;xlabel('X(cm)');ylabel('Y(cm)');zlabel('Z(cm)');
view([30,40,120])
title('3D path of Cartesian Motion');hold off;

%% ø�s������u�Awith direction
Cartesian_no_direction_plot=figure('Name','Cartesian Move with Direction ');
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

axis([-40 40 -40 60 10 30]);grid on;xlabel('X(cm)');ylabel('Y(cm)');zlabel('Z(cm)');
view([30,60,120])
title('3D path of Cartesian Motion');hold off;
end