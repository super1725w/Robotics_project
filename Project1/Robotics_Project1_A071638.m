%Before running this progrom, you may have to download robotics toolbox
%from http://petercorke.com/wordpress/toolboxes/robotics-toolbox#Downloading_the_Toolbox
startup_rvc;%����Robotics toolbox
clear;
clc;
%% ��l�ѼƳ]�w�A�B�[�J���׭���
L1limit = [-150/180*pi 150/180*pi];
L2limit = [-30/180*pi 100/180*pi];
L3limit = [-120/180*pi 0/180*pi];
L4limit = [-110/180*pi 110/180*pi];
L5limit = [-180/180*pi 180/180*pi];
L6limit = [-180/180*pi 180/180*pi];
L1 = Link('d',0,'a',0.12,'alpha',-pi/2,'qlim',L1limit);
L2 = Link('d',0,'a',0.25,'alpha',0,'qlim',L2limit);
L3 = Link('d',0,'a',0.26,'alpha',0,'qlim',L3limit);
L4 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',L4limit);
L5 = Link('d',0,'a',0,'alpha',pi/2,'qlim',L5limit);
L6 = Link('d',0,'a',0,'alpha',0,'qlim',L6limit);
qlim = [L1limit;L2limit;L3limit;L4limit;L5limit;L6limit];
%% 
while true
    
    prompt = 'Please type (P1 or P2) to continue and (END) to end:';
    select = input(prompt,'s');
    if strcmp(select,'P1')  %....................................................�Ĥ@�D�{��
        %% ��Jn,o,a,p�ëإ�robot
        prompt = 'Input nx:';nx = str2num(input(prompt,'s'));
        prompt = 'Input ny:';ny = str2num(input(prompt,'s'));
        prompt = 'Input nz:';nz = str2num(input(prompt,'s'));
        n = [nx ny nz];
        prompt = 'Input ox:';ox = str2num(input(prompt,'s'));
        prompt = 'Input oy:';oy = str2num(input(prompt,'s'));
        prompt = 'Input oz:';oz = str2num(input(prompt,'s'));
        o = [ox oy oz];
        prompt = 'Input ax:';ax = str2num(input(prompt,'s'));
        prompt = 'Input ay:';ay = str2num(input(prompt,'s'));
        prompt = 'Input az:';az = str2num(input(prompt,'s'));
        a = [ax ay az];
        prompt = 'Input px:';px = str2num(input(prompt,'s'));
        prompt = 'Input py:';py = str2num(input(prompt,'s'));
        prompt = 'Input pz:';pz = str2num(input(prompt,'s'));
        p = [px py pz];
        robot = SerialLink([L1,L2,L3,L4,L5,L6]);
        Q0 = [0 0 0 0 0 0];%theta��l��
        T = [[n 0]' [o 0]' [a 0]' [p 1]'];%�Q��noap�ഫ��4*4���x�}T
        %% �p��Ҧ�theta����
        answer=popo_calculate(T);
        %% �˴����X�ոѳ��bqlimit����
        in = zeros(1,8); %�p��O�_�C��theta�����b���
        q_ans = [];  %�N�blimit�����ѰO���U��
        for i=1:8    %�˴�8�ո�
           for k = 1:6   %�˴�6�Ө�
              if (qlim(k,1)/pi*180 <= answer(i,k)) && (answer(i,k)<= qlim(k,2)/pi*180)
                   in(i)=in(i)+1;  
              end  
           end
           if in(i)==6 %�Y�O6�Ө����b����h�O��
                    q_ans(i,:) = answer(i,:);
           end
        end
        %% print�X�ѵ�
        fprintf('There are 8 answers�G\n')
        for i =1:8
            fprintf(strcat('Ans(',int2str(i),')','\n'))
            disp([round(answer(i,1),3),round(answer(i,2),3),round(answer(i,3),3),...
            round(answer(i,4),3),round(answer(i,5),3),round(answer(i,6),3)])
            fprintf('-------------------------------------------------------------------------\n')
        end
        fprintf('Only ')
        for i =1:8
           if in(i) ==6
               fprintf(strcat(' Ans(',int2str(i),') '))
           end
        end
        fprintf('in theta limit \n')
        
        for i = 1:size(q_ans,1)
            disp([round(q_ans(i,1),3),round(q_ans(i,2),3),round(q_ans(i,3),3),...
            round(q_ans(i,4),3),round(q_ans(i,5),3),round(q_ans(i,6),3)])
            fprintf('\n')
        end
    elseif strcmp(select,'P2') %...........................................................�ĤG�D�{��
        %% ��Jtheta�ëإ�robot
        correct=0;
        while true  %�Y��Jtheta�W�L����N�n�D���s��J
            if correct==0
                prompt = 'Input theta1:';
                theta1 = str2num(input(prompt,'s'));
                theta1 = theta1/180*pi;
                prompt = 'Input theta2:';
                theta2 = str2num(input(prompt,'s'));
                theta2 = theta2/180*pi;
                prompt = 'Input theta3:';
                theta3 = str2num(input(prompt,'s'));
                theta3 = theta3/180*pi;
                prompt = 'Input theta4:';
                theta4 = str2num(input(prompt,'s'));
                theta4 = theta4/180*pi;
                prompt = 'Input theta5:';
                theta5 = str2num(input(prompt,'s'));
                theta5 = theta5/180*pi;
                prompt = 'Input theta6:';
                theta6 = str2num(input(prompt,'s'));
                theta6 = theta6/180*pi;
                if ((qlim(1,1)>theta1) || (theta1>qlim(1,2)) || (qlim(2,1)>theta2) || (theta2>qlim(2,2)) || ...
                        (qlim(3,1)>theta3) || (theta3>qlim(3,2)) || (qlim(4,1)>theta4) || (theta4>qlim(4,2)) ...
                        || (qlim(5,1)>theta5) || (theta5>qlim(5,2)) || (qlim(6,1)>theta6) || (theta6>qlim(6,2)))  %�P�_�O�_�W�X����
                     correct=0;
                     fprintf('Please type the theta within the limit \n')
                else
                    break
                end
            
            end
        end
        theta = [theta1 theta2 theta3 theta4 theta5 theta6];
        robot = SerialLink([L1,L2,L3,L4,L5,L6]);
        %% �p��noap�����
        noap = robot.fkine(theta);%forward kinematics
        noap_ = [noap.n noap.o noap.a noap.t];
        disp('(n, o, a, p) = ');
        disp(round(noap_,3))
        %% �p��RPY��theta'phi'psi  ������ո�
        roll_RPY_1 = atan2(noap.n(2),noap.n(1)); %phi���Ĥ@�ո�
        pitch_RPY_1 = atan2( -noap.n(3) , (cos(roll_RPY_1)*noap.n(1)+sin(roll_RPY_1)*noap.n(2)));%�Q��phi�p��theta
        yaw_RPY_1 = atan2((sin(roll_RPY_1)*noap.a(1)-cos(roll_RPY_1)*noap.a(2)) , (-sin(roll_RPY_1)*noap.o(1)+cos(roll_RPY_1)*noap.o(2)));%�Q��phi�p��psi
        fprintf('For RPY, (x, y, z, �X, �c, �r)=\n')
        disp([round(noap.t(1),3), round(noap.t(2),3),round(noap.t(3),3),...
            round(roll_RPY_1,3),round(pitch_RPY_1,3),round(yaw_RPY_1,3)]);
        fprintf('        or      \n')
        roll_RPY_2 = atan2(-noap.n(2),-noap.n(1));%phi���ĤG�ո�
        pitch_RPY_2 = atan2( -noap.n(3) , (cos(roll_RPY_2)*noap.n(1)+sin(roll_RPY_2)*noap.n(2)));%�Q��phi�p��theta
        yaw_RPY_2 = atan2((sin(roll_RPY_2)*noap.a(1)-cos(roll_RPY_2)*noap.a(2)) , (-sin(roll_RPY_2)*noap.o(1)+cos(roll_RPY_2)*noap.o(2)));%�Q��phi�p��psi
        disp([round(noap.t(1),3), round(noap.t(2),3),round(noap.t(3),3),...
            round(roll_RPY_2,3),round(pitch_RPY_2,3),round(yaw_RPY_2,3)]);
      %% �p��Euler��theta'phi'psi  ������ո�
        roll_Euler_1 = atan2(noap.a(2),noap.a(1));%phi���Ĥ@�ո�
        pitch_Euler_1 = atan2( cos(roll_Euler_1)*noap.a(1)+sin(roll_Euler_1)*noap.a(2) , noap.a(3));%�Q��phi�p��theta
        yaw_Euler_1 = atan2((-sin(roll_Euler_1)*noap.n(1)+cos(roll_Euler_1)*noap.n(2)) , (-sin(roll_Euler_1)*noap.o(1)+cos(roll_Euler_1)*noap.o(2)));%�Q��phi�p��psi
        fprintf('For Euler, (x, y, z, �X, �c, �r)=\n')
        disp([round(noap.t(1),3), round(noap.t(2),3),round(noap.t(3),3),...
            round(roll_Euler_1,3),round(pitch_Euler_1,3),round(yaw_Euler_1,3)]);
        fprintf('        or      \n')
        roll_Euler_2 = atan2(-noap.a(2),-noap.a(1));%phi���ĤG�ո�
        pitch_Euler_2 = atan2( cos(roll_Euler_2)*noap.a(1)+sin(roll_Euler_2)*noap.a(2) , noap.a(3));%�Q��phi�p��theta
        yaw_Euler_2 = atan2((-sin(roll_Euler_2)*noap.n(1)+cos(roll_Euler_2)*noap.n(2)) , (-sin(roll_Euler_2)*noap.o(1)+cos(roll_Euler_2)*noap.o(2)));%�Q��phi�p��psi
        disp([round(noap.t(1),3), round(noap.t(2),3),round(noap.t(3),3),...
            round(roll_Euler_2,3),round(pitch_Euler_2,3),round(yaw_Euler_2,3)]);
        %% 
        pause(0.5)
    elseif strcmp(select,'END') %�����{��
        disp("Ending program!")
        break
    else %������J���~
        disp("Please type 'P1' or 'P2' or 'END' ")
        
    end
end

