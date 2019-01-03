function result=ik(pose,a,lower_limit,upper_limit)
fprintf('%~~~~~~~~~~~~~~~ inverse  kinematic ~~~~~~~~~~~~~~~\n ')
nx = pose(1,1);ox = pose(1,2); ax = pose(1,3);px = pose(1,4);
ny = pose(2,1);oy = pose(2,2); ay = pose(2,3);py = pose(2,4);
nz = pose(3,1);oz = pose(3,2); az = pose(3,3);pz = pose(3,4);
R = sqrt(px*px+py*py+pz*pz -2*a(1)*sqrt(px*px+py*py) +a(1)*a(1));           %��XJ1��J3���t
ans=zeros(8,6);
for i=1:8
    if(i<5)                              %�B�δX��k�p��e�T�Ө���(theta) 
		if(R > a(2) + a(3))              %�Y���ŦX�T���ΡA�h���L
            continue;
        end
		psi = atan2(pz,sqrt(px*px+py*py) - a(1)); %��Xpsi
		phi = acos((a(2)*a(2) + R*R - a(3)*a(3)) / (2*a(2)*R)); 
		theta3 =pi - acos((a(2)*a(2)  +a(3)*a(3) - R*R) / (2*a(2)*a(3)));
		if(psi < 0)
            psi = -psi;	   %�Npsi�լ��Ĥ@�H��
        end
		ans(i,1) = atan2(py,px);	
    else
		if (R > a(2) + a(3))     %�p�G���ŦX�T���Ϋh���L
            continue;
        end
		psi = atan2(pz,sqrt(px*px+py*py) + a(1));
		phi = acos((a(2)*a(2) + R*R - a(3)*a(3)) / (2*a(2)*R));
		theta3 = acos((a(2)*a(2)  +a(3)*a(3) - R*R) / (2*a(2)*a(3)));
        if(psi < 0)
            psi = -psi;	
        end
		ans(i,1) = atan2(py,px)+ pi;	
    end
      if(mod(i-1,4)<2)                %��1 2 3 4 5 6 7 8 -> 1 2 4 5  
        ans(i,2)= psi - phi	;
        ans(i,3) = theta3;				
      else
        ans(i,2) = -(psi - phi);
        ans(i,3) = -theta3;
      end
		if(mod(i,2)==1)             %�� 1 3 5 7 
            s1 = sin(ans(i,1));             %²�ƹB������N��
            c1 = cos(ans(i,1));             %²�ƹB������N��
            s23 = sin(ans(i,2)+ans(i,3));   %²�ƹB������N��
            c23 = cos(ans(i,2)+ans(i,3));   %²�ƹB������N��
            ans(i,4) = atan2(-c1*s23*ax - s1*s23*ay - c23*az , c1*c23*ax + s1*c23*ay - s23*az); %�⨤4
            s4 = sin(ans(i,4));
            ans(i,6) = atan2(-s1*ox + c1*oy, s1*nx - c1*ny); %�⨤6
            ans(i,5) = atan2(-c1*s23*ax - s1*s23*ay - c23*az , s4*(-s1*ax + c1*ay));%�⨤5
            ans(i+1,4) = ans(i,4) - pi;     ans(i+1,5) = -ans(i,5);         ans(i+1,6) = ans(i,6) - pi;
        end    
end
 ans=ans.*(180/pi);                     %��������
 for i=1:8
    % fprintf('Ans ( %d )\n',i)
     for j=1:6
        if ans(i,j)<-180
            ans(i,j)=ans(i,j)+360;    %�ˬd�U���צb+-180�פ�
        elseif ans(i,j)>180
            ans(i,j)=ans(i,j)-360;    %�ˬd�U���צb+-180�פ�
        end
     end
     %disp(ans(i,:))                   %�L�X�ӱƯx�}
     check=1;
     for j=1:6
        if ans(i,j)<lower_limit(j)
            ans(i,j)=ans(i,j)+360;    %�ˬd�U���צblower�w�q��
        elseif ans(i,j)>upper_limit(j)
            ans(i,j)=ans(i,j)-360;    %�ˬd�U���צbupper�w�q��
        end
        
        if(ans(i,j)> upper_limit(j) || ans(i,j) < lower_limit(j))
         %   fprintf("    theta %d is out of range\n",j);
        check=0;                      %�аO�X���X�檺��
        end
     end
     if(check==1)
        result=ans(i,:);         %��X�̦n���Ѩæ^��
         break;
     end
   % fprintf('-------------------------------------\n')
 end
end