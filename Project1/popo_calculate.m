function result=popo_calculate(T)
    n = T(1:3,1);
    o = T(1:3,2);
    a = T(1:3,3);
    p = T(1:3,4);
   a1 = 0.12;    % condition of a1
   a2 = 0.25;    % condition of a2
   a3 = 0.26;    % condition of a3
   px=p(1);
   py=p(2);
   pz=p(3);
   nx=n(1);
   ny=n(2);
   nz=n(3);
   ox=o(1);
   oy=o(2);
   oz=o(3);
   ax=a(1);
   ay=a(2);
   az=a(3);
   R = 180 / pi;
   P = pi / 180;
%% theta1
    theta1_1=atan2(py,px)*R;
    theta1_2=atan2(-py,-px)*R;
    
   %% theta3    
    c31 = (power(px, 2) + power(py, 2) + power(pz, 2) - 2 * a1*(cos(theta1_1*P)*(px) + sin(theta1_1*P)*(py)) + power(a1, 2) - power(a2, 2) - power(a3, 2)) / (2 * a2*a3);
    c32 = (power(-px, 2) + power(-py, 2) + power(pz, 2) - 2 * a1*(cos(theta1_2*P)*(-px) + sin(theta1_2*P)*(-py)) + power(a1, 2) - power(a2, 2) - power(a3, 2)) / (2 * a2*a3);
    s31 = sqrt(1 - power(c31, 2));
    s32 = -sqrt(1 - power(c31, 2));

    s33 = sqrt(1 - power(c32, 2));
    s34 = -sqrt(1 - power(c32, 2));

    theta3_1 = -atan2(s31, c31)*R;
    theta3_2 = -atan2(s32, c31)*R;

    theta3_3 = -atan2(s33, c32)*R;
    theta3_4 = -atan2(s34, c32)*R;
    
  %% theta2

     c23_1 = ((a2*cos(theta3_1*P) + a3)*(cos(theta1_1*P)*px + sin(theta1_1*P)*py - a1) + a2*sin(theta3_1*P)*pz) / (power(cos(theta1_1*P)*px + sin(theta1_1*P)*py - a1, 2) + power(pz, 2));
 	 c23_2 = ((a2*cos(theta3_2*P) + a3)*(cos(theta1_1*P)*px + sin(theta1_1*P)*py - a1) + a2*sin(theta3_2*P)*pz) / (power(cos(theta1_1*P)*px + sin(theta1_1*P)*py - a1, 2) + power(pz, 2));
     s23_1 = (a2*sin(theta3_1*P)*(cos(theta1_1*P)*px + sin(theta1_1*P)*py - a1) - pz*(a2*cos(theta3_1*P) + a3)) / (power(cos(theta1_1*P)*px + sin(theta1_1*P)*py - a1, 2) + power(pz, 2));
     s23_2 = (a2*sin(theta3_2*P)*(cos(theta1_1*P)*px + sin(theta1_1*P)*py - a1) - pz*(a2*cos(theta3_2*P) + a3)) / (power(cos(theta1_1*P)*px + sin(theta1_1*P)*py - a1, 2) + power(pz, 2));

     c23_3 = ((a2*cos(theta3_3*P) + a3)*(cos(theta1_2*P)*px + sin(theta1_2*P)*py - a1) + a2*sin(theta3_3*P)*pz) / (power(cos(theta1_2*P)*px + sin(theta1_2*P)*py - a1, 2) + power(pz, 2));
     c23_4 = ((a2*cos(theta3_4*P) + a3)*(cos(theta1_2*P)*px + sin(theta1_2*P)*py - a1) + a2*sin(theta3_4*P)*pz) / (power(cos(theta1_2*P)*px + sin(theta1_2*P)*py - a1, 2) + power(pz, 2));
     s23_3 = (a2*sin(theta3_3*P)*(cos(theta1_2*P)*px + sin(theta1_2*P)*py - a1) - pz*(a2*cos(theta3_3*P) + a3)) / (power(cos(theta1_2*P)*px + sin(theta1_2*P)*py - a1, 2) + power(pz, 2));
     s23_4 = (a2*sin(theta3_4*P)*(cos(theta1_2*P)*px + sin(theta1_2*P)*py - a1) - pz*(a2*cos(theta3_4*P) + a3)) / (power(cos(theta1_2*P)*px + sin(theta1_2*P)*py - a1, 2) + power(pz, 2));

     theta23_1 = atan2(s23_1, c23_1)*R;
     theta23_2 = atan2(s23_2, c23_2)*R;
     theta23_3 = atan2(s23_3, c23_3)*R;
     theta23_4 = atan2(s23_4, c23_4)*R;

 	 theta2_1 = theta23_1 - theta3_1; 
     theta2_2 = theta23_2 - theta3_2; 
     theta2_3 = theta23_3 - theta3_3; 
     theta2_4 = theta23_4 - theta3_4; 
    
   %% theta4

     theta4_1 = atan2(-cos(theta1_1*P)*s23_1*ax - sin(theta1_1*P)*s23_1*ay - c23_1*az, cos(theta1_1*P)*c23_1*ax + sin(theta1_1*P)*c23_1*ay - s23_1*az)*R;
     theta4_2 = atan2(-cos(theta1_1*P)*s23_2*ax - sin(theta1_1*P)*s23_2*ay - c23_2*az, cos(theta1_1*P)*c23_2*ax + sin(theta1_1*P)*c23_2*ay - s23_2*az)*R; 
     theta4_3 = atan2(-(-cos(theta1_1*P)*s23_1*ax - sin(theta1_1*P)*s23_1*ay - c23_1*az), -(cos(theta1_1*P)*c23_1*ax + sin(theta1_1*P)*c23_1*ay - s23_1*az))*R;
     theta4_4 = atan2(-(-cos(theta1_1*P)*s23_2*ax - sin(theta1_1*P)*s23_2*ay - c23_2*az), -(cos(theta1_1*P)*c23_2*ax + sin(theta1_1*P)*c23_2*ay - s23_2*az))*R; 
     theta4_5 = atan2(-cos(theta1_2*P)*s23_3*ax - sin(theta1_2*P)*s23_3*ay - c23_3*az, cos(theta1_2*P)*c23_3*ax + sin(theta1_2*P)*c23_3*ay - s23_3*az)*R;
     theta4_6 = atan2(-cos(theta1_2*P)*s23_4*ax - sin(theta1_2*P)*s23_4*ay - c23_4*az, cos(theta1_2*P)*c23_4*ax + sin(theta1_2*P)*c23_4*ay - s23_4*az)*R;
     theta4_7 = atan2(-(-cos(theta1_2*P)*s23_3*ax - sin(theta1_2*P)*s23_3*ay - c23_3*az), -(cos(theta1_2*P)*c23_3*ax + sin(theta1_2*P)*c23_3*ay - s23_3*az))*R;
     theta4_8 = atan2(-(-cos(theta1_2*P)*s23_4*ax - sin(theta1_2*P)*s23_4*ay - c23_4*az), -(cos(theta1_2*P)*c23_4*ax + sin(theta1_2*P)*c23_4*ay - s23_4*az))*R;


   %% theta5
     s5_1 = (cos(theta1_1*P)*cos(theta4_1*P)*c23_1- cos(theta1_1*P)*sin(theta4_1*P)*s23_1)*ax + (sin(theta1_1*P)*cos(theta4_1*P)*c23_1 - sin(theta1_1*P)*sin(theta4_1*P)*s23_1)*ay - (cos(theta4_1*P)*s23_1 + sin(theta4_1*P)*c23_1)*az;
     s5_2 = (cos(theta1_1*P)*cos(theta4_2*P)*c23_2- cos(theta1_1*P)*sin(theta4_2*P)*s23_2)*ax + (sin(theta1_1*P)*cos(theta4_2*P)*c23_2 - sin(theta1_1*P)*sin(theta4_2*P)*s23_2)*ay - (cos(theta4_2*P)*s23_2 + sin(theta4_2*P)*c23_2)*az;
     s5_3 = (cos(theta1_1*P)*cos(theta4_3*P)*c23_1- cos(theta1_1*P)*sin(theta4_3*P)*s23_1)*ax + (sin(theta1_1*P)*cos(theta4_3*P)*c23_1 - sin(theta1_1*P)*sin(theta4_3*P)*s23_1)*ay - (cos(theta4_3*P)*s23_1 + sin(theta4_3*P)*c23_1)*az;
     s5_4 = (cos(theta1_1*P)*cos(theta4_4*P)*c23_2- cos(theta1_1*P)*sin(theta4_4*P)*s23_2)*ax + (sin(theta1_1*P)*cos(theta4_4*P)*c23_2 - sin(theta1_1*P)*sin(theta4_4*P)*s23_2)*ay - (cos(theta4_4*P)*s23_2 + sin(theta4_4*P)*c23_2)*az;    
     s5_5 = (cos(theta1_2*P)*cos(theta4_5*P)*c23_3- cos(theta1_2*P)*sin(theta4_5*P)*s23_3)*ax + (sin(theta1_2*P)*cos(theta4_5*P)*c23_3 - sin(theta1_2*P)*sin(theta4_5*P)*s23_3)*ay - (cos(theta4_5*P)*s23_3 + sin(theta4_5*P)*c23_3)*az;
     s5_6 = (cos(theta1_2*P)*cos(theta4_6*P)*c23_4- cos(theta1_2*P)*sin(theta4_6*P)*s23_4)*ax + (sin(theta1_2*P)*cos(theta4_6*P)*c23_4 - sin(theta1_2*P)*sin(theta4_6*P)*s23_4)*ay - (cos(theta4_6*P)*s23_4 + sin(theta4_6*P)*c23_4)*az;
     s5_7 = (cos(theta1_2*P)*cos(theta4_7*P)*c23_3- cos(theta1_2*P)*sin(theta4_7*P)*s23_3)*ax + (sin(theta1_2*P)*cos(theta4_7*P)*c23_3 - sin(theta1_2*P)*sin(theta4_7*P)*s23_3)*ay - (cos(theta4_7*P)*s23_3 + sin(theta4_7*P)*c23_3)*az;
     s5_8 = (cos(theta1_2*P)*cos(theta4_8*P)*c23_4- cos(theta1_2*P)*sin(theta4_8*P)*s23_4)*ax + (sin(theta1_2*P)*cos(theta4_8*P)*c23_4 - sin(theta1_2*P)*sin(theta4_8*P)*s23_4)*ay - (cos(theta4_8*P)*s23_4 + sin(theta4_8*P)*c23_4)*az;

     c5_1 = cos(theta1_1*P)*ay - sin(theta1_1*P)*ax;
     c5_2 = cos(theta1_2*P)*ay - sin(theta1_2*P)*ax;
     
  	 theta5_1 = atan2(s5_1, c5_1)*R;
     theta5_2 = atan2(s5_2, c5_1)*R;
     
     theta5_3 = atan2(s5_3, c5_1)*R;
     theta5_4 = atan2(s5_4, c5_1)*R;
     theta5_5 = atan2(s5_5, c5_2)*R;
     theta5_6 = atan2(s5_6, c5_2)*R;
     theta5_7 = atan2(s5_7, c5_2)*R;
     theta5_8 = atan2(s5_8, c5_2)*R;
     
    
    %% theta6
     theta6_1 = atan2(-sin(theta1_1*P)*ox + cos(theta1_1*P)*oy, sin(theta1_1*P)*nx - cos(theta1_1*P)*ny)*R;
     theta6_2 = atan2(-sin(theta1_2*P)*ox + cos(theta1_2*P)*oy, sin(theta1_2*P)*nx - cos(theta1_2*P)*ny)*R;
     
     theta6_3 = atan2(-(-sin(theta1_1*P)*ox + cos(theta1_1*P)*oy),-( sin(theta1_1*P)*nx - cos(theta1_1*P)*ny))*R;
     theta6_4 = atan2(-(-sin(theta1_2*P)*ox + cos(theta1_2*P)*oy), -(sin(theta1_2*P)*nx - cos(theta1_2*P)*ny))*R;
     %% result
     result = [[theta1_1 theta2_1 theta3_1 theta4_1 theta5_1 theta6_1];...
               [theta1_1 theta2_1 theta3_1 theta4_3 theta5_3 theta6_2];...
               [theta1_1 theta2_2 theta3_2 theta4_2 theta5_2 theta6_1];...
               [theta1_1 theta2_2 theta3_2 theta4_4 theta5_4 theta6_2];...
               [theta1_2 theta2_3 theta3_3 theta4_5 theta5_5 theta6_3];...
               [theta1_2 theta2_3 theta3_3 theta4_7 theta5_7 theta6_4];...
               [theta1_2 theta2_4 theta3_4 theta4_6 theta5_6 theta6_3];...
               [theta1_2 theta2_4 theta3_4 theta4_8 theta5_8 theta6_4];];
end