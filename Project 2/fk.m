function [px, py, pz, e1,e2, e3,ax,ay,az]=fk(theta_test,d,alpha,a,euler)
A1 = math(1,theta_test,d,alpha,a);     A2 = math(2,theta_test,d,alpha,a);
A3 = math(3,theta_test,d,alpha,a);     A4 = math(4,theta_test,d,alpha,a); 
A5 = math(5,theta_test,d,alpha,a);     A6 = math(6,theta_test,d,alpha,a);
%fprintf('%~~~~~~~~~~~~~~~ forward kinematic ~~~~~~~~~~~~~~~\n ')                  
%fprintf('-----Cartesian Point : ( n, o, a, p )-----\n ')
T6=A1*A2*A3*A4*A5*A6;
%fprintf('    (   x   ,    y   ,   z   ,   Φ   ,   θ   ,   Ψ   )\n ');
px = T6(1,4); py =  T6(2,4); pz =  T6(3,4);
nx = T6(1,1); ny = T6(2,1);    
ox = T6(1,2); oy = T6(2,2);
ax = T6(1,3); ay = T6(2,3); az = T6(3,3); 
    if ((ax - 0 <0.000001) && (ay - 0 <0.000001))        %如果偏差過小，則視為0
        euler(1,1) = 0;
    else
    	euler(1,1) = atan2(ay, ax);                      %算出phi
        euler(2,1) = euler(1,1)+180;                     %考慮兩角度
    end
    for i=1:2
        sinPhi = sin(euler(i,1));
		cosPhi = cos(euler(i,1));
		euler(i,2) = atan2(cosPhi*ax + sinPhi*ay,az);   %算出theta
		euler(i,3) = atan2((-sinPhi)*nx + cosPhi*ny,(-sinPhi)*ox + cosPhi*oy); %算出psi
    end
    e1=euler(1,1);e2=euler(1,2);e3=euler(1,3);%回傳值設定
	
end
function [A]=math(i,theta,d,alpha,a)
    A = [cosd(theta(i)) , -sind(theta(i))*cosd(alpha(i))   ,  sind(theta(i))*sind(alpha(i))  , a(i)*cosd(theta(i));
         sind(theta(i)) ,  cosd(theta(i))*cosd(alpha(i))   , -cosd(theta(i))*sind(alpha(i))  , a(i)*sind(theta(i));
            0          ,     sind(alpha(i))          ,       cosd(alpha(i))               ,      d(i)     ;
            0          ,           0                 ,            0                       ,        1      ];
end