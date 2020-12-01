close all
clear all

initparam=[0,0,0,0];

 clc;
 clear;
 close all

 
  
  L(1) = Link('d', 6, 'a', 12, 'alpha', pi);
  L(2) = Link('d', 0, 'a', 15, 'alpha', 0);
  L(3) = Link('theta', 0, 'a', 0, 'alpha', 0);
  L(3).qlim = [0,10];
  L(4) = Link('d', 1, 'a', 0, 'alpha', 0);
  
  %T01 = getTransformMatrix(theta1,d1,0.4,180);
% T12 = getTransformMatrix(theta2,d2,0.15,0);
% T23 = getTransformMatrix(0,d3,0,0);
% T34 = getTransformMatrix(theta4,d4,0,0);

%d1=0.6;
% d2=0;
% d4=0.1;


 Rob = SerialLink (L);
 Rob.name = 'L Arm';
 
  Rob.fkine([0,0,40,0])
 
 
 Scaramove(Rob,[0,0,40,0]);
 
 function Scaramove(Rob,finalparam)




initparam=[0,0,0,0];

i=1;

while(i==1)
    i=0;
    %NextCenter=[NextCenter multiplot(initparam,NextCenter)];
    Rob.plot(initparam)
    hold on
    pause(0.001)
    for j=1:4
        if(j==3)
            if(initparam(3)>finalparam(3))
                
                if(abs(initparam(3)-finalparam(3))>0.0000000001)
                    
                initparam(3)=initparam(3)-1;
                i=1;
                end
                
            elseif(initparam(3)<finalparam(3))
               
                if(abs(initparam(3)-finalparam(3))>0.0000000001)
                    
                initparam(3)=initparam(3)+1;
                i=1;
                end
            end
            
        else
             if(initparam(j)>finalparam(j))
                
                if(abs(initparam(j)-finalparam(j))>0.1)
                    
                initparam(j)=initparam(j)-0.1;
                i=1;
                end
                
            elseif(initparam(j)<finalparam(j))
               
                if(abs(initparam(j)-finalparam(j))>0.1)
                    
                initparam(j)=initparam(j)+0.1;
                i=1;
                end
            end
            
        end
            
            
        
        
    end
end
hold off

end
 
 


% NextCenter=[0.55;0;0.5];
% 
% Scaramove([45,45,0.4,0]);
% 
% [T00,T01,T12,T23,T34,Etip] =  forwardKinematics(0,0,0,0);
% 
% 
% function Scaramove(finalparam)
% 
% figure
% plot3(0,0,0)
% NextCenter=[0.55;0;0.5];
% initparam=[0,0,0,0];
% i=1;
% 
% while(i==1)
%     i=0;
%     NextCenter=[NextCenter multiplot(initparam,NextCenter)];
%     pause(0.01)
%     for j=1:4
%         if(j==3)
%             if(initparam(3)>finalparam(3))
%                 
%                 if(abs(initparam(3)-finalparam(3))>0.0000000001)
%                     
%                 initparam(3)=initparam(3)-0.01;
%                 i=1;
%                 end
%                 
%             elseif(initparam(3)<finalparam(3))
%                
%                 if(abs(initparam(3)-finalparam(3))>0.0000000001)
%                     
%                 initparam(3)=initparam(3)+0.01;
%                 i=1;
%                 end
%             end
%             
%         else
%             if(initparam(j)>finalparam(j))
%                 initparam(j)=initparam(j)-1;
%                 i=1;
%                 
%             elseif(initparam(j)<finalparam(j))
%                 initparam(j)=initparam(j)+1;
%                 i=1;
%             end
%             
%         end
%             
%             
%         
%         
%     end
% end
% 
% 
% end
% 
% 
% function [T00,T01,T12,T23,T34,Etip] =  forwardKinematics(theta1,theta2,d3,theta4)
% 
% d1=0.6;
% d2=0;
% d4=0.1;
% 
% 
% T00 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
% T01 = getTransformMatrix(theta1,d1,0.4,180);
% T12 = getTransformMatrix(theta2,d2,0.15,0);
% T23 = getTransformMatrix(0,d3,0,0);
% T34 = getTransformMatrix(theta4,d4,0,0);
% 
% 
% Etip = T00 * T01 * T12 * T23 * T34;
% 
% end
% 
% function [T] = getTransformMatrix(theta, d, a, alpha)
% T = [cosd(theta) -sind(theta) * cosd(alpha) sind(theta) * sind(alpha) a * cosd(theta);...
%      sind(theta) cosd(theta) * cosd(alpha)  -cosd(theta) * sind(alpha) a * sind(theta);...
%      0,sind(alpha),cosd(alpha),d;...
%      0,0,0,1];
% end
% 
% function nextCenter=oneplot(prevcenter,T)
% nextCenter=T(1:end-1,end);
% 
% Centers=[prevcenter nextCenter];
% 
% x=Centers(1,:);
% y=Centers(2,:);
% z=Centers(3,:);
% hold on
% 
% plot3(x,y,z,'k','LineWidth',2);
% 
% xa=nextCenter+T(1:end-1,1)./10;
% xcenter=[nextCenter xa];
% xa=xcenter(1,:);
% ya=xcenter(2,:);
% za=xcenter(3,:);
% plot3(xa,ya,za,'r','LineWidth',2)
% 
% ya=nextCenter+T(1:end-1,2)./10;
% ycenter=[nextCenter ya];
% xa=ycenter(1,:);
% ya=ycenter(2,:);
% za=ycenter(3,:);
% plot3(xa,ya,za,'g','LineWidth',2)
% 
% za=nextCenter+T(1:end-1,3)./10;
% zcenter=[nextCenter za];
% xa=zcenter(1,:);
% ya=zcenter(2,:);
% za=zcenter(3,:);
% plot3(xa,ya,za,'b','LineWidth',3)
% 
% 
% 
% hold off
% 
% 
% 
% end
% 
% 
% function nextCenter= multiplot(param,points)
% [T00,T01,T12,T23,T34,Etip] =  forwardKinematics(param(1),param(2),param(3),param(4));
% plot3(0,0,0)
% hold on
% plot3(points(1,:),points(2,:),points(3,:))
% hold off
% nextCenter=oneplot([0;0;0],T00);
% nextCenter=oneplot([0;0;0],T01);
% nextCenter=oneplot(nextCenter,T01*T12);
% 
% nextCenter=oneplot(nextCenter,T01*T12*T23);
% 
% nextCenter=oneplot(nextCenter,T01*T12*T23*T34);
% 
% 
% 
% axis 'equal'
% x0=0;
% y0=0;
% width=800;
% height=800;
% set(gcf,'position',[x0,y0,width,height])
% 
% 
% 
% end
% 
% function [T] = initplot(initparam)
% [T00,T01,T12,T23,T34,T45,T56,Etip] =  forwardKinematics(initparam(1),initparam(2),initparam(3),initparam(4));
% plot3(0,0,0)
% 
% nextCenter=oneplot([0;0;0],T00);
% nextCenter=oneplot([0;0;0],T01);
% nextCenter=oneplot(nextCenter,T01*T12);
% 
% nextCenter=oneplot(nextCenter,T01*T12*T23);
% 
% nextCenter=oneplot(nextCenter,T01*T12*T23*T34);
% 
% 
% 
% axis 'equal'
% x0=0;
% y0=0;
% width=800;
% height=800;
% set(gcf,'position',[x0,y0,width,height])
% 
% 
% 
% end