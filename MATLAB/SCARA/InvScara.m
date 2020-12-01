close all
clear
dhparams = [12   pi	6 	0;
            15	0   0  0;
            0	0	0	0;
            0   0	1	0;
            ];
        
robot = rigidBodyTree;
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;

addBody(robot,body1,'base')


body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','prismatic');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');


setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');


body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;


addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')


showdetails(robot)
%show(robot);
% randConfig = robot.randomConfiguration;
% %tform = getTransform(robot,randConfig,'L6','base');
% 
% %show(robot,randConfig);
% 
% ik = inverseKinematics('RigidBodyTree',robot);
% weights = [0 0 0 6 6 0];
% initialguess = robot.homeConfiguration;
% initialguess(1).JointPosition = 0;
% initialguess(2).JointPosition = 0;
% initialguess(3).JointPosition = 0;
% initialguess(4).JointPosition = 0;
% 
% tform = getTransform(robot,randConfig,'body4','base');
% 
% 
% x1=-0.85;
% y1=0.4;
% z1=0.76;
% 
% Etv=[ 
%          0.7071         0.7071         0   10;
%          0.7071         -0.7071        0    0;
%          0         0         -1   0;
%          0         0         0    1
% ];
% 
% [configSoln,solnInfo] = ik('body4',Etv,weights,initialguess);
% vals = [configSoln(:).JointPosition];
% 
% 
% param=round(vals,2);
% param(3)=round(vals(3));
% %NextCenter=[-0.8654;0.3793;0.7620];
% 
% Scaramove(Rob,param);





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
 Rob.name = 'Scara';
 
 
 g=randperm(15);
 x=12+g(1:4);
 y=zeros(1,4);
 z=zeros(1,4);
 h=figure;
%  for i=1:4
%      ymin=0;
%      ymax=27^2-x(1,i)^2;
%      ymax=sqrt(ymax);
%      ymax=fix(ymax);
%      g=randperm(ymax-ymin);
%      y(1,i)=ymin+g(1);
%      z(1,i)=-35;
%      
%  end
%  figure
 
 
 x=[12 20 7 11];
 y=[-14 18 -26 21];
 z=[-35 -35 -35 -35];
 demo(robot,Rob,x,y,z,-27,0,h)
 
 
 function demo(robot,Rob,x,y,z,destx,desty,h)
 
 initparam=[0,0,0,0];
 scatter3(x,y,z,65,'filled')
 hold on
 scatter3(-27,0,-35,100,'k','filled')
 hold off
 donex=[];
 doney=[];
 donez=[];
 check=true;
 for i= 1:4
     initparam=reach(initparam,robot,Rob,x(1,i),y(1,i),h,check)
     check=false;
     donex(i)=x(1,i);
     doney(i)=y(1,i);
     donez(i)=-35;
     hold on
     scatter3(donex,doney,donez,45,'red','filled')
     hold off
     initparam=reach(initparam,robot,Rob,destx,desty,h,false)
     
     
 end
 end
 

 
 
 %reach(robot,Rob,12,14)
 
 
 function initparam=reach(initparam,robot,Rob,x,y,h,write)
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0 0 0 1 1 0];
initialguess = robot.homeConfiguration;
initialguess(1).JointPosition = 0;
initialguess(2).JointPosition = 0;
initialguess(3).JointPosition = 0;
initialguess(4).JointPosition = 0;

Etv=[ 
         0.7071         0.7071         0   x;
         0.7071         -0.7071        0    y;
         0         0         -1   0;
         0         0         0    1
];

[configSoln,solnInfo] = ik('body4',Etv,weights,initialguess);
vals = [configSoln(:).JointPosition];


param=round(vals,5);
param(3)=15;
%NextCenter=[-0.8654;0.3793;0.7620];

initparam=Scaramove(initparam,Rob,param,h,write);

param(3)=40;
initparam=Scaramove(initparam,Rob,param,h,false);

param(3)=15;
initparam=Scaramove(initparam,Rob,param,h,false);

%Rob.fkine(param)

 end


 
 function initparam= Scaramove(initparam,Rob,finalparam,h,write)



filename = 'output1.gif';


i=1;
n=1;
while(i==1)
    i=0;
    %NextCenter=[NextCenter multiplot(initparam,NextCenter)];
    hold on
    Rob.plot(initparam)
    frame = getframe(h); 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if write
          imwrite(imind,cm,filename,'gif', 'DelayTime',0.1,'Loopcount',inf); 
          write=false;
      else 
          imwrite(imind,cm,filename,'gif','DelayTime',0.1,'WriteMode','append'); 
      end 
    
    pause(0.0001)
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
                 
                if(abs(initparam(j)-finalparam(j))>0.05)
                    
                initparam(j)=initparam(j)-0.05;
                i=1;
                
                
             elseif(abs(initparam(j)-finalparam(j))>0.001)
                    
                initparam(j)=initparam(j)-0.001;
                i=1;
                end
                
            elseif(initparam(j)<finalparam(j))
                if(abs(initparam(j)-finalparam(j))>0.05)
                    
                initparam(j)=initparam(j)+0.05;
                i=1;
                
               
            elseif(abs(initparam(j)-finalparam(j))>0.001)
                    
                initparam(j)=initparam(j)+0.001;
                i=1;
                end
            end
            
        end
            
            
        
        
    end
end
hold off

end