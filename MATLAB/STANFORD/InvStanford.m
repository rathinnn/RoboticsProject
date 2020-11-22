dhparams = [0   	-pi/2	0.762 	0;
            0	-pi/2       0.394312       0;
            0	0	0	-pi/2;
            0   	-pi/2	0.2268	0;
            0       -pi/2	0   	0;
            0       0       0.4318     0];
        
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
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');

setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');
setFixedTransform(jnt6,dhparams(6,:),'dh');

body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;

addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')
addBody(robot,body6,'body5')

showdetails(robot)
%show(robot);
randConfig = robot.randomConfiguration;
%tform = getTransform(robot,randConfig,'L6','base');

show(robot,randConfig);

ik = inverseKinematics('RigidBodyTree',robot);
weights = [1 1 1 1 1 1];
initialguess = robot.homeConfiguration;
initialguess(1).JointPosition = 0;
initialguess(2).JointPosition = pi/2;
initialguess(3).JointPosition = 0.2;
initialguess(4).JointPosition = 0;
initialguess(5).JointPosition = pi;
initialguess(6).JointPosition = 0;
tform = getTransform(robot,randConfig,'body6','base');


x1=-0.85;
y1=0.4;
z1=0.76;

Etv=[ 
         0         0   -1.0000   x1;
   -1.0000         0         0    y1;
         0    1.0000         0    z1;
         0         0         0    1.0000
];

[configSoln,solnInfo] = ik('body6',Etv,weights,initialguess);
vals = [configSoln(:).JointPosition];
store=rad2deg(vals);

param=round(store);
param(3)=round(vals(3),2);
%NextCenter=[-0.8654;0.3793;0.7620];
NextCenter=[0;0.3943;0.7967];
initparam=[0,0,0,0,0,0];
NextCenter=Stanfordmove(initparam,param,NextCenter);
param2=param;

x1=-0.85;
y1=0.2;
z1=0.76;

Etv=[ 
         0         0   -1.0000   x1;
   -1.0000         0         0    y1;
         0    1.0000         0    z1;
         0         0         0    1.0000
];
weights = [0.1 0.1 0.1 6 0.01 6];
[configSoln,solnInfo] = ik('body6',Etv,weights,initialguess);
vals = [configSoln(:).JointPosition];
store=rad2deg(vals);

param=round(store);
param(3)=round(vals(3),2);

NextCenter=Stanfordmove(param2,param,NextCenter);
 


































function NextCenter = Stanfordmove(initparam,finalparam,NextCenter)


plot3(0,0,0)
grid on

%initparam=[0,90,0.2,0,180,0];

i=1;

while(i==1)
    i=0;
    NextCenter=[NextCenter multiplot(initparam,NextCenter)];
    pause(0.01)
    for j=1:6
        if(j==3)
            if(initparam(3)>finalparam(3))
                if(abs(initparam(3)-finalparam(3))>0.0000000001)
                    
                initparam(3)=initparam(3)-0.01;
                i=1;
                end
                
            elseif(initparam(3)<finalparam(3))
                 if(abs(initparam(3)-finalparam(3))>0.0000000001)
                    
                initparam(3)=initparam(3)+0.01;
                i=1;
                end
            end
            
        else
            if(initparam(j)>finalparam(j))
                initparam(j)=initparam(j)-1;
                i=1;
                
            elseif(initparam(j)<finalparam(j))
                initparam(j)=initparam(j)+1;
                i=1;
            end
            
        end
            
            
        
        
    end
end


end


function [T00,T01,T12,T23,T34,T45,T56,Etip] =  forwardKinematics(theta1,theta2,d3,theta4,theta5,theta6)

d1=0.762;
d2=0.394312;
d4=0.2268;
d6=0.4318;
T00 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
T01 = getTransformMatrix(theta1,d1,0,-90);
T12 = getTransformMatrix(theta2,d2,0,-90);
T23 = getTransformMatrix(-90,d3,0,0);
T34 = getTransformMatrix(theta4,d4,0,-90);
T45 = getTransformMatrix(theta5,0,0,-90);
T56 = getTransformMatrix(theta6,d6,0,0);

Etip = T00 * T01 * T12 * T23 * T34 * T45 * T56;

end

function [T] = getTransformMatrix(theta, d, a, alpha)
T = [cosd(theta) -sind(theta) * cosd(alpha) sind(theta) * sind(alpha) a * cosd(theta);...
     sind(theta) cosd(theta) * cosd(alpha)  -cosd(theta) * sind(alpha) a * sind(theta);...
     0,sind(alpha),cosd(alpha),d;...
     0,0,0,1];
end

function nextCenter=oneplot(prevcenter,T)
nextCenter=T(1:end-1,end);

Centers=[prevcenter nextCenter];

x=Centers(1,:);
y=Centers(2,:);
z=Centers(3,:);
hold on

plot3(x,y,z,'k','LineWidth',2);

xa=nextCenter+T(1:end-1,1)./10;
xcenter=[nextCenter xa];
xa=xcenter(1,:);
ya=xcenter(2,:);
za=xcenter(3,:);
plot3(xa,ya,za,'r','LineWidth',2)

ya=nextCenter+T(1:end-1,2)./10;
ycenter=[nextCenter ya];
xa=ycenter(1,:);
ya=ycenter(2,:);
za=ycenter(3,:);
plot3(xa,ya,za,'g','LineWidth',2)

za=nextCenter+T(1:end-1,3)./10;
zcenter=[nextCenter za];
xa=zcenter(1,:);
ya=zcenter(2,:);
za=zcenter(3,:);
plot3(xa,ya,za,'b','LineWidth',3)



hold off

grid on

end


function nextCenter= multiplot(param,points)
[T00,T01,T12,T23,T34,T45,T56,Etip] =  forwardKinematics(param(1),param(2),param(3),param(4),param(5),param(6));
plot3(0,0,0)
hold on
plot3(points(1,:),points(2,:),points(3,:))
d=0.36;
%scatter3(d*[3,0,0,3,3,0,3],d*[0,3,0,3,0,3,3],d*[0,0,3,0,3,3,3],'MarkerEdgeAlpha',0)
%scatter3(d*[3,0,0,3,3,0,3],-d*[0,3,0,3,0,3,3],d*[0,0,3,0,3,3,3],'MarkerEdgeAlpha',0)
%scatter3(-d*[3,0,0,3,3,0,3],d*[0,3,0,3,0,3,3],d*[0,0,3,0,3,3,3],'MarkerEdgeAlpha',0)
%scatter3(-d*[3,0,0,3,3,0,3],-d*[0,3,0,3,0,3,3],d*[0,0,3,0,3,3,3],'MarkerEdgeAlpha',0)

hold off
nextCenter=oneplot([0;0;0],T00);
nextCenter=oneplot([0;0;0],T01);
nextCenter=oneplot(nextCenter,T01*T12);

nextCenter=oneplot(nextCenter,T01*T12*T23);

nextCenter=oneplot(nextCenter,T01*T12*T23*T34);

nextCenter=oneplot(nextCenter,T01*T12*T23*T34*T45);


nextCenter=oneplot(nextCenter,T01*T12*T23*T34*T45*T56);

axis 'equal'




end

function [T] = initplot(initparam)
[T00,T01,T12,T23,T34,T45,T56,Etip] =  forwardKinematics(initparam(1),initparam(2),initparam(3),initparam(4),initparam(5),initparam(6));
plot3(0,0,0)

nextCenter=oneplot([0;0;0],T00);
nextCenter=oneplot([0;0;0],T01);
nextCenter=oneplot(nextCenter,T01*T12);

nextCenter=oneplot(nextCenter,T01*T12*T23);

nextCenter=oneplot(nextCenter,T01*T12*T23*T34);

nextCenter=oneplot(nextCenter,T01*T12*T23*T34*T45);


nextCenter=oneplot(nextCenter,T01*T12*T23*T34*T45*T56);

axis 'equal'
x0=0;
y0=0;
width=800;
height=800;
set(gcf,'position',[x0,y0,width,height])



end
