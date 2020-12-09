close all
clc
%clear all

initparam=[0,90,0.2,0,180,0];
%initparam=[0,0,0,0,0,0];

x = input('Enter number from 0 to 9');
draw(x)

function draw(g)
initparam=[0,90,0.2,0,180,0];
if(g==1)
    
    
    NextCenter=[-0.8654;0.3793;0.7620];
    param=[2,90,0.2,0,180,0];
    NextCenter=Stanfordmove(initparam,param,NextCenter);
    
    param2=[-2,90,0.2,0,180,0];
    NextCenter=Stanfordmove(param,param2,NextCenter);
    
    param3=[0,90,0.2,0,180,0];
    NextCenter=Stanfordmove(param2,param3,NextCenter);
    
    param4=[0,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param3,param4,NextCenter);
    
    param5=[-2,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param4,param5,NextCenter);
    
    param6=[-3,98,0.2,0,180,0];
    NextCenter=Stanfordmove(param5,param6,NextCenter);
    
elseif g==2
    
    
    NextCenter=[-0.8654;0.3793;0.7620];
    param=[-10,90,0.2,0,180,0];
    NextCenter=Stanfordmove(initparam,param,NextCenter);
    
    param2=[-10,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param,param2,NextCenter);
    
    param3=[0,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param2,param3,NextCenter);
    
    param4=[0,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param3,param4,NextCenter);
    
    param5=[-10,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param4,param5,NextCenter);
    
elseif g==5
    
    NextCenter=[-0.8654;0.3793;0.7620];
    param=[10,90,0.2,0,180,0];
    NextCenter=Stanfordmove(initparam,param,NextCenter);
    
    param2=[10,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param,param2,NextCenter);
    
    param3=[0,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param2,param3,NextCenter);
    
    param4=[0,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param3,param4,NextCenter);
    
    param5=[10,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param4,param5,NextCenter);
    
elseif g==3
    
    
    NextCenter=[-0.8654;0.3793;0.7620];
    param=[10,90,0.2,0,180,0];
    NextCenter=Stanfordmove(initparam,param,NextCenter);
    
    param2=[10,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param,param2,NextCenter);
    
    
    param3=[0,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param2,param3,NextCenter);
    
    param4=[10,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param3,param4,NextCenter);
    
    param5=[10,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param4,param5,NextCenter);
    
    param6=[0,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param5,param6,NextCenter);
    
    
elseif g==4
    
    NextCenter=[-0.8654;0.3793;0.7620];
    param=[10,90,0.2,0,180,0];
    NextCenter=Stanfordmove(initparam,param,NextCenter);
    
    param2=[0,90,0.2,0,180,0];
    NextCenter=Stanfordmove(param,param2,NextCenter);
    
    param3=[0,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param2,param3,NextCenter);
    
    param4=[0,90,0.2,0,180,0];
    NextCenter=Stanfordmove(param3,param4,NextCenter);
    
    param5=[5,90,0.2,0,180,0];
    NextCenter=Stanfordmove(param4,param5,NextCenter);
    
    param6=[5,95,0.2,0,180,0];
    NextCenter=Stanfordmove(param5,param6,NextCenter);
    
    param7=[5,85,0.2,0,180,-90];
    NextCenter=Stanfordmove(param6,param7,NextCenter);
    
    
elseif g==0
    
    NextCenter=[-0.8654;0.3793;0.7620];
    param=[6,90,0.2,0,180,0];
    NextCenter=Stanfordmove(initparam,param,NextCenter);
    
    param2=[8,102,0.2,0,180,0];
    NextCenter=Stanfordmove(param,param2,NextCenter);
    
    param3=[0,104,0.2,0,180,0];
    NextCenter=Stanfordmove(param2,param3,NextCenter);
    
    param4=[-2,92,0.2,0,180,0];
    NextCenter=Stanfordmove(param3,param4,NextCenter);
    
    param5=[0,90,0.2,0,180,0];
    NextCenter=Stanfordmove(param4,param5,NextCenter);
    
    param6=[0,88,0.2,0,180,90];
    NextCenter=Stanfordmove(param5,param6,NextCenter);
    
elseif g==6
    
    
    NextCenter=[-0.8654;0.3793;0.7620];
    param=[0,110,0.2,0,180,0];
    NextCenter=Stanfordmove(initparam,param,NextCenter);
    
    param2=[10,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param,param2,NextCenter);
    
    param3=[0,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param2,param3,NextCenter);
    
    param4=[0,90,0.2,0,180,0];
    NextCenter=Stanfordmove(param3,param4,NextCenter);
    
    param5=[10,90,0.2,0,180,0];
    NextCenter=Stanfordmove(param4,param5,NextCenter);
    
    param6=[10,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param5,param6,NextCenter);
    
    param7=[-1,100,0.2,0,180,90];
    NextCenter=Stanfordmove(param6,param7,NextCenter);
    
elseif g==7
    
    
    
    NextCenter=[-0.8654;0.3793;0.7620];
    param=[0,100,0.2,0,170,0];
    NextCenter=Stanfordmove(initparam,param,NextCenter);
    
    param2=[0,100,0.2,0,180,90];
    NextCenter=Stanfordmove(param,param2,NextCenter);
    
    
elseif g==8
    
    
    NextCenter=[-0.8654;0.3793;0.7620];
    param=[10,90,0.2,0,180,0];
    NextCenter=Stanfordmove(initparam,param,NextCenter);
    
    param2=[10,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param,param2,NextCenter);
    
    
    param3=[0,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param2,param3,NextCenter);
    
    param4=[10,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param3,param4,NextCenter);
    
    param5=[10,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param4,param5,NextCenter);
    
    param6=[0,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param5,param6,NextCenter);
    
    param7=[0,90,0.2,0,180,0];
    NextCenter=Stanfordmove(param6,param7,NextCenter);
    
    
    
elseif g==9
    
    
    NextCenter=[-0.8654;0.3793;0.7620];
    param=[0,110,0.2,0,180,0];
    NextCenter=Stanfordmove(initparam,param,NextCenter);
    
    param2=[-10,110,0.2,0,180,0];
    NextCenter=Stanfordmove(param,param2,NextCenter);
    
    
    param3=[-10,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param2,param3,NextCenter);
    
    param4=[0,100,0.2,0,180,0];
    NextCenter=Stanfordmove(param3,param4,NextCenter);
    
    param5=[0,90,0.2,0,180,0];
    NextCenter=Stanfordmove(param4,param5,NextCenter);
    
    param6=[-10,90,0.2,0,180,-90];
    NextCenter=Stanfordmove(param5,param6,NextCenter);
    
    
    
end

end




%[T00,T01,T12,T23,T34,T45,T56,Etip] =  forwardKinematics(0,90,0.2,0,180,0);



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
plot3(points(1,:),points(2,:),points(3,:),'m','LineWidth',2)
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