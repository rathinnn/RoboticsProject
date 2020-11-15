close all
clear all

initparam=[0,90,0.2,0,180,0];


figure
plot3(0,0,0)

multiplot(initparam)





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
T = [cosd(theta) -sind(theta) * cosd(alpha) sind(theta) * sind(alpha) a
 sind(theta) cosd(theta) * cosd(alpha) -cosd(theta) * sind(alpha) a
 0,sind(alpha),cosd(alpha),d;
 0,0,0,1];
end

function nextCenter=oneplot(prevcenter,T)
    nextCenter=T(1:end-1,end);
    
    Centers=[prevcenter nextCenter];
    
    x=Centers(1,:)
    y=Centers(2,:)
    z=Centers(3,:)
    hold on
    
    plot3(x,y,z,'k','LineWidth',3);
    
    xa=nextCenter+T(1:end-1,1)./10;
    xcenter=[nextCenter xa];
    xa=xcenter(1,:);
    ya=xcenter(2,:);
    za=xcenter(3,:);
    plot3(xa,ya,za,'r','LineWidth',3)
    
    ya=nextCenter+T(1:end-1,2)./10;
    ycenter=[nextCenter ya];
    xa=ycenter(1,:);
    ya=ycenter(2,:);
    za=ycenter(3,:);
    plot3(xa,ya,za,'g','LineWidth',3)
    
    za=nextCenter+T(1:end-1,3)./10;
    zcenter=[nextCenter za];
    xa=zcenter(1,:);
    ya=zcenter(2,:);
    za=zcenter(3,:);
    plot3(xa,ya,za,'b','LineWidth',4)
    
    
    
    hold off
    
    
    

end


function [T] = multiplot(param)
[T00,T01,T12,T23,T34,T45,T56,Etip] =  forwardKinematics(param(1),param(2),param(3),param(4),param(5),param(6));
nextCenter=oneplot([0;0;0],T00)
nextCenter=oneplot([0;0;0],T01)
nextCenter=oneplot(nextCenter,T01*T12)

nextCenter=oneplot(nextCenter,T01*T12*T23)

nextCenter=oneplot(nextCenter,T01*T12*T23*T34)

nextCenter=oneplot(nextCenter,T01*T12*T23*T34*T45)


nextCenter=oneplot(nextCenter,T01*T12*T23*T34*T45*T56)

axis 'equal'
end