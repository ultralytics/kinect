function [h] = fcnplotcam(ha,xhat,C)
r2d = 180/pi;
d2r = pi/180;

r = .5;

ov = ones(1,10);

el = [ov*-21.5, linspace(-21.5, 21.5, 10), ov*21.5, linspace(21.5, -21.5, 10)]*d2r;
az = [linspace(-28.5, 28.5, 10), ov*28.5, linspace(28.5, -28.5, 10), ov*-28.5]*d2r;

x = [fcnSC2CC([ones(40,1)*r el' az']); [0 0 0]; fcnSC2CC([r 21.5*d2r -28.5*d2r]); [0 0 0]; fcnSC2CC([r 21.5*d2r 28.5*d2r]); [0 0 0]; fcnSC2CC([r -21.5*d2r 28.5*d2r])];
x=fcnxform(x,xhat);
h=plot3(ha,x(:,1),x(:,2),x(:,3),'-','color',[0 0 0]); hold(ha,'on')


%plot image
[X,Y] = ndgrid(linspace(-28.5, 28.5, 640), linspace(21.5, -21.5, 480));

x = fcnSC2CCd([ones(640*480,1)*r, reshape(Y,[640*480 1]), reshape(X,[640*480 1])]);
x=fcnxform(x,xhat);
X=reshape(x(:,1),[640 480]);  Y=reshape(x(:,2),[640 480]);  Z=reshape(x(:,3),[640 480]);


%h = [h, surf(ha,X',Y',Z',C,'edgecolor','none')];

h = [h, surf(ha,X',Y',Z',C,'edgecolor','none','facecolor','texture','cdata',C)];

end


