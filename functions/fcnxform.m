function [x2, C] = fcnxform(x,xhat)

C = fcnRPY2DCM_W2B(xhat(4:6));

x1 = x*C;
x2 = [x1(:,1)+xhat(1), x1(:,2)+xhat(2), x1(:,3)+xhat(3)]; %translation

end

