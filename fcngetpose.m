% Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

function xhat = fcngetpose(xc,x1,xhat,i,rgb,xyzi,h)
options = optimset('TolX',1E-9,'TolFun',1E-9,'MaxIter',5000,'Display','off','MaxFunEvals',5000,'LargeScale', 'off','GradObj','on');
if i==1
    return
end

%v1=ceil(rand(1E4,1)*size(xc,1));
%delete(get(h(6),'children')); plot3(h(6),xc(v1,1),xc(v1,2),xc(v1,3),'.','markersize',5,'color',[.7 .7 1]); hold(h(5),'on'); hold(h(6),'on') %cumulative

%camvec = fcnRPY2DCM_B2W(xhat(4:6))*[1; 0; 0];
% theta = fcnangle([xc(:,1)-xhat(1), xc(:,2)-xhat(2), xc(:,3)-xhat(3)], camvec');  %fov=53deg horizontal 43deg vertical
% xc = xc(theta<(45*pi/180),:);

%v1=ceil(rand(1E4,1)*size(xc,1));
%plot3(h(6),xc(v1,1),xc(v1,2),xc(v1,3),'.','markersize',5,'color',[0 0 1])
%plot3(h(6),[0 camvec(1)]+xhat(1),[0 camvec(2)]+xhat(2),[0 camvec(3)]+xhat(3),'-k','linewidth',3)


x = zeros(1,6);
fx = zeros(1,1);
for j=1
    [x(j,:), fx(j)] = fcnoptimize(xc,x1,xhat,.75,2E6,options,i);
end
[fx1,k]=min(fx);

xhat = x(k,:);
%fprintf('dispersion=%.3f\n',std(fx)/mean(fx))
end
    

function [xhat,fval] = fcnoptimize(xc,x1,xhat,fraction,nxcp,options,i)
tic
nxc=size(xc,1);  n1=size(x1,1);  nxcp=min(nxc,nxcp); %max historical point to select from past
if i>5
    x = linspace(0,1,100);
    ci = fcnrandcdf(cumsum(exp(-x*6)),x,round(9000/fraction),'linear');  ci=nxc-round(ci*nxcp)-1;
    %[y,x]=hist(ci,30); bar(h(5),x,y,1,'b'); axis(h(5),'on','normal','tight')
else
    ci=uint32(rand(round(9000/fraction),1)*(nxcp-1)+1);
end
xc1=double(xc(ci,:));
ci=uint32(rand(round(3000/fraction),1)*(n1-1)+1);  x11=x1(ci,:);

ns = createns(xc1,'NSMethod','kdtree');
[xhat,fval]=fminunc(@(x) fcncost(xc1,double(x11),x,ns,fraction), double(xhat), options);  fprintf('%9.3f%9.3f%9.3f%9.3f%9.3f%9.3f (%.2fs icp, fx=%.3f)\n',xhat,toc,fval)
end

function [fx, gx] = fcncost(vc0,v10,x,ns,fraction)
dx = 1E-5;
n1=size(v10,1);
v1=v10*fcnRPY2DCM_W2B(x(4:6));  v1=[v1(:,1)+x(1), v1(:,2)+x(2), v1(:,3)+x(3)];

[i, r] = knnsearch(ns,v1,'k',1);
[r, j] = sort(r); %min ranges sorted
a = 1:round(fraction*n1);  j=j(a);
fx = sum(r(a).^2);
vc = vc0(i(j),:);
v1 = v1(j,:);

v10=v10(j,:);
v{1} = [v1(:,1)+dx, v1(:,2), v1(:,3)];
v{2} = [v1(:,1), v1(:,2)+dx, v1(:,3)];
v{3} = [v1(:,1), v1(:,2), v1(:,3)+dx];
vt = v10*fcnRPY2DCM_W2B(x(4:6)+[dx 0 0]);  vt=[vt(:,1)+x(1), vt(:,2)+x(2), vt(:,3)+x(3)];  v{4}=vt;
vt = v10*fcnRPY2DCM_W2B(x(4:6)+[0 dx 0]);  vt=[vt(:,1)+x(1), vt(:,2)+x(2), vt(:,3)+x(3)];  v{5}=vt;
vt = v10*fcnRPY2DCM_W2B(x(4:6)+[0 0 dx]);  vt=[vt(:,1)+x(1), vt(:,2)+x(2), vt(:,3)+x(3)];  v{6}=vt;

fg=zeros(6,1);
for i=1:6
    rs=sum((v{i}-vc).^2,2);
    fg(i) = sum(rs);
end
gx = (fg-fx)./dx;
end

    