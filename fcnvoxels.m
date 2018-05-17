%function v=fcnvoxels(x1)
nv=400;
nr=size(x1,1);

xmin = min(x1,[],1);
xmax = max(x1,[],1);

x=linspace(xmin(1),xmax(1),nv);
y=linspace(xmin(2),xmax(2),nv);
z=linspace(xmin(3),xmax(3),nv);
[xm,ym,zm] = ndgrid(x,y,z);

xi=zeros(size(x1),'uint16');
xi(:,1) = fcnindex1(x,x1(:,1));
xi(:,2) = fcnindex1(y,x1(:,2));
xi(:,3) = fcnindex1(z,x1(:,3));

isovalue = 1;
v = accumarray(xi, 1, [nv nv nv]);


figure
h=gca;

n = isosurface(xm,ym,zm,v,isovalue);
p1 = patch(n);%,'AmbientStrength',1);
set(p1,'FaceColor',[1 1 1]);%,'EdgeColor','none','FaceAlpha',1)
reducepatch(p1, .8)
isonormals(x,y,z,v,p1) %WTF does this actually do?
lighting phong
material shiny

%mrp = nv + rand*nv;
%light('Position',[0 0 0]);  light('Position',[0 0 mrp-1]);  light('Position',[0 0 mrp+1]);  light('Position',[-5 0 mrp]);  light('Position',[0 -5 mrp]);  light('Position',[5 0 mrp]);  light('Position',[0 5 mrp]); 

p2 = patch(isocaps(xm,ym,zm,v,isovalue)); 
set(p2,'EdgeColor','none','FaceColor','interp','SpecularColorReflectance',0,'SpecularStrength',.3);


hold(h,'on'); axis(h,'equal','vis3d'); box(h,'on'); view(h,-60,89); set(h,'clim',[0.8 4]); set(h,'ydir','reverse','zdir','reverse','ylimmode','manual','zlimmode','manual','xlimmode','manual');
xlabel(h,'x (m)'); ylabel(h,'y (m)'); zlabel(h,'z (m)');