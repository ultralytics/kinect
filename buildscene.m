clc; clear; close;
vidFlag=true;

%Add MATLAB common functions to MATLAB path!
x = mfilename('fullpath');  path1=x(1:find(x==filesep,1,'last'));
addpath([path1 'functions'])
addpath([path1 'matfiles'])
load kinect_single.mat

sf=1; %starting frame
ef=size(xyz,4);
xyz = xyz(:,:,:,sf:ef);
rgb = rgb(:,:,:,sf:ef);

ni = size(xyz,4);
xyzv = reshape(xyz,[480*640 3 ni]);  clear xyz
%rgbv = double(reshape(rgb,[480*640 3 ni]))/255;

[h0, hf]=fig(2,3); 
set(hf,'Units','pixels','position',[0 0 1280 720]); 
set(h0,'nextplot','replacechildren'); set(h0(3),'cameraviewangle',8,'clim',[0.8 4],'position',[.38 .05 .6 .9])
lim3 = [0 3 -2 2 -2 1];  xlims=[0 0]; ylims=[0 0]; zlims=[0 0];
set(h0([1 4]),'cameraviewangle',5.5)
C=[0  0  1;  1  0  0;  0 -1  0];  %Takashi 2 NED
set(h0(2),'position',[.38 .1 .08 .08],'cameraviewangle',4)
set(h0(5),'position',[.38 .42 .1 .1])
set(h0(6),'position',[.39 .82 .1 .1])
axis(h0([2 5]),'off')
%delete(h0(6))

h1=[];  h4a=[]; h2a=[]; h1a=[]; h1b=[]; xc=zeros(60E6,3,'single');
nc=1;
%v = zeros(256,256,256);
xhat = [0 0 0, 0 0 0];% xyz rpy
if vidFlag
    vidObj = VideoWriter(fcnincrementfname('SLAM_rgb'),'MPEG-4');
    open(vidObj);
end
for i=1:ni
    fprintf('frame %.0f\n',i)
    xyzi = xyzv(:,:,i)*C';
    rv = fcnrange(xyzi);  v1=find(rv>0);  x1=xyzi(v1,:);  n1=numel(v1);
    r=reshape(double(rv),480,640); r(r==0)=nan;
    
    %221
    h=h0(1);
    delete(h1a(ishandle(h1a))); h1a=image(rgb(:,:,:,i),'Parent',h); title(h,sprintf('frame %.0f, t=%.2fs',i,i/30),'units','normalized','position',[.5 .9]); hold(h,'on'); if i==1; set(h,'ydir','reverse'); axis(h,'off','equal'); end
    
    %224
    if i==1
        h=h0(4); h224=pcolor(h,r); shading(h,'flat'); hc=colorbar('peer',h,'location','north'); set(h,'clim',[0.8 4]); set(h,'ydir','reverse'); fcncolorbar(1,'m',hc); axis(h,'equal','off'); set(hc,'xcolor',[.7 .7 .7],'ycolor',[.7 .7 .7]); box(h,'on')
    else
        set(h224,'CData',r);
    end
    
    %223
    h=h0(3);
    if i>1
        I1 = im2double(rgb2gray(rgb(:,:,:,i-1)));
        %xhat = fcngetvispose3(xc(1:nc,:),x1,xhat,i,rgb,xyzi,xyzim1,h0);
        %[~,xhat]=fcnregsmall2(h0,xhat,rgb,I1,i);
        [xhat,I1,features1,vp1] = fcngetvispose(h0,xhat,rgb,I1,features1,vp1,i);
        xhat = fcngetpose(xc(1:nc,:),x1,xhat,i,rgb,xyzi,h0);
    else
        I1 = im2double(rgb2gray(rgb(:,:,:,i)));
        cornerPoints1 = detectHarrisFeatures(I1);%,'MaximumCornerCount', 500, 'CornerThreshold', 0.00001, 'NeighborhoodSize', [21 21]);
        [features1, vp1] = extractFeatures(I1, cornerPoints1, 'BlockSize', 21); %valid points
    end
    x1_w=fcnxform(x1,xhat);  xc(nc:nc+n1-1,:)=x1_w;  nc=nc+n1-1;

    delete(h4a); %set(h4a,'color',[.8 .8 .8],'linewidth',.5)
    h4a = fcnplotcam(h,xhat,rgb(:,:,:,i)); hold(h,'on');
    v2 = int32(rand(50000,1)*nc+1);  xc1=xc(v2,:);
    scatter3(h,xc1(:,1),xc1(:,2),xc1(:,3),1,fcnrange(xc1,xhat(1:3)));
    
    if i==1
        hold(h,'on'); axis(h,'equal','vis3d'); box(h,'on'); view(h,-60,89); set(h,'clim',[0.8 4]); set(h,'ydir','reverse','zdir','reverse','ylimmode','manual','zlimmode','manual','xlimmode','manual');
        xlabel(h,'x (m)'); ylabel(h,'y (m)'); zlabel(h,'z (m)'); axis(h,'off')
    end
    view(h,-60+i*.5,40)
   
    x=fcnxform([0 0 0; .1 0 0],xhat);  plot3(h,x(:,1),x(:,2),x(:,3),'r-');
    y=fcnxform([0 0 0; 0 .1 0],xhat);  plot3(h,y(:,1),y(:,2),y(:,3),'g-');
    z=fcnxform([0 0 0; 0 0 .1],xhat);  plot3(h,z(:,1),z(:,2),z(:,3),'b-');
    
    %get new lims
    xlims=fcnminmax([x1_w(:,1); xlims']); ylims=fcnminmax([x1_w(:,2); ylims']); zlims=fcnminmax([x1_w(:,3); zlims']);  axis(h0(3),[xlims ylims zlims])
    if i==1;  fcnfontsize(18); end
        
    xyzim1=xyzi;
    drawnow;  if vidFlag; writeVideo(vidObj,getframe(hf)); end
end
xc=xc(1:nc,:);
v2 = randi(nc,min(1E6,nc),1);  xc1=xc(v2,:);  scatter3(h,xc1(:,1),xc1(:,2),xc1(:,3),0.01,fcnrange(xc1,xhat(1:3)));

%MESHLAB OUTPUT -----------------------------------------------------------
% fid=fopen('xc.asc','wt');
% fprintf(fid,'%3.4f, %3.4f, %3.4f \n',xc(1:10E6,:)')
% fclose(fid)

%VOXELIZE -----------------------------------------------------------------
%v = fcnvoxels(xc1);

delete(h0(h0~=h0(3)))
h=h0(3);
pos1 = get(h,'pos');
pos2 = [0 0 1 1];
v = 1-logspace(0,-2,30);
view0=get(h,'view');
for i=1:numel(v)
    set(h,'pos',pos1*(1-v(i))+pos2*v(i))
    view(h,view0(1),view0(2)+v(i)*50);
    drawnow;  if vidFlag; writeVideo(vidObj,getframe(hf)); end
    fprintf('Camera transition frame %.0f/%.0f\n',i,numel(v))
end

v = 1-logspace(0,-2,150);
view0=get(h,'view');
va = get(h,'cameraviewangle');
for i = 1:numel(v)
    view(h,view0(1)+v(i)*450,view0(2)+v(i)*-75)
    set(h,'cameraviewangle',va-va*.5*v(i));
    drawnow;  if vidFlag; writeVideo(vidObj,getframe(hf)); end
    fprintf('Camera transition frame %.0f/%.0f\n',i,numel(v))
end
if vidFlag; close(vidObj); end









