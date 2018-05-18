function [rpy,xhat] = fcnregsmall2(hin,xhat,rgb,I1,i)
options = optimset('TolX',1E-9,'TolFun',1E-9,'MaxIter',50,'Display','off','MaxFunEvals',5000,'LargeScale', 'off','GradObj','on'); tic
if i==1
    return
end
p2r = 0.00155898; %pixels to radians, ((57/640+43/480)/2)*pi/180
scale = 1/4;

pf = 0; %plotflag

I2 = im2double(rgb2gray(rgb(:,:,:,i)));
I1s = imresize(I1, [480 640]*scale,'bilinear');
I2s = imresize(I2, [480 640]*scale,'bilinear');
a = I1s(30:end-29,30:end-29);
if pf; h=fig(2,3); axes(h(1)); imshow(I1s); axes(h(2)); imshow(I2s); axes(h(3)); imshow(a); axis([-30 130 -30 90]); end
imagesc(rgb(:,:,:,i-1),'Parent',hin(6));  set(hin(6),'ydir','reverse','cameraviewangle',3); title(hin(6),'previous frame','units','normalized','position',[.5 1]); axis(hin([2 5 6]),'off','equal')

xv = -29:1:29;  nxv = numel(xv);
fx = zeros(nxv, nxv);
for j=1:59 %up-down
    I2sj=I2s(30+xv(j):end-29+xv(j),:);
    for k=1:59 %left-right
        b = I2sj(:,30+xv(k):end-29+xv(k)); %if pf; axes(h(4)); imshow(b); drawnow; end
        di = a-b;
        fx(j,k) = sum(sum(di.^2));
    end
end
[~,i] = min3(fx);  
rpy = [0 [-i(1)+30, i(2)-30]*p2r/scale];
pcolor(hin(2),fliplr(fx)); shading(hin(2),'flat'); title(hin(2),sprintf('pitch=%.0f, yaw=%.0f',rpy(2:3)*57.3))


if nargout==2
   xhat = [xhat(1:3), fcnW2BDCM2RPY(  (fcnRPY2DCM_B2W(xhat(4:6))*fcnRPY2DCM_W2B(rpy))'  )]; 
end

% %DEBLUR PSF
% PSF = fspecial('gaussian',20,20);
% INITPSF = ones(size(PSF));
% WT = zeros(size(I2s));
% WT(20:end-19,20:end-19) = 1;
% [J P] = deconvblind(I2s,INITPSF,20,.1,WT);
% figure; imagesc(PSF)
% figure; imshow(J)
% figure; imshow(I2s)

fprintf('%9.3f%9.3f%9.3f rpy (%.2fs small registration)\n',rpy,toc)
end


    