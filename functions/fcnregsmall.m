% Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license

function rpy = fcnregsmall(hin,xhat,rgb,I1,i)
options = optimset('TolX',1E-9,'TolFun',1E-9,'MaxIter',50,'Display','off','MaxFunEvals',5000,'LargeScale', 'off','GradObj','on'); tic
if i==1
    return
end
p2r = 0.00155898; %pixels to radians, ((57/640+43/480)/2)*pi/180
scale = 1/4;

pf = 1; %plotflag

I2 = im2double(rgb2gray(rgb(:,:,:,i)));
I1s = imresize(I1, [480 640]*scale,'bilinear');
I2s = imresize(I2, [480 640]*scale,'bilinear');
a = I1s(30:end-29,30:end-29);
if pf; h=fig(2,3); axes(h(1)); imshow(I1s); axes(h(2)); imshow(I2s); axes(h(3)); imshow(a); axis([-30 130 -30 90]); end
imagesc(rgb(:,:,:,i-1),'Parent',hin(6));  set(hin(6),'ydir','reverse','cameraviewangle',4); axis(hin(6),'off','equal')

xv = -29:1:29;
fx = zeros(size(xv));
for j=1:59
    b = I2s(30+xv(j):end-29+xv(j),30:end-29); if pf; axes(h(4)); imshow(b); drawnow; end
    di = a-b;
    fx(j) = mean(mean(di.^2));
end
[~, ix]=min(fx);  ix=30-ix;  pitch=ix*p2r/scale;
plot(hin(2),fx); title(hin(2),sprintf('pitch=%.0f',pitch*57.3))

for j=1:59
    b = I2s(30:end-29,30+xv(j):end-29+xv(j)); if pf; axes(h(5)); imshow(b); drawnow; end
    di = a-b;
    fx(j) = mean(mean(di.^2));
end
[~, ix]=min(fx);  ix=ix-30;  yaw=ix*p2r/scale;
plot(hin(5),fx); title(hin(5),sprintf('yaw=%.0f',yaw*57.3))

rpy = [0 pitch yaw];
fprintf('%9.3f%9.3f%9.3f rpy (%.2fs small registration)\n',rpy,toc)
end


    