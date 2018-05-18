function [xhat,I2,features2,vp2] = fcngetvispose(h,xhat,rgb,I1,features1,vp1,i)
options = optimset('TolX',1E-9,'TolFun',1E-9,'MaxIter',50,'Display','off','MaxFunEvals',5000,'LargeScale', 'off','GradObj','on'); tic
if i==1
    return
end
rpy0 = fcnregsmall2(h,xhat,rgb,I1,i);
%rpy0 = [0 0 0];

%I1 = im2double(rgb2gray(rgb(:,:,:,i-1)));
I2 = im2double(rgb2gray(rgb(:,:,:,i)));
%cornerPoints1 = detectHarrisFeatures(I1);
cornerPoints2 = detectHarrisFeatures(I2);
%[features1, vp1] = extractFeatures(I1, cornerPoints1, 'BlockSize', 21); %valid points
[features2, vp2] = extractFeatures(I2, cornerPoints2, 'BlockSize', 21);
[indexPairs, match_metric] = matchFeatures(features1, features2, 'MatchThreshold', 20);
mp1 = vp1(indexPairs(:, 1), :);  %matched points 1
mp2 = vp2(indexPairs(:, 2), :);  %matched points 2
%cvexShowImagePair(I1, I2, 'Corners in left image', 'Corners in right image', 'SingleColor', cornerPoints1, cornerPoints2);

mp1 = mp1.Location;
mp2 = mp2.Location;

r=sqrt(sum((mp1-mp2).^2,2));
j=r<100;
mp1=mp1(j,:);  mp2=mp2(j,:);
for ii=1:6
    r=sqrt(sum((mp1-mp2).^2,2));  mu=mean(r);  sigma=std(r);
    j=r<(mu+2*sigma) & r>(mu-2*sigma);
    mp1=mp1(j,:);  mp2=mp2(j,:);
end
%cvexShowMatches(I1, I2, mp1, mp2, 'Matched points in I1', 'Matched points in I2');

if isempty(mp1)
    fprintf('WARNING: Not enough tie points to register.\n')
    xhat = [xhat(1:3), fcnW2BDCM2RPY(  (fcnRPY2DCM_B2W(xhat(4:6))*fcnRPY2DCM_W2B(rpy0))'  )];
    return
end

% [y,i]=sort(rs);
% inliers = i(1:round(numel(rs)*.5));
ip1 = mp1;
ip2 = mp2;
if size(mp1,1)>10
    try %#ok<TRYNC>
        [fMatrix, inliers] = estimateFundamentalMatrix(mp1, mp2, 'Method', 'RANSAC', 'NumTrials', 1000, 'DistanceThreshold', 0.01, 'Confidence', 99.999);
        ip1 = mp1(inliers, :);
        ip2 = mp2(inliers, :);
    end
end

if isempty(ip1)
    fprintf('WARNING: Not enough tie points to register.\n')
    xhat = [xhat(1:3), fcnW2BDCM2RPY(  (fcnRPY2DCM_B2W(xhat(4:6))*fcnRPY2DCM_W2B(rpy0))'  )];
    return
end

%cvexShowMatches(I1, I2, ip1, ip2, 'Inlier points in I1', 'Inlier points in I2');
plot(h(1),[ip1(:,1) ip2(:,1)]',[ip1(:,2) ip2(:,2)]','y-',ip1(:,1),ip1(:,2),'ro',ip2(:,1),ip2(:,2),'g+')
%[t1, t2] = estimateUncalibratedRectification(fMatrix, ip1, ip2, size(I2));
%cvexShowStereoImages('Rectified image 1', 'Rectified image 2', I1, I2, ip1, ip2, fMatrix, t1, t2);
%cvexShowMatches(I1, I2, ip1, ip2, 'Inlier points in I1', 'Inlier points in I2', 'RC', t1, t2);

%I3 = reshape(fcnrange(xyzim1),480,640);  I3=I3./max3(I3);
%I4 = reshape(fcnrange(xyzi),480,640);  I4=I4./max3(I4);
%cvexShowMatches(I3, I4, ip1, ip2, 'Inlier points in I1', 'Inlier points in I2', 'RC', t1, t2);
%cvexShowImagePair(I3, I4, 'Corners in left image', 'Corners in right image', 'SingleColor', ip1, ip2);

ip1=double(ip1); ip2=double(ip2);
a=[ip1(:,1)-641/2 481/2-ip1(:,2)];  b=[ip2(:,1)-641/2 481/2-ip2(:,2)];
%figure; plot(a(:,1),a(:,2),'ro',b(:,1),b(:,2),'g+',[a(:,1)'; b(:,1)'],[a(:,2)'; b(:,2)'],'y'); axis([-640/2 640/2 -480/2 480/2])

p2r = 0.00155898; %pixels to radians, ((57/640+43/480)/2)*pi/180
asc=[ones(size(a,1),1), a(:,[2 1])*p2r];  bsc=[ones(size(b,1),1), b(:,[2 1])*p2r];
acc=fcnSC2CC(asc);  bcc=fcnSC2CC(bsc);

rpy=fminunc(@(rpy) fcnanglecost(acc,bcc,rpy), rpy0, options);
fprintf('%9.3f%9.3f%9.3f rpy (%.2fs RANSAC corners)\n',rpy,toc)
if any(abs(rpy)>.6)
    fprintf('WARNING: Visual registration failed, position jump detected (roll=%.0f, pitch=%.0f, yaw=%.0f)!\n',rpy*57.3)
    rpy = rpy0;
end

xhat = [xhat(1:3), fcnW2BDCM2RPY(  (fcnRPY2DCM_B2W(xhat(4:6))*fcnRPY2DCM_W2B(rpy0))'  )];
fprintf('%9.3f%9.3f%9.3f%9.3f%9.3f%9.3f (%.2fs RANSAC corners)\n',xhat,toc)
end
    

function [fx, gx] = fcnanglecost(a,b,x)
dx = 1E-5;
a1 = fcnxform(a,[0 0 0 x]);
theta = fcnangle(a1,b);
fx = sum(theta.^2);

fg=zeros(3,1);
for i=1:3
    x1=x; x1(i)=x1(i)+dx;
    a1 = fcnxform(a,[0 0 0 x1]);
    theta = fcnangle(a1,b);
    fg(i) = sum(theta.^2);
end
gx = (fg-fx)./dx;
end


    