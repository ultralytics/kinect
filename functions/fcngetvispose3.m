function xhat = fcngetvispose(xc,x1,xhat,i,rgb,xyzi,xyzim1,h)
options = optimset('TolX',1E-9,'TolFun',1E-9,'MaxIter',50,'Display','off','MaxFunEvals',5000,'LargeScale', 'off','GradObj','on'); tic
if i==1
    return
end

I1 = im2double(rgb2gray(rgb(:,:,:,i-1)));
I2 = im2double(rgb2gray(rgb(:,:,:,i)));
hcornerdet = vision.CornerDetector('MaximumCornerCount', 500, 'CornerThreshold', 0.00001, 'NeighborhoodSize', [21 21]);
cornerPoints1 = step(hcornerdet, I1);
cornerPoints2 = step(hcornerdet, I2);
[features1, validPoints1] = extractFeatures(I1, cornerPoints1, 'BlockSize', 21);
[features2, validPoints2] = extractFeatures(I2, cornerPoints2, 'BlockSize', 21);
[indexPairs, match_metric] = matchFeatures(features1, features2, 'Metric', 'normxcorr', 'MatchThreshold', 20);
mp1 = validPoints1(indexPairs(:, 1), :);  %matched points 1
mp2 = validPoints2(indexPairs(:, 2), :);  %matched points 2
%cvexShowImagePair(I1, I2, 'Corners in left image', 'Corners in right image', 'SingleColor', cornerPoints1, cornerPoints2);

r=sqrt(sum((double(mp1)-double(mp2)).^2,2));
j=r<100;
mp1=mp1(j,:);  mp2=mp2(j,:);
for i=1:6
    r=sqrt(sum((double(mp1)-double(mp2)).^2,2));  mu=mean(r);  sigma=std(r);
    j=r<(mu+2*sigma) & r>(mu-2*sigma);
    mp1=mp1(j,:);  mp2=mp2(j,:);
    %cvexShowMatches(I1, I2, mp1, mp2, 'Matched points in I1', 'Matched points in I2');
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

%cvexShowMatches(I1, I2, ip1, ip2, 'Inlier points in I1', 'Inlier points in I2');
plot(h(1),[ip1(:,1) ip2(:,1)]',[ip1(:,2) ip2(:,2)]','y-',ip1(:,1),ip1(:,2),'ro',ip2(:,1),ip2(:,2),'g+')
%[t1, t2] = estimateUncalibratedRectification(fMatrix, ip1, ip2, size(I2));
%cvexShowStereoImages('Rectified image 1', 'Rectified image 2', I1, I2, ip1, ip2, fMatrix, t1, t2);
%cvexShowMatches(I1, I2, ip1, ip2, 'Inlier points in I1', 'Inlier points in I2', 'RC', t1, t2);

%I3 = reshape(fcnrange(xyzim1),480,640);  I3=I3./max3(I3);
%I4 = reshape(fcnrange(xyzi),480,640);  I4=I4./max3(I4);
%cvexShowMatches(I3, I4, ip1, ip2, 'Inlier points in I1', 'Inlier points in I2', 'RC', t1, t2);
%cvexShowImagePair(I3, I4, 'Corners in left image', 'Corners in right image', 'SingleColor', ip1, ip2);


%3D OPTIMIZATION
ind=sub2ind([480 640],ip1(:,2),ip1(:,1));  a=xyzim1(ind,:);
ind=sub2ind([480 640],ip2(:,2),ip2(:,1));  b=xyzi(ind,:);
i=find(fcnrange(a,b)<10 & sum(a.*b,2)>0);  a=a(i,:);  b=b(i,:);
%delete(get(h(5),'children')); 
%figure; plot3(a(:,1),a(:,2),b(:,3),'ro',b(:,1),b(:,2),b(:,3),'g+',[a(:,1)'; b(:,1)'],[a(:,2)'; b(:,2)'],[a(:,3)'; b(:,3)'],'y')

if numel(i)<4
    fprintf('Visual registration failed, only %.0f points available!\n',numel(i))
    return
end
a_w=fcnxform(a,xhat);
xhatout=fminunc(@(x) fcncost(a_w,b,x), double(xhat), options);
%x = fcnxform(x1,xhat);  %plot3(h(6),x(:,1),x(:,2),x(:,3),'.g','markersize',5);

r=fcnrange(xhat(1:3),xhatout(1:3));
droll = abs(xhat(4)-xhatout(4));
dpitch = abs(xhat(5)-xhatout(5));
dyaw = abs(xhat(6)-xhatout(6));

if r>.5 || droll>.5 || dpitch>.5 || dyaw>.5
    fprintf('Visual registration failed, position jump detected (%.1fm, roll=%.0f, pitch=%.0f, yaw=%.0f)!\n',r,droll*57.3,dpitch*57.3,dyaw*57.3)
    return
end
xhat = xhatout;
fprintf('%9.3f%9.3f%9.3f%9.3f%9.3f%9.3f (%.2fs icp)\n',xhat,toc)
end
    

function [fx, gx] = fcncost(a,b,x)
dx = 1E-5;
b0=b;  b=fcnxform(b,x);
[~,rs] = rangec(a,b);
fx = sum(rs);

fg=zeros(6,1);
for i=1:6
    x1=x; x1(i)=x1(i)+dx;
    b = fcnxform(b0,x1);
    rs=sum((b-a).^2,2);
    fg(i) = sum(rs);
end
gx = (fg-fx)./dx;
end

    