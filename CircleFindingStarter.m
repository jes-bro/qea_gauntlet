%This script will provide some sample code for finding circles in a LIDAR
%dataset. This is not meant to be the most robust routine, but should
%provide a starting point for real-time circle identification, or to be
%combined with a robust line fitting script to identify both circles and
%lines.

%First, let's play around with some sample data and show how we can apply
%linear regression to circle fitting

%The makeCircle function generates data points that lie on an arc of a
%circle with some noise added in.
[x,y,rs,thetas] = makeCircle(2, 0, 120, .05);

%We will set up the circle fitting problem using linear regression
%techniques from linear algebra i.e. an overconstrained system of linear equations
% where A*w = b. To do this, we will use the general form of the circle
% equation: x^2+y^2+Ax+By+C-0. Written in terms of our weight vector, w,
% this will be x^2+y^2+w(1)x+w(2)y+w(3)=0. In matrix form:
%[x(1) y(1) 1 [w(1)  = [-x(1)^2-y(1)^2
% x(2) y(2) 1  w(2)     -x(1)^2-y(1)^2
% :     :  :   w(3)]      :      :
% x(n) y(n) 1]          -x(n)^2-y(n)^2]

A = [x y ones(size(x))];
b = -x.^2 - y.^2;
w = A\b;

% convert from the least squares solution to the more familiar parameters
% of a circle.
xc = -w(1)/2;
yc = -w(2)/2;
r = sqrt(xc.^2 + yc.^2 - w(3));

figure(1)
scatter(x,y);
xlim([-2 2]);
ylim([-2 2]);
axis equal
hold on
viscircles([xc yc], r);
title('Demo Circle Fit')
%%
clear r

%For this demo script, we will use the playpensample data set. In this
%scan, there is one arc of datapoints in the upper left that could be
%considered a portion of the "Barrel of Benevolence." Our goal will be to
%identify and fit a circle to that set of points.
load playpensample

%eliminate zeros and sample farther than 3m (range of LIDAR)
index=find(r~=0 & r<3);
r_clean=r(index);
theta_clean=theta(index);

%plot the polar data as verification
figure(2)
polarplot(deg2rad(theta_clean),r_clean,'ks','MarkerSize',6,'MarkerFaceColor','m')
title('Visualization of Polar Data')

%convert to Cartesian and plot again for verification
[x,y]=pol2cart(deg2rad(theta_clean),r_clean);
points=[x,y];
figure(3)
plot(x,y,'ks')
xlabel('[m]')
ylabel('[m]')
%%
clear A b w
%Now, we will use a RANSAC-type approach to finding and fitting a circle to
%the playpensample data. 

%To make sure we find the BOB, we would like to know the radius we are
%looking for. I don't know that off hand, so I'm going to isolate those
%points and use the circle fitting tool.
indexx=find(x > -0.25 & x < -0.15 );
indexy=find(y > 0.04 & y < 0.2);
index_circ=intersect(indexx,indexy);
circx=x(index_circ);
circy=y(index_circ);
figure(3)
hold on
plot(circx,circy,'b*')

%apply our linear regression approach
A = [circx circy ones(size(circx))];
b = -circx.^2 - circy.^2;
w = A\b;

% convert from the least squares solution to the more familiar parameters
% of a circle.
xc = -w(1)/2;
yc = -w(2)/2;
rBOB = sqrt(xc.^2 + yc.^2 - w(3));

%Plot this on scan to check
figure(3)
viscircles([xc yc], rBOB);
title('Test to find BOB radius')

%%
%Ok, let's go ahead and do some RANSAC-type stuff. We will use the
%robustLineFit code as a base to start from.
clear A b w xc yc

 n=1000;
 bestr = [];
 bestInlierSet = zeros(0,2);
 bestOutlierSet = zeros(0,2);
 bestcenter = zeros(0,2);
for k=1:n %number of candidate circles to try
    
    %The minimum number of points we would need to define a circle is two,
    %but with only two points you have two-solutions for the location of
    %the center. Instead of dealing with that, I am going to sleect three
    %points in this case.
    %Select three points at random using the 'datasample' function
    candidates = datasample(points, 3, 'Replace', false);
    
    %now we apply our linear regression routine to the three points.
    %apply our linear regression approach
    A = [circx circy ones(size(circx))];
    b = -circx.^2 - circy.^2;
    w = A\b;

    % convert from the least squares solution to the more familiar parameters
    % of a circle.
    xc = -w(1)/2;
    yc = -w(2)/2;
    rfit = sqrt(xc.^2 + yc.^2 - w(3));
   
    %To identify inliers, we are going to look for points that are within a
    %threshold distance from our best fit circle.
    threshold=0.01;
    distFromCircle = abs(sqrt((points(:,1) - xc).^2 + (points(:,2) - yc).^2) - rfit);
    inliers=distFromCircle < threshold;
    
    %Now, we check if the number of inliers is greater than the best we
    %have found. If so, the candidate line is our new best candidate. We
    %also want to make sure the fitted radius is close to the radius of the
    %BOB
    if abs(rfit-rBOB) < 0.1  && sum(inliers) > size(bestInlierSet,1)
        bestInlierSet=points(inliers,:); %points where logical array is true
        bestOutlierSet = points(~inliers, :); %points where logical array is not true
        bestr=rfit;
        bestcenter=[xc,yc];
    end

end

%%
%Visualize the result

figure(4)
h1=plot(x,y,'ks');
hold on
h2=viscircles([xc yc], bestr);
title('Circle Fitting')
legend([h1 h2],'Scan Points','Fit Circle')
xlabel('[m]')
ylabel('[m]')


