%{ 
  DIY 3D Scanner Post-Processing Code (DIY 3D Scanner - Super Make Something Episode 8) - https://youtu.be/-qeD2__yK4c
  by: Alex - Super Make Something
  date: January 2nd, 2016
  license: Creative Commons - Attribution - Non-Commercial.  More information available at: http://creativecommons.org/licenses/by-nc/3.0/
 %}

clear all;
clc;

%% Processing variables
maxDistance=20; %Upper limit -- raw scan value only scanning "air"
minDistance=0; %Lower limit -- raw scan value error: reporting negative reading

midThreshUpper=0.5; %Offset radius threshold around 0
midThreshLower=-midThreshUpper; %Offset radius threshold around 0

windowSize=3; %Window size for average filter to clean up mesh
interpRes=1; %Interpolation resolution, i.e. keep every interRes-th row

centerDistance=10.3;%9.2; %[cm] - Distance from scanner to center of turntable
zDelta=0.1;
rawData=load('farmer.txt'); %Load text file from SD Card
rawData(rawData<0)=0; %Remove erroneous scans from raw data
indeces=find(rawData==9999); %Find indeces of '9999' delimiter in text file, indicating end of z-height scan

% Arrange into matrix, where each row corresponds to one z-height.
r(1,:)=rawData(1:indeces(1));
for i=2:1:size(indeces,1)
   r(i,:)=rawData(indeces(i-1)+1:indeces(i)); 
end

r(:,end)=[]; % Delete last row of 9999 delimiters.

r=centerDistance-r; %Offset scan so that distance is with respect to turntable center of rotation

r(r>maxDistance)=NaN; %Remove scan values greater than maxDistance;
r(r<minDistance)=NaN; %Remove scan values less than minDistance

%Remove scan values around 0
midThreshUpperIdx=r>midThreshLower;
midThreshLowerIdx=r<midThreshUpper;
midThreshIdx=midThreshUpperIdx.*midThreshLowerIdx;
r(midThreshIdx==1)=NaN;


% Create theta matrix with the same size as r -- each column in r corresponds to specific orientation
%theta=0:360/size(r,2):360;
theta=360:-360/size(r,2):0;
theta(end)=[];
theta=repmat(theta,[size(r,1) 1]);

theta=theta*pi/180; %Convert to radians

% Create z-height array where each row corresponds to one z-height
z=0:zDelta:size(r,1)*zDelta;
z(end)=[];
z=z';
z=repmat(z,[1,size(r,2)]);

[x,y,z]=pol2cart(theta,r,z); %Convert to cartesian coordinates


%Replace NaN values in x, y with nearest neighbor at the same height

for i=1:1:size(x,1)
   if sum(isnan(x(i,:)))==size(x,2)
      x(i:end,:)=[];
      y(i:end,:)=[];
      z(i:end,:)=[];
    break;
   end
end

for i=1:1:size(x,1)
   latestValueIdx=find(~isnan(x(i,:)),1,'first');
   latestX=x(i,latestValueIdx);
   latestY=y(i,latestValueIdx);
   for j=1:1:size(x,2)
       if isnan(x(i,j))==0
          latestX=x(i,j); 
          latestY=y(i,j);
       else
           x(i,j)=latestX;
           y(i,j)=latestY;
       end
       
   end
end

%Resample array based on desired mesh resolution
interpIdx=1:interpRes:size(x,1);
xInterp=x(interpIdx,:);
yInterp=y(interpIdx,:);
zInterp=z(interpIdx,:);



%Smoothe data to eliminate more noise
h=fspecial('average',windowSize); %Define average filter
%h=fspecial('gaussian',10,1.25); %Define gaussian filter
xInterp=padarray(xInterp,[0, windowSize],'symmetric'); %Add symmetric duplicate padding along rows to correctly filter array edges
yInterp=padarray(yInterp,[0, windowSize],'symmetric'); %Add symmetric duplicate padding along rows to correctly filter array edges
xInterp=filter2(h,xInterp); %Filter x
yInterp=filter2(h,yInterp); %Filter y
xInterp=xInterp(:,windowSize:end-windowSize-1); %Remove padding
yInterp=yInterp(:,windowSize:end-windowSize-1); %Remove padding

%Force scan to wrap by duplicating first column values at end of arrays
xInterp(:,end)=xInterp(:,1);
yInterp(:,end)=yInterp(:,1);
zInterp(:,end)=zInterp(:,1);

%Add top to close shape
xTop=mean(xInterp(end,:));
yTop=mean(yInterp(end,:));
xInterp(end+1,:)=xTop;
yInterp(end+1,:)=yTop;
zInterp(end+1,:)=zInterp(end,1)-zInterp(end-1,1)+zInterp(end,1);

%surf(xInterp,yInterp,zInterp); %Plot point cloud as a mesh to verify that processing is correct
plot3(xInterp,yInterp,zInterp,'.b'); %Plot point cloud as a mesh to verify that processing is correct

surf2stl('farmer.stl',xInterp,yInterp,zInterp); %Export as STL file