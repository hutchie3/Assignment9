clc 
clear all
close all

StartingFrame = 0;
EndingFrame = 4;

for k = StartingFrame : EndingFrame
    
    Ipts1 = [];
    Ipts2 = [];
    
    D1 = [];
    D2 = [];
    
    BaseIndex = [];
    SubIndex = [];
    index = [];
    
    BaseLength = [];
    SubLength = [];
    
    SubValue = [];
    
    % Reading in images.. if past the first frame, use the Jregistered
    % image from the previous step as pos1, and load a new image into pos2
    if k == 0
        pos1 = imread(['img/', sprintf('%0.1d',k),'.jpg']);
        pos2 = imread(['img/', sprintf('%0.1d',k+1),'.jpg']);
    else
        pos1 = Jregistered;
        pos2 = imread(['img/', sprintf('%0.1d',k+1),'.jpg']);
    end
    
    Im1 = rgb2gray(pos1);
    Im2 = rgb2gray(pos2); 
    
    Ipts1 = OpenSurf(pos1);
    Ipts2 = OpenSurf(pos2);
    
    BaseLength = length(Ipts1);
    SubLength = length(Ipts2);
    
    for i = 1:BaseLength
        D1(:,i) = Ipts1(i).descriptor;
        x1(:,i) = Ipts1(i).x;
        y1(:,i) = Ipts1(i).y;
    end
    
    for i = 1:SubLength
        D2(:,i) = Ipts2(i).descriptor;
        x2(:,i) = Ipts2(i).x;
        y2(:,i) = Ipts2(i).y;
    end
    
    for i = 1:BaseLength
        subtract = (repmat(D1(:,i), [1, SubLength]) - D2).^2;
        distance = sum(subtract);
        [SubValue(i), SubIndex(i)] = min(distance);
        x1 = repmat(x1(:,i), [1, SubLength]);
        y1 = repmat(y1(:,i), [1, SubLength]);
    end
    [value, index] = sort(SubValue);

    % Roughly matching all of the corners by correlation
    [m1,m2] = matchbycorrelation(Im1, [x1; y1], Im2, [x2; y2], 21, 100);

    % RANSAC
    [H, inliers] = ransacfithomography(m1, m2, 0.001);
    
    % Moving = second frame, Fixed = first frame or Jregistered
    fixedPoints = [m1(2,inliers)' m1(1,inliers)'];
    movingPoints = [m2(2,inliers)' m2(1,inliers)'];
     
    % Determining the transform based on the relationship matrices between
    % the coordinates in the two images
    tform = fitgeotrans(movingPoints,fixedPoints,'NonreflectiveSimilarity');
    
    % Image registration (alignment)
    Jregistered = imwarp(pos2,tform,'OutputView',imref2d(size(pos1)));
    falsecolorOverlay = imfuse(pos1,Jregistered);

    % Putting everything together into a 2x2 image
    I1 = cat(2,pos2,Jregistered);
    im1rgb = cat(3,Im1,Im1,Im1);
    I2 = cat(2,im1rgb,falsecolorOverlay);
    I = cat(1,I1,I2);

    % Accounting for the concatenation
    shiftY = size(Im1,1);
    shiftX = size(Im1,2);

    % Displaying the four images
    imshow(I,'Border','tight'); hold on;

    % Plotting the relationships between POS1 and POS2 images (RANSAC
    % results only at this point)
    plot(m1(2,inliers),m1(1,inliers)+shiftY,'r+');
    plot(m2(2,inliers),m2(1,inliers)+shiftY,'b+');    



end

index = index(1:100);

Jregistered = imwarp(pos2, tform, 'OutputView', imref2d(size(pos1)));
falsecolorOverlay = imfuse(pos1, Jregistered);
blendcolorOverlay = imfuse(pos1, Jregistered, 'blend');