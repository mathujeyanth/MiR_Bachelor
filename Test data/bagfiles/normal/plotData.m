close all;
clc;
clear;

%% Import of data
tic %Timing of section - Start timer

normalBag = rosbag('simplePath_2.bag');

bagselect_bscan= select(normalBag, 'Topic', '/b_scan');
bagselect_fscan= select(normalBag,'Topic', '/f_scan');

msgs = readMessages(normalBag);



disp('Time for section: "Import of data"')
toc %Timing of section - Stop timer
%% Calculating off-set
tic %Timing of section - Start timer

for i=1:length(msgs)
    if strcmp(msgs{i,1}.MessageType, 'tf2_msgs/TFMessage')
        if strcmp(msgs{i,1}.Transforms.ChildFrameId, 'front_laser_link')
            FrontLidarPosition= [...
                msgs{i,1}.Transforms.Transform.Translation.X...
                msgs{i,1}.Transforms.Transform.Translation.Y...
                msgs{i,1}.Transforms.Transform.Translation.Z...
                ];
            break;
        end
    end
end
for i=1:length(msgs)
    if strcmp(msgs{i,1}.MessageType, 'tf2_msgs/TFMessage')
        if strcmp(msgs{i,1}.Transforms.ChildFrameId, 'back_laser_link')
            BackLidarPosition= [...
                msgs{i,1}.Transforms.Transform.Translation.X...
                msgs{i,1}.Transforms.Transform.Translation.Y...
                msgs{i,1}.Transforms.Transform.Translation.Z...
                ];
            break;
        end
    end
end

disp('Time for section: "Calculating off-set"')
toc %Timing of section - Stop timer
%% Correct plotting of backScan and frontScan in same plot

% varTemp=msgs{6,1};
% thetaFront = varTemp.readScanAngles;
% theta=-thetaFront-thetaFront(1);
% 
% 
% rangeFront = varTemp.Ranges;
% [xFront,yFront] = pol2cart(theta,rangeFront);
% xFront=xFront-FrontLidarPosition(1);
% yFront=yFront+FrontLidarPosition(2);
% 
% varTemp=msgs{1,1};
% %thetaBack=varTemp.readScanAngles;
% rangeBack=varTemp.Ranges;
% [xBack,yBack]=pol2cart(theta,-rangeBack);
% xBack=xBack-BackLidarPosition(1);
% yBack=yBack+BackLidarPosition(2);
% 
% figure(1)
% 
% hold on
% for i=1:length(rangeFront)
%     if rangeFront(i) >= 25 || rangeFront(i) == 0
%         continue
%     else
%         scatter(xFront(i),yFront(i),'.','blue')
%     end
%     hold on
% end
% 
% for i=1:length(rangeBack)
%     if rangeBack(i) >= 25 || rangeBack(i) == 0
%         continue
%     else
%         scatter(xBack(i),yBack(i),'.','red')
%     end
%     hold on
% end
% hold off


% figure(3);polarplot(thetaFront,rangeFront,'.')
% [xFront,yFront]=pol2cart(thetaFront,rangeFront);
% figure(4);plot(xFront,yFront,'.')



% hold off
% hold on
%
% varTemp=msgs{1,1};
% thetaBack=varTemp.readScanAngles;
% rangeBack=varTemp.Ranges;
% [xBack,yBack]=pol2cart(thetaBack,rangeBack);
% xBack=xBack+BackLidarPosition(1);
% yBack=yBack+BackLidarPosition(2);
% scatter(xBack,yBack,'.','blue')
%
%
% hold off

% Rotated Polar plot
% figure(202);polarplot(-thetaBack-thetaBack(1),-rangeBack,'.');
% hold on;polarplot(-thetaFront-thetaFront(1),rangeFront,'.');hold off;

%% Move of data into 2 vectors
tic %Timing of section - Start timer

index_front=1;
index_back=1;
%Allocate size matching number of scans along with number of LIDAR
clear vecFront vecBack
vecFront=zeros(bagselect_bscan.NumMessages,541); 
vecBack=zeros(bagselect_fscan.NumMessages,541);


for i=1:length(msgs)
    if strcmp(msgs{i,1}.MessageType, 'sensor_msgs/LaserScan')
        if strcmp(msgs{i,1}.Header.FrameId, 'front_laser_link')
            vecFront(index_front,:)=(msgs{i,1}.Ranges)';
            index_front=index_front+1;
        end
        if strcmp(msgs{i,1}.Header.FrameId, 'back_laser_link')
            vecBack(index_back,:)=(msgs{i,1}.Ranges)';
            index_back=index_back+1;
        end
    end
end

clear index_front index_back
disp('Time for section: "Move of data into 2 vectors"')
toc %Timing of section - Stop timer
%% Applying off-set
tic %Timing of section - Start timer

thetaFront = msgs{1,1}.readScanAngles;
theta=-thetaFront-thetaFront(1); 
clear thetaFront

clear xFront yFront xBack yBack %Deletion of old matrices - ensures correct creation of matrices
xFront=zeros(min(size(vecFront,1),size(vecBack,1)),min(size(vecFront,2),size(vecBack,2)));
yFront=zeros(min(size(vecFront,1),size(vecBack,1)),min(size(vecFront,2),size(vecBack,2)));
xBack=zeros(min(size(vecFront,1),size(vecBack,1)),min(size(vecFront,2),size(vecBack,2)));
yBack=zeros(min(size(vecFront,1),size(vecBack,1)),min(size(vecFront,2),size(vecBack,2)));

for i=1:min(size(vecFront,1),size(vecBack,1))
    [xFront(i,:),yFront(i,:)] = pol2cart(theta',vecFront(i,:));
    xFront(i,:)= xFront(i,:)-FrontLidarPosition(1);
    yFront(i,:)= yFront(i,:)+FrontLidarPosition(2);
    
    [xBack(i,:),yBack(i,:)] = pol2cart(theta',-vecBack(i,:));
    xBack(i,:)= xBack(i,:)-BackLidarPosition(1);
    yBack(i,:)= yBack(i,:)+BackLidarPosition(2);
end

disp('Time for section:"Applying off-set"')
toc %Timing of section - Stop timer
%% Removal of invalid data
tic %Timing of section - Start timer
for i=1:504
    for j=1:length(theta)
        if vecFront(i,j) >= 25 || vecFront(i,j) == 0
            xFront(i,j)=NaN;
            yFront(i,j)=NaN;
        end
        if vecBack(i,j) >= 25 || vecBack(i,j) == 0
            xBack(i,j)=NaN;
            yBack(i,j)=NaN;
        end
    end
end

disp('Time for section:"Removal of invalid data"')
toc %Timing of section - Stop timer
%% Plot of front and back with correct off-set
tic %Timing of section - Start timer
for i=1:504
    figure(1)
    
    scatter(xFront(i,:),yFront(i,:),'.','blue')
    hold on
    scatter(xBack(i,:),yBack(i,:),'.','red');
    axis([-20 20 -20 20])
    hold off
    disp(i)
    pause(50/1000)
end

disp('Time for section:"Plot of front and back with correct off-set"')
toc %Timing of section - Stop timer