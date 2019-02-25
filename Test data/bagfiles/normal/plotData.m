
normalBag = rosbag('simplePath_2.bag');
backScan = select (normalBag,'Topic','b_scan');



normalBag.AvailableTopics

%normalBag.MessageList

bagselect_bscan= select(normalBag, 'Topic', '/b_scan')

msgs = readMessages(normalBag);

ranges = msgs{1,1}.Ranges;
angles=msgs{1,1}.readScanAngles;
plot(msgs{1,1})

%%
for i=1:15
    msgs{i,1}.MessageType
end


%% Simple sammen plot af front og back
% for i=1:length(msgs)
%     if strcmp(msgs{i,1}.MessageType, 'sensor_msgs/LaserScan')
%          if strcmp(msgs{i,1}.Header.FrameId, 'back_laser_link')
%             figure(1)
%             plot(msgs{i,1})
%             title('Back scan - Lidar')
%             pause(5/1000)
%          end
%          if strcmp(msgs{i,1}.Header.FrameId, 'front_laser_link')
%             figure(2)
%             plot(msgs{i,1})
%             title('Front laser - Lidar')
%             pause(5/1000)
%          end
%     end
% end



%%
back_laser=0;

for i=1:length(msgs)
    if strcmp(msgs{i,1}.MessageType, 'sensor_msgs/LaserScan')
        if strcmp(msgs{i,1}.Header.FrameId, 'back_laser_link')
            back_laser=back_laser+1;
        end
    end    
end

front_laser=0;

for i=1:length(msgs)
    if strcmp(msgs{i,1}.MessageType, 'sensor_msgs/LaserScan')
        if strcmp(msgs{i,1}.Header.FrameId, 'front_laser_link')
            front_laser=front_laser+1;
        end
    end    
end

%% Type of transform
for i=18:44
    if strcmp(msgs{i,1}.MessageType, 'tf2_msgs/TFMessage')
        disp(':::')
        disp(i)
        disp(msgs{i,1}.Transforms.ChildFrameId)
        disp('---')
    end
end

%% Print out front_laser_link
for i=18:200
    if strcmp(msgs{i,1}.MessageType, 'tf2_msgs/TFMessage')
        if strcmp(msgs{i,1}.Transforms.ChildFrameId, 'back_laser_link')
            disp(':::')
            disp('XYZ')
            disp(i)
            disp(msgs{i,1}.Transforms.Transform.Translation.X)
            disp(msgs{i,1}.Transforms.Transform.Translation.Y)
            disp(msgs{i,1}.Transforms.Transform.Translation.Z)
            disp('---')
            break;
        end
    end
end    

%% Plot data manuelt
LaserScan_index=0;
for i=1:length(msgs)
    if strcmp(msgs{i,1}.MessageType, 'sensor_msgs/LaserScan')
        LaserScan_index=i;
        break;
    end
end

thetaFront=msgs{LaserScan_index,1}.AngleMin:msgs{LaserScan_index,1}.AngleIncrement:msgs{LaserScan_index,1}.AngleMax;

rangesFront=transpose(msgs{1,1}.Ranges);

[frontX,frontY]=pol2cart(thetaFront,rangesFront);
figure(1)
scatter(frontY,frontX);
axis([-25 25 -25 25]);


%%

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


varTemp=msgs{6,1};
thetaFront = varTemp.readScanAngles;
rangeFront = varTemp.Ranges;
[xFront,yFront] = pol2cart(-thetaFront-thetaFront(1),rangeFront);
xFront=xFront-FrontLidarPosition(1);
yFront=yFront+FrontLidarPosition(2);


varTemp=msgs{1,1};
thetaBack=varTemp.readScanAngles;
rangeBack=varTemp.Ranges;
[xBack,yBack]=pol2cart(-thetaBack-thetaBack(1),-rangeBack);
xBack=xBack-BackLidarPosition(1);
yBack=yBack+BackLidarPosition(2);

figure(1)
hold on
for i=1:length(rangeFront)
   if rangeFront(i) >= 25 || rangeFront(i) == 0
      continue
   else
       scatter(xFront(i),yFront(i),'.','blue')
   end
   hold on
end

for i=1:length(rangeBack)
   if rangeBack(i) >= 25 || rangeBack(i) == 0
      continue
   else
      scatter(xBack(i),yBack(i),'.','red')
   end
   hold on
end
hold off


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


%% Plot af front og back med korrekt off-set
for i=1:length(msgs)
    if strcmp(msgs{i,1}.MessageType, 'sensor_msgs/LaserScan')
         if strcmp(msgs{i,1}.Header.FrameId, 'back_laser_link')
            figure(1)
            plot(msgs{i,1})
            title('Back scan - Lidar')
            pause(5/1000)
         end
         if strcmp(msgs{i,1}.Header.FrameId, 'front_laser_link')
            figure(2)
            plot(msgs{i,1})
            title('Front laser - Lidar')
            pause(5/1000)
         end
    end
end