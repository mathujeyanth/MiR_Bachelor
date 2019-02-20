
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


%%
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