clc
close all
clear
%% Variables to define
numRuns=30;

%% Import of data
tic
%Find longest vector
vecLength=0;
for i=1:numRuns
    filename = strcat('run_2layer_test',num2str(i),'-0_MiR_Robot_LBrain-tag-Environment_Cumulative Reward.csv');
    temp_data=importdata(filename);
    if size(temp_data.data,1)>vecLength
        vecLength=size(temp_data.data,1);
    end
end

%Import data to zero-padded matrix
data=zeros(vecLength,2*numRuns);
for i=1:numRuns
    filename = strcat('run_2layer_test',num2str(i),'-0_MiR_Robot_LBrain-tag-Environment_Cumulative Reward.csv');
    temp_data=importdata(filename);
    data(1:length(temp_data.data),2*i)=temp_data.data(:,2);
    data(1:length(temp_data.data),2*i+1)=temp_data.data(:,3);
end

disp('Import of data took:')
toc

%% Data processing
tic

% Find number of steps
test_data=[0 2500 5 2500 5;0 5000 6 5000 6;0 7500 6 7500 6;0 10000 5 10000 5;0 12500 4 12500 4;0 15000 5 15000 5;0 17500 5 17500 6;0 20000 6 20000 6;0 22500 6 22500 6;0 25000 6 25000 6;0 27500 6 0 0];


for i=1:numRuns
    for j=1:vecLength
        nOver6=0;
        for k=j:j+7   
            if k==200 %Reached end of steps
                break
            end
            if data(k,2*i+1) >= 6
                nOver6=nOver6+1;
            end
            if nOver6 >= 4
                output(i)=data(k,2*i);
                break
            end
        end
    end
end

disp('Data processomg took:')
toc

mean(output)