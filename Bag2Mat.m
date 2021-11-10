clear;clc;

% 在ros环境中解压缩原始bag文件:rosbag decompress 1.bag

%读取解压后的bag文件数据
bag = rosbag('1.orig.bag');
%读取水平雷达topic 数据
 laser = select(bag, 'Time', ...
            [bag.StartTime bag.EndTime], 'Topic', '/horizontal_laser_2d');
        
%% 从文件中查找数据的大小 
N = laser.NumMessages;%雷达数据条数
x = readMessages(laser,1);
[M,~] = size(x{1,1}.Ranges);
times = zeros(N,1);%时间参数
ranges = zeros(N,M);%距离参数

%% 循环读取数据 ：整体读取时会出现内存不足的情况
for i=1:N
    temp = readMessages(laser,i);
    times(i) = temp{1,1}.Header.Stamp.Sec;%时间数据
    ranges_temp = temp{1,1}.Ranges;%雷达测量数据（1079维数据）
    ranges(i,:) = ranges_temp;
    %显示进度
    if mod(i,100)==0
        disp(['处理进度%：', num2str(i/N*100)]);
    end
end
%数据保存为mat文件
save new_laser_data.mat times ranges