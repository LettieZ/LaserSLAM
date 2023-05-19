
%% 预先工作
% 打包数据: rosbag record  /scan
% 查看打包信息: rosbag info *.bag 
% 在ros环境中解压缩原始bag文件: rosbag decompress *.bag

%% 读取数据
clear;clc;
% 读取解压后的bag文件数据
bag = rosbag('./dataset/new_laser_data0.bag');
% 读取topic数据并保存
laser = select(bag, 'Time', ...
              [bag.StartTime bag.EndTime], 'Topic', '/horizontal_laser_2d');          
% imu = select(bag, 'Time',[bag.StartTime bag.EndTime], 'Topic', '/imu');
% geometry_message=select(bag,'MessageType','sensor_msgs/LaserScan');
% odom_message=select(bag,'MessageType','nav_msgs/Odometry');

%% 从文件中查找数据的大小 
N = laser.NumMessages;%雷达数据条数
x = readMessages(laser,1);
[M,~] = size(x{1,1}.Ranges);
%[M,~] = size(x{1,1}.x);
times = zeros(N,1);%时间参数
ranges = zeros(N,M);%距离参数

%% 循环读取数据 ：整体读取时会出现内存不足的情况
for i=1:N
    temp = readMessages(laser,i);
    times(i) = temp{1,1}.Header.Stamp.Sec;%时间数据
    ranges_temp = temp{1,1}.Ranges;%雷达测量数据
    ranges(i,:) = ranges_temp;
    %显示进度
    if mod(i,100)==0
        disp(['处理进度%：', num2str(i/N*100)]);
    end
end

%% 数据保存为mat文件
save ./new_laser_data.mat times ranges

%% 可视化极坐标雷达点 https://blog.csdn.net/xi_shui/article/details/121293508
% LaserData = load('./new_laser_data.mat');
for i=1:N
    for j=1:1:360
    %         % ===== 与RVIZ odom坐标对应 ===========================================
    %         if j < 180
    %         polarscatter(-(180-j)*0.0174,laser_PointCloud{1,1}.Ranges(j),'.')
    %         hold on
    %         elseif j>= 180
    %         polarscatter((j-180)*0.0174,laser_PointCloud{1,1}.Ranges(j),'.')
    %         hold on
    %         end
        polarscatter(j,ranges(j,:),'.') % 注意:MATLAB的极坐标画点函数是弧度制
        %scatter3(xyz(:,1), xyz(:,2), xyz(:,3), '.');
        hold on
    end
end


%% 读取PointCloud2雷达数据
clear;clc;
filepath=fullfile('./dataset','0308.bag');
bag=rosbag(filepath);
% bagmessage = readMessages(bag);  %提取bag文件中的信息，查看属性

% PointCloud2数据包含的fields有：x，y, z, intensity, normal_x , normal_y, normal_z, curvature
laser_PointCloud = select(bag, 'Time', ...  % 选取需要的多个类型消息
                         [bag.StartTime bag.EndTime], 'Topic', '/horizontal_laser_2d_fromPointCloud2');
bagmessage = readMessages(laser_PointCloud);

xyz=readXYZ(bagmessage{1});      %提取Data中的xyz信息，得到的结果是xyz坐标
% intensity=readField(bagmess{1},'intensity'); %提取intensity信息

%% 读取坐标值,画图
scatter3(xyz(:,1), xyz(:,2), xyz(:,3), '.');
scatter(xyz(:,1),xyz(:,2))

position=zeros(180,3);
for i=1:360
    %====odom_message数据读取===
    % position(i,1)=data{i,1}.Pose.Pose.Position.X;
    % position(i,2)=data{i,1}.Pose.Pose.Position.Y;
    % position(i,3)=data{i,1}.Pose.Pose.Position.Z;
    position(i,1)=xyz(i,1);
    position(i,2)=xyz(i,2);
    position(i,3)=xyz(i,3);
end

% 画三维图
figure();
for i=1:180
    plot3(position(i,1),position(i,2),position(i,3),'r.','markersize',5);
    hold on
end

% 画二维图
figure();
for i=1:180
    plot(position(i,1),position(i,2),'r.','markersize',5);
    hold on
end
% https://blog.csdn.net/qq_35451217/article/details/88059535
