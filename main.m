%主函数
clear; close all; clc;
%cfig = figure(1);
cfig = figure('Position', [10,10,1280,1080]);
%% 激光雷达的传感器参数
lidar = SetLidarParameters();
%% 地图参数
borderSize      = 1;            % 边界尺寸
pixelSize       = 0.05;         % 栅格地图的一个单元的边长 对应 实际距离pixelSize米(这里设置为0.05米)
miniUpdated     = false;        % 
miniUpdateDT    = 0.2;          % 单位m   若机器人在x方向或y方向移动超过miniUpdateDT则更新位姿
miniUpdateDR    = deg2rad(10);  % 单位rad 若机器人旋转超过miniUpdateDR 则更新位姿 
% 运动滤波：如果机器人从最后一次键扫描移动了0.1米或旋转了5度，我们将添加一个新的键扫描并更新地图

%% 扫描匹配参数
fastResolution  = [0.2; 0.2; deg2rad(20)]; % [m; m; rad]的分辨率 快速搜索窗口[0.2m,20rad]
bruteResolution = [0.01; 0.01; deg2rad(0.1)]; % not used

%% 读取激光雷达数据
% lidar_data = load('./dataset/horizental_lidar.mat');
% N = size(lidar_data.timestamps, 1);%扫描次数(控制下面的循环次数)
lidar_data = load('./dataset/new_laser_data1.mat');
N = size(lidar_data.times, 1);

%% 构造一个空全局地图
map.points = [];%地图点集
map.connections = [];
map.keyscans = [];%keyscans保存当前正确位姿的扫描数据 如果预测得到的下一位姿出现错误 则返回到距其最近的前一位姿重新计算
pose = [0; 0; 0];%初始位姿为(x=0,y=0,theta=0)
path = pose;%位姿并置构成路径

%是否将绘制过程保存成视频
saveFrame=0;
if saveFrame==1
    % 视频保存文件定义与打开
    writerObj=VideoWriter('SLAMprocess.avi');  % 定义一个视频文件用来存动画  
    open(writerObj);                    % 打开该视频文件
end

%% Here we go!!!!!!!!!!!!!!!!!!!!
for scanIdx = 1 : 1 : N
    disp(['scan ', num2str(scanIdx)]);
    
    % 得到当前的扫描 [x1,y1; x2,y2; ...]
    %time = lidar_data.timestamps(scanIdx) * 1e-9;%时间设置成每1e-9扫描一次
    scan = ReadAScan(lidar_data, scanIdx, lidar, 6);%最远距离6米;得到该次扫描数据的局部笛卡尔坐标

    % 如果是第一次扫描 则初始化
    if scanIdx == 1
        map = Initialize(map, pose, scan);%把扫描数据scan坐标 通过位姿pose 转换为全局地图map坐标
        miniUpdated = true;   % false
        continue;
    end

    % 1. 如果我们在最后一步执行了 mini更新，我们将更新 局部点集图 和 局部栅格地图（粗略）
    % 1. If we executed a mini update in last step, we shall update the local points map and local grid map (coarse)
    if miniUpdated
%         %误差逐渐累积，最终定位出错
%         scan_temp = ReadAScan(lidar_data, scanIdx-1, lidar, 6);
%         pose_temp = pose;  % pose_guess   pose[scanIdx-1]？
%         scan_temp_w = Transform(scan_temp, pose_temp);
%         map.points = scan_temp_w;
%         localMap = ExtractLocalMap(map.points, pose_temp, scan, borderSize);%得到当前扫描的全局坐标
%         
        localMap = ExtractLocalMap(map.points, pose, scan, borderSize);%得到当前扫描的全局坐标
        gridMap1 = OccuGrid(localMap, pixelSize);%从点集localMap 栅格单元尺寸对应实际长度以pixelSize 创建占用栅格地图
        gridMap2 = OccuGrid(localMap, pixelSize/2);%从点集localMap 栅格单元尺寸对应实际长度以pixelSize/2 创建占用栅格地图
    end
    
    % 2. 使用恒定速度运动模型预测当前位姿(即用前一状态到本状态的过程 作为本状态到下一状态的过程 从而由本状态预测下一状态)
    if scanIdx > 2
% scan-scan 相邻两帧之间的匹配
%         scan_temp = ReadAScan(lidar_data, scanIdx-1, lidar, 6);
%         pose_temp = pose;
%         scan_temp_w = Transform(scan_temp, pose_temp);
%         map.points = scan_temp_w;
%         localMap = ExtractLocalMap(map.points, pose_temp, scan_temp, borderSize);
        
% scan-map的匹配
%         localMap = ExtractLocalMap(map.points, pose, scan, borderSize);%得到当前扫描的全局坐标
%         gridMap1 = OccuGrid(localMap, pixelSize);%从点集localMap 栅格单元尺寸对应实际长度以pixelSize 创建占用栅格地图
%         gridMap2 = OccuGrid(localMap, pixelSize/2);
          pose_guess = pose + DiffPose(path(:,end-1), pose);%预测下一位姿=当前位姿+(当前位姿与上一位姿的差) pose是一个全局坐标
    else
% scan-scan的匹配
%         scan_temp = ReadAScan(lidar_data, scanIdx-1, lidar, 6);
%         pose_temp = pose;
%         scan_temp_w = Transform(scan_temp, pose_temp);
%         map.points = scan_temp_w;
%         localMap = ExtractLocalMap(map.points, pose_temp, scan_temp, borderSize);%得到当前扫描的全局坐标

% scan-map的匹配
%         localMap = ExtractLocalMap(map.points, pose, scan, borderSize);%得到当前扫描的全局坐标
%         gridMap1 = OccuGrid(localMap, pixelSize);%从点集localMap 栅格单元尺寸对应实际长度以pixelSize 创建占用栅格地图
%         gridMap2 = OccuGrid(localMap, pixelSize/2);
        
        pose_guess = pose;
    end
    
    % 3. 快速匹配
    if miniUpdated
        [pose, ~] = FastMatch(gridMap1, scan, pose_guess, fastResolution);%根据当前栅格地图 优化 预测的下一位姿
    else
        [pose, ~] = FastMatch(gridMap2, scan, pose_guess, fastResolution);
    end
    
    % 4. 使用较高的分辨率再细化 预测下一位姿
    % gridMap = OccuGrid(localMap, pixelSize/2);
    [pose, hits] = FastMatch(gridMap2, scan, pose, fastResolution/2);%返回进一步更新的下一位姿pose
    
    % 如果机器人移动了一定距离，则执行mini更新
    dp = abs(DiffPose(map.keyscans(end).pose, pose));%两次位姿的差
    if dp(1)>miniUpdateDT || dp(2)>miniUpdateDT || dp(3)>miniUpdateDR
        miniUpdated = true;
        [map, pose] = AddAKeyScan(map, gridMap2, scan, pose, hits,...
                        pixelSize, bruteResolution, 0.1, deg2rad(3));
    else
        miniUpdated = false;
    end
    
    path = [path, pose]; %把当前位姿pose 并入路径path     
    
    % ===== Loop Closing =========================================
    % if miniUpdated
    %     if TryLoopOrNot(map)
    %         map.keyscans(end).loopTried = true;
    %         map = DetectLoopClosure(map, scan, hits, 4, pi/6, pixelSize);
    %     end
    % end
    %----------------------------------------------------------------------
    
    % 绘图
    if mod(scanIdx, 5) == 0%每30步画一次图
        PlotMap(cfig, map, path, scan, scanIdx);
        pause(0.3);%暂停0.3秒。
        %获取视频帧并保存成视频
        if saveFrame==1
            frame = getframe(cfig);
            writeVideo(writerObj, frame);
        end
    end
end
if saveFrame==1
    close(writerObj); %关闭视频文件句柄 
end
