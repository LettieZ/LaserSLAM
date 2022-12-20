%�����״ﴫ��������
%Laser sensor's parameters
function lidar = SetLidarParameters()
 
% lidar.angle_min = -2.351831;%��Сɨ���
% lidar.angle_max =  2.351831;%���ɨ���
% lidar.angle_increment = 0.004363;%�Ƕ�����  ��lidar��������֮��ļн�
% lidar.npoints   = 1079;
% lidar.range_min = 0.023;
% lidar.range_max = 60;
% lidar.scan_time = 0.025;%ɨ��ʱ��
% lidar.time_increment  = 1.736112e-05;%ʱ������
% lidar.angles = (lidar.angle_min : lidar.angle_increment : lidar.angle_max)';%һ��ɨ��������ĽǶ�

lidar.angle_min = 0;%��Сɨ���
lidar.angle_max =  6.28;%���ɨ��� 360/180*PI
lidar.angle_increment = 0.01749;%�Ƕ�����  ��lidar��������֮��ļн� 6.28/(360-1)
lidar.npoints   = 360;
lidar.range_min = 0.001;
lidar.range_max = 6.0;
lidar.scan_time = 0.166;%ɨ��ʱ��
lidar.time_increment  = 0.00183;%ʱ������ 0.166/360=0.00183  4.157
lidar.angles = (lidar.angle_min : lidar.angle_increment : lidar.angle_max)';%һ��ɨ��������ĽǶ�
