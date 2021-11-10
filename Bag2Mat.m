clear;clc;

% ��ros�����н�ѹ��ԭʼbag�ļ�:rosbag decompress 1.bag

%��ȡ��ѹ���bag�ļ�����
bag = rosbag('1.orig.bag');
%��ȡˮƽ�״�topic ����
 laser = select(bag, 'Time', ...
            [bag.StartTime bag.EndTime], 'Topic', '/horizontal_laser_2d');
        
%% ���ļ��в������ݵĴ�С 
N = laser.NumMessages;%�״���������
x = readMessages(laser,1);
[M,~] = size(x{1,1}.Ranges);
times = zeros(N,1);%ʱ�����
ranges = zeros(N,M);%�������

%% ѭ����ȡ���� �������ȡʱ������ڴ治������
for i=1:N
    temp = readMessages(laser,i);
    times(i) = temp{1,1}.Header.Stamp.Sec;%ʱ������
    ranges_temp = temp{1,1}.Ranges;%�״�������ݣ�1079ά���ݣ�
    ranges(i,:) = ranges_temp;
    %��ʾ����
    if mod(i,100)==0
        disp(['�������%��', num2str(i/N*100)]);
    end
end
%���ݱ���Ϊmat�ļ�
save new_laser_data.mat times ranges