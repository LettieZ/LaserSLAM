%��LiDARd��idx��ɨ�����ݴӼ�����ת��Ϊ�ѿ�������(�����С���ľֲ�����)
% Read a laser scan
function scan = ReadAScan(lidar_data, idx, lidar, usableRange)
%--------------------------------------------------------------------------
% ����:
%lidar_dataΪ��ȡ��LiDARɨ������
%idxΪɨ�����������ֵ
%lidarΪ��SetLidarParameters()���õ�LiDAR����
%usableRangeΪ��ʹ�õķ�Χ
%--------------------------------------------------------------------------
    angles = lidar.angles;%
    ranges = lidar_data.ranges(idx, :)';%ѡȡLiDAR���ݵ�ranges��idx������Ӧ�����ɨ�������
    % ɾ����Χ��̫�ɿ��ĵ�
    % Remove points whose range is not so trustworthy
    maxRange = min(lidar.range_max, usableRange);
    isBad = ranges < lidar.range_min | ranges > maxRange;%ranges��С����С�ǶȻ�������Ƕȵ� ���ݵ� �����±�
    angles(isBad) = [];
    ranges(isBad) = [];
    % �Ӽ�����ת��Ϊ�ѿ�������
    % Convert from polar coordinates to cartesian coordinates
    [xs, ys] = pol2cart(angles, ranges);%(angles, ranges)Ϊ�������е�(theta,rho)
    scan = [xs, ys];  
end