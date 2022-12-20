Data = load('./dataset/Data7.txt');
Data = single(Data);
ptCloud = pointCloud(Data(:,1:3));
pcwrite(ptCloud, 'test.pcd', 'Encoding', 'ascii'); %将程序中的xyz数据写入pcd文件中
pc = pcread('test.pcd');
pcshow(pc); %显示点云

Data = load('./dataset/Data7.txt');
Data = single(Data);
ptCloud = pointCloud(Data(:,1:3));
pcwrite(ptCloud, 'test.pcd', 'Encoding', 'ascii'); %将程序中的xyz数据写入pcd文件中
pc = pcread('test.pcd');
pcshow(pc); %显示点云
