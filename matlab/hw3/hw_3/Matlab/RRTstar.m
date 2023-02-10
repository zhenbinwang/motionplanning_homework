%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% 流程初始化
clear all; close all;clc;
x_I=1; y_I=1;           % 设置初始点
x_G=700; y_G=700;       % 设置目标点
Thr=50;                 %设置目标点阈值
Delta= 30;              % 设置扩展步长
RadiusForNeib = 80;     % rewire的范围,半径r
%% 建树初始化
T.v(1).x = x_I;         % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
T.v(1).y = y_I;
T.v(1).xPrev = x_I;     % 起始节点的父节点仍然是其本身
T.v(1).yPrev = y_I;
T.v(1).cost = 0;        %从父节点到该节点的距离，这里可取欧氏距离
T.v(1).indPrev = 0;     %
%% 开始构建树——作业部分
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%地图x轴长度
yL=size(Imp,2);%地图y轴长度
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% 绘制起点和目标点
count=1;
findPath = 0;
update_count = 0;
path.pos = [];
UpdateTime = 50;            % 更新路径的时间间隔
pHandleList = [];
lHandleList = [];
resHandleList = [];
for iter = 1:3000
    x_rand=[];
    %Step 1: 在地图中随机采样一个点x_rand
    %提示：用（x_rand(1),x_rand(2)）表示环境中采样点的坐标
    x_rand(1) = unidrnd(x_G+100);
    x_rand(2) = unidrnd(y_G+100);
    
    x_near=[];
    %Step 2: 遍历树，从树中找到最近邻近点x_near
    %提示：x_near已经在树T里
    
    dis_min = 100000;
    index_min = 0;
    for i=1:size(T.v,2)
        dis = sqrt((T.v(i).x - x_rand(1))^2 + (T.v(i).y - x_rand(2))^2);
        if dis_min > dis
            dis_min = dis;
            index_min = i;
        end
    end
    
    x_near(1)=T.v(index_min).x;
    x_near(2)=T.v(index_min).y;
    
    x_new=[];
    %Step 3: 扩展得到x_new节点
    %提示：注意使用扩展步长Delta
    
    x_new(1) =x_near(1) + (x_rand(1) - x_near(1))*Delta/dis_min;
    x_new(2) =x_near(2) + (x_rand(2) - x_near(2))*Delta/dis_min;
    
    %检查节点是否是collision-free
    if ~collisionChecking(x_near,x_new,Imp)
        continue;
    end
    
    %Step 4: 在以x_new为圆心,半径为R的圆内搜索节点 (NearC)
    nearIndexList = [];
    nearcost = [];
    index_near = 0;
    for i=1:count
        dis = sqrt((T.v(i).x - x_new(1))^2 + (T.v(i).y - x_new(2))^2);
        if dis < RadiusForNeib
            nearcost = [nearcost dis];
            nearIndexList = [nearIndexList i];
        end
    end
    
    %Step 5: 选择x_new的父节点,使x_new的累计cost最小 (ChooseParent)
    dis_min = 1e10;
    new_node = [];
    for i=1:length(nearcost)
        if dis_min > nearcost(i) +  T.v(nearIndexList(i)).cost
            mincost(1) = T.v(nearIndexList(i)).x;       % 起始节点的父节点仍然是其本身
            mincost(2) = T.v(nearIndexList(i)).y;
            if ~collisionChecking(mincost,x_new,Imp)
                continue;   %有障碍物
            end
            
            dis_min =  nearcost(i) +  T.v(nearIndexList(i)).cost ;
            index_parent = nearIndexList(i);
        end
    end
    
    count=count+1;
    
    %Step 6: 将x_new插入树T (AddNodeEdge)
    T.v(count).x = x_new(1);            % T是我们要做的树，v是节点，这里先把起始点加入到T里面来
    T.v(count).y = x_new(2);
    T.v(count).xPrev = T.v(index_parent).x;       % 起始节点的父节点仍然是其本身
    T.v(count).yPrev = T.v(index_parent).y;
    T.v(count).cost = dis_min;          % 从父节点到该节点的距离，这里可取欧氏距离
    T.v(count).indPrev = index_parent;     %
    
    
   l_handle = plot([T.v(count).xPrev, x_new(1)], [T.v(count).yPrev, x_new(2)], 'b', 'Linewidth', 2);
   p_handle = plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
   
   pHandleList = [pHandleList p_handle];    %绘图的句柄索引即为count
   lHandleList = [lHandleList l_handle];
   pause(0.1);
   
    %Step 7: 剪枝 (rewire)
    for i=1:length(nearIndexList)
        if(nearIndexList(i) ~= index_parent)
            dis = nearcost(i) + T.v(count).cost;
            if dis < T.v(nearIndexList(i)).cost
                x_neib(1) = T.v(nearIndexList(i)).x;     % 符合剪枝条件节点的坐标
                x_neib(2) = T.v(nearIndexList(i)).y;
                if ~collisionChecking(x_neib,x_new,Imp) 
                    continue;   %有障碍物
                end
                T.v(nearIndexList(i)).xPrev = x_new(1);      % 对该neighbor信息进行更新
                T.v(nearIndexList(i)).yPrev = x_new(2);
                T.v(nearIndexList(i)).cost = dis;
                T.v(nearIndexList(i)).indPrev = count;       % x_new的索引
                
                %delete(pHandleList());
                %delete(lHandleList(nearIndexList(rewire_index)));
                lHandleList(nearIndexList(i)) = plot([T.v(nearIndexList(i)).x, x_new(1)], [T.v(nearIndexList(i)).y, x_new(2)], 'r', 'Linewidth', 2);
            end
        end
    end

     %Step 8:检查是否到达目标点附近 
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < Thr && ~findPath)    % 找到目标点，此条件只进入一次
        findPath = 1;

        count = count+1;    %手动将Goal加入到树中
        Goal_index = count;
        T.v(count).x = x_G;          
        T.v(count).y = y_G; 
        T.v(count).xPrev = x_new(1);     
        T.v(count).yPrev = x_new(2);
        T.v(count).cost = T.v(count - 1).cost + disToGoal;
        T.v(count).indPrev = count - 1;     %其父节点x_near的index
    end
    
    if(findPath == 1)
        update_count = update_count + 1;
        if(update_count == UpdateTime)
            update_count = 0;
            j = 2;
            path.pos(1).x = x_G; 
            path.pos(1).y = y_G;
            pathIndex = T.v(Goal_index).indPrev;
            while 1     
                path.pos(j).x = T.v(pathIndex).x;
                path.pos(j).y = T.v(pathIndex).y;
                pathIndex = T.v(pathIndex).indPrev;    % 沿终点回溯到起点
                if pathIndex == 0
                    break
                end
                j=j+1;
            end  
            
            for delete_index = 1:length(resHandleList)
            	delete(resHandleList(delete_index));
            end
            for j = 2:length(path.pos)
                res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'g', 'Linewidth', 4);
                resHandleList = [resHandleList res_handle];
            end
        end
    end  
	pause(0.05); %暂停DelayTime s,使得RRT*扩展过程容易观察
end
for delete_index = 1:length(resHandleList)
	delete(resHandleList(delete_index));
end
for j = 2:length(path.pos)
	res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'g', 'Linewidth', 4);
	resHandleList = [resHandleList res_handle];
end
            
disp('The path is found!');


