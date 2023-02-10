%***************************************
%Author: Chaoqun Wang
%Date: 2019-10-15
%***************************************
%% ���̳�ʼ��
clear all; close all;clc;
x_I=1; y_I=1;           % ���ó�ʼ��
x_G=700; y_G=700;       % ����Ŀ���
Thr=50;                 %����Ŀ�����ֵ
Delta= 30;              % ������չ����
RadiusForNeib = 80;     % rewire�ķ�Χ,�뾶r
%% ������ʼ��
T.v(1).x = x_I;         % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
T.v(1).y = y_I;
T.v(1).xPrev = x_I;     % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
T.v(1).yPrev = y_I;
T.v(1).cost = 0;        %�Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
T.v(1).indPrev = 0;     %
%% ��ʼ������������ҵ����
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,1);%��ͼx�᳤��
yL=size(Imp,2);%��ͼy�᳤��
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% ��������Ŀ���
count=1;
findPath = 0;
update_count = 0;
path.pos = [];
UpdateTime = 50;            % ����·����ʱ����
pHandleList = [];
lHandleList = [];
resHandleList = [];
for iter = 1:3000
    x_rand=[];
    %Step 1: �ڵ�ͼ���������һ����x_rand
    %��ʾ���ã�x_rand(1),x_rand(2)����ʾ�����в����������
    x_rand(1) = unidrnd(x_G+100);
    x_rand(2) = unidrnd(y_G+100);
    
    x_near=[];
    %Step 2: ���������������ҵ�����ڽ���x_near
    %��ʾ��x_near�Ѿ�����T��
    
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
    %Step 3: ��չ�õ�x_new�ڵ�
    %��ʾ��ע��ʹ����չ����Delta
    
    x_new(1) =x_near(1) + (x_rand(1) - x_near(1))*Delta/dis_min;
    x_new(2) =x_near(2) + (x_rand(2) - x_near(2))*Delta/dis_min;
    
    %���ڵ��Ƿ���collision-free
    if ~collisionChecking(x_near,x_new,Imp)
        continue;
    end
    
    %Step 4: ����x_newΪԲ��,�뾶ΪR��Բ�������ڵ� (NearC)
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
    
    %Step 5: ѡ��x_new�ĸ��ڵ�,ʹx_new���ۼ�cost��С (ChooseParent)
    dis_min = 1e10;
    new_node = [];
    for i=1:length(nearcost)
        if dis_min > nearcost(i) +  T.v(nearIndexList(i)).cost
            mincost(1) = T.v(nearIndexList(i)).x;       % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
            mincost(2) = T.v(nearIndexList(i)).y;
            if ~collisionChecking(mincost,x_new,Imp)
                continue;   %���ϰ���
            end
            
            dis_min =  nearcost(i) +  T.v(nearIndexList(i)).cost ;
            index_parent = nearIndexList(i);
        end
    end
    
    count=count+1;
    
    %Step 6: ��x_new������T (AddNodeEdge)
    T.v(count).x = x_new(1);            % T������Ҫ��������v�ǽڵ㣬�����Ȱ���ʼ����뵽T������
    T.v(count).y = x_new(2);
    T.v(count).xPrev = T.v(index_parent).x;       % ��ʼ�ڵ�ĸ��ڵ���Ȼ���䱾��
    T.v(count).yPrev = T.v(index_parent).y;
    T.v(count).cost = dis_min;          % �Ӹ��ڵ㵽�ýڵ�ľ��룬�����ȡŷ�Ͼ���
    T.v(count).indPrev = index_parent;     %
    
    
   l_handle = plot([T.v(count).xPrev, x_new(1)], [T.v(count).yPrev, x_new(2)], 'b', 'Linewidth', 2);
   p_handle = plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
   
   pHandleList = [pHandleList p_handle];    %��ͼ�ľ��������Ϊcount
   lHandleList = [lHandleList l_handle];
   pause(0.1);
   
    %Step 7: ��֦ (rewire)
    for i=1:length(nearIndexList)
        if(nearIndexList(i) ~= index_parent)
            dis = nearcost(i) + T.v(count).cost;
            if dis < T.v(nearIndexList(i)).cost
                x_neib(1) = T.v(nearIndexList(i)).x;     % ���ϼ�֦�����ڵ������
                x_neib(2) = T.v(nearIndexList(i)).y;
                if ~collisionChecking(x_neib,x_new,Imp) 
                    continue;   %���ϰ���
                end
                T.v(nearIndexList(i)).xPrev = x_new(1);      % �Ը�neighbor��Ϣ���и���
                T.v(nearIndexList(i)).yPrev = x_new(2);
                T.v(nearIndexList(i)).cost = dis;
                T.v(nearIndexList(i)).indPrev = count;       % x_new������
                
                %delete(pHandleList());
                %delete(lHandleList(nearIndexList(rewire_index)));
                lHandleList(nearIndexList(i)) = plot([T.v(nearIndexList(i)).x, x_new(1)], [T.v(nearIndexList(i)).y, x_new(2)], 'r', 'Linewidth', 2);
            end
        end
    end

     %Step 8:����Ƿ񵽴�Ŀ��㸽�� 
    disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2);
    if(disToGoal < Thr && ~findPath)    % �ҵ�Ŀ��㣬������ֻ����һ��
        findPath = 1;

        count = count+1;    %�ֶ���Goal���뵽����
        Goal_index = count;
        T.v(count).x = x_G;          
        T.v(count).y = y_G; 
        T.v(count).xPrev = x_new(1);     
        T.v(count).yPrev = x_new(2);
        T.v(count).cost = T.v(count - 1).cost + disToGoal;
        T.v(count).indPrev = count - 1;     %�丸�ڵ�x_near��index
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
                pathIndex = T.v(pathIndex).indPrev;    % ���յ���ݵ����
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
	pause(0.05); %��ͣDelayTime s,ʹ��RRT*��չ�������׹۲�
end
for delete_index = 1:length(resHandleList)
	delete(resHandleList(delete_index));
end
for j = 2:length(path.pos)
	res_handle = plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'g', 'Linewidth', 4);
	resHandleList = [resHandleList res_handle];
end
            
disp('The path is found!');


