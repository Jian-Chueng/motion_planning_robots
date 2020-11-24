function path = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
    %pre-process the grid map, add offset
    size_map = size(map,1);
    %size_map:起点+终点+障碍物的总数
    %map:第一个是起点，最后一个是终点，其余是障碍物
    Y_offset = 0;
    X_offset = 0;
    
    %Define the 2D grid map array.
    %Obstacle=-1, Target = 0, Start=1
    MAP=2*(ones(MAX_X,MAX_Y));
    
    %Initialize MAP with location of the target
    xval=floor(map(size_map, 1)) + X_offset;
    yval=floor(map(size_map, 2)) + Y_offset;
    xTarget=xval;
    yTarget=yval;
    MAP(xval,yval)=0;
    
    %Initialize MAP with location of the obstacle
    for i = 2: size_map-1 %number of obstacle
        xval=floor(map(i, 1)) + X_offset;
        yval=floor(map(i, 2)) + Y_offset;
        MAP(xval,yval)=-1;
    end 
    
    %Initialize MAP with location of the start point
    xval=floor(map(1, 1)) + X_offset;
    yval=floor(map(1, 2)) + Y_offset;
    xStart=xval;
    yStart=yval;
    MAP(xval,yval)=1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
    %--------------------------------------------------------------------------
    OPEN=[];
    %CLOSED LIST STRUCTURE
    %--------------
    %X val | Y val |
    %--------------
    % CLOSED=zeros(MAX_VAL,2);
    CLOSED=[];

    %Put all obstacles on the Closed list
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k,1)=i;
                CLOSED(k,2)=j;
                k=k+1;
            end
        end
    end
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    xNode=xval;
    yNode=yval;
    OPEN_COUNT=1;
    hn=distance(xNode,yNode,xTarget,yTarget);
    gn=0;
    fn=hn+gn;
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,hn,gn,fn);
    OPEN(OPEN_COUNT,1)=1;% Note: here it is changed
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;
%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    while (NoPath==1 &&(xNode~=xTarget || yNode~=yTarget))
    
%     If the queue is empty, return FALSE; break;
        if (sum(OPEN(:,1))==0) 
            break; 
        end
%     Remove the node “n” with the lowest g(n) from the priority queue 弹出现在path cost最小的node
        i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
        xNode_expand = OPEN(i_min,2);
        yNode_expand = OPEN(i_min,3);
        gNode_expand = OPEN(i_min,7);
        OPEN(i_min,1)= 0; %marked as visted
    %      Mark node “n” as expanded 弹出的node存入CLOSED list
        CLOSED_COUNT = CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,1)=xNode_expand;
        CLOSED(CLOSED_COUNT,2)=yNode_expand;
    %     If the node “n” is the goal state, return TRUE; break;
        if xNode_expand == xTarget && yNode_expand == yTarget
            NoPath = 0;
            break;
        end
%     For all unexpanded neighbors “m” of node “n”
        exp_array=expand_array(xNode_expand,yNode_expand,gNode_expand,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
        % expand_array:|X val |Y val |h(n) |g(n)|f(n)|
        if ~isempty(exp_array)
            for m = 1:1:length(exp_array(:,1)) % For all unexpanded neighbors “m” of node “n”
                xNode = exp_array(m,1);
                yNode = exp_array(m,2);
                % If node m is not in OPEN, push node m into OPEN 
                if  isempty(node_index(OPEN,xNode,yNode)) 
                    OPEN_COUNT=OPEN_COUNT + 1;
                    hn=distance(xNode,yNode,xTarget,yTarget);
                    gn=gNode_expand + distance(xNode,yNode,xNode_expand,yNode_expand);
                    fn=hn + gn; 

                    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode_expand,yNode_expand,hn,gn,fn);                
                    OPEN(OPEN_COUNT,1)=1;  
                else
                    %If m Node is in OPEN and g(m) > g(n) + Cnm, which means a better path to this node is found
                    %then update parents and g , f value  
                    n_inx = node_index(OPEN, xNode,yNode);
                    if OPEN(n_inx,7) > (gNode_expand + distance(xNode,yNode,xNode_expand,yNode_expand))
                        OPEN(n_inx,4) = xNode_expand;  % Parent X val
                        OPEN(n_inx,5) = yNode_expand;  % Parent Y val
                        OPEN(n_inx,7) = gNode_expand + distance(xNode,yNode,xNode_expand,yNode_expand);  % g(m)
                        OPEN(n_inx,8) = OPEN(n_inx,6) + OPEN(n_inx,7);  % f(m)
                    end 

                end
                                    
            end
        end    
   
    
    end % End of While
    
    
    path=[];
    count=1;
    last_x=xTarget; %存入终点
    last_y=yTarget;
    %从终点开始，寻找当前节点的parent node，直到找到起点，形成path
    while(last_x ~= xStart || last_y ~= yStart)
        path(count,:)=[last_x,last_y];  %将当前点存入path中
        count=count+1;
        current_index=node_index(OPEN,last_x,last_y);
        last_x=OPEN(current_index,4);%找到当前节点的parent坐标
        last_y=OPEN(current_index,5);
    end
    path(count,:)=[xStart, yStart];
    

    
   
    
end