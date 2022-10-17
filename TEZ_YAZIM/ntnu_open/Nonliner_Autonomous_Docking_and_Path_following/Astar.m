function [path]= Astar(xmax,ymax,start,goal,obstacle,w)

% Author:    Anthony Chrabieh
% Date:      2017-11-06
% Revisions: 2020-03-13

MAP = zeros(xmax,ymax);

%Place obstacles in the map
for i=1:2:size(obstacle,1)
    MAP(obstacle(i,:),obstacle(i+1,:))=inf;
end

%Heuristic Weight
weight = sqrt(w);

%Heuristic Map of all nodes
for x = 1:size(MAP,1)
    for y = 1:size(MAP,2)
        if(MAP(x,y)~=inf)
            H(x,y) = weight*norm(goal-[x,y]);
            G(x,y) = inf;
        end
    end
end

%% initial conditions
G(start(1),start(2)) = 0;
F(start(1),start(2)) = H(start(1),start(2));

closedNodes = [];
openNodes = [start G(start(1),start(2)) F(start(1),start(2)) 0]; %[x y G F cameFrom]

%% Solve
solved = false;

while(~isempty(openNodes))
    
    %Find node from open set with smallest F value
    [~,I] = min(openNodes(:,4));
    
    %Set current node
    current = openNodes(I,:);
    
    %If goal is reached, break the loop
    if(current(1:2)==goal)
        closedNodes = [closedNodes;current];
        solved = true;
        break;
    end
    
    %remove current node from open set and add it to closed set
    openNodes(I,:) = [];
    closedNodes = [closedNodes;current];
    
    %For all neighbors of current node
    for x = current(1)-1:current(1)+1
        for y = current(2)-1:current(2)+1
            
            %If out of range skip
            if (x<1||x>xmax||y<1||y>ymax)
                continue
            end
            
            %If object skip
            if (isinf(MAP(x,y)))
                continue
            end
            
            %If current node skip
            if (x==current(1)&&y==current(2))
                continue
            end
            
            %If already in closed set skip
            skip = 0;
            for j = 1:size(closedNodes,1)
                if(x == closedNodes(j,1) && y==closedNodes(j,2))
                    skip = 1;
                    break;
                end
            end
            if(skip == 1)
                continue
            end
            
            A = [];
            %Check if already in open set
            if(~isempty(openNodes))
                for j = 1:size(openNodes,1)
                    if(x == openNodes(j,1) && y==openNodes(j,2))
                        A = j;
                        break;
                    end
                end
            end
            
            newG = G(current(1),current(2)) + round(norm([current(1)-x,current(2)-y]),1);
            
            %If not in open set, add to open set
            if(isempty(A))
                G(x,y) = newG;
                newF = G(x,y) + H(x,y);
                
                newNode = [x y G(x,y) newF size(closedNodes,1)];
                openNodes = [openNodes; newNode];
                continue
            end
            
            %If no better path, skip
            if (newG >= G(x,y))
                continue
            end
            
            G(x,y) = newG;
            
            newF = newG + H(x,y);
            openNodes(A,3:5) = [newG newF size(closedNodes,1)];
        end
    end
end

if (solved)
    
    j = size(closedNodes,1);
    path = [];
    while(j > 0)
        x = closedNodes(j,1);
        y = closedNodes(j,2);
        j = closedNodes(j,5);
        path = [x,y;path];
    end
else
    path= [];
end
end