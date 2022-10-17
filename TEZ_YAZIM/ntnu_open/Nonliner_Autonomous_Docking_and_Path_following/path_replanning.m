function [] = path_replanning(Rm,x_c_obstacle, y_c_obstacle,weight,wp_index,p)

global wpt obstacle_list path;

y_obstacle= (y_c_obstacle-Rm):(y_c_obstacle+Rm);
x_obstacle=(x_c_obstacle-Rm):(x_c_obstacle + Rm);

x_start = round(p(1));
y_start= round(p(2));

x_max=max(wpt.pos.x);
y_max=max(wpt.pos.y);

space_x=round((wpt.pos.x(wp_index+1)-x_c_obstacle)/2);
space_y=round((wpt.pos.y(wp_index+1)-y_c_obstacle)/2);

x_end=wpt.pos.x(wp_index+1)- space_x;
y_end=wpt.pos.y(wp_index+1)-space_y;

obstacle = [x_obstacle;y_obstacle];

path= Astar(x_max,y_max,[x_start y_start],[x_end y_end],obstacle,weight);

if ~isempty(path)
    
    x= path(:,2);
    y=path(:,1);
    dangle=diff(diff(y)./diff(x));
    
    %Find vertices in the path found
    index = find(abs(dangle)>0.1)+1;
    
    %Include the start point as a waypoint
    if length(index)>2
        if sqrt((x_start-path(index(1),1))^2 - (y_start-path(index(1),2))^2)>0
            index = [1; index];
        end
    end
    
    %Update waypoints and obstacle list
    if ~isempty(index)
        if ~all([wpt.pos.x;wpt.pos.y]==[path(index(1),1)';path(index(1),2)'])
            wpt.pos.y=[wpt.pos.y(1:wp_index) path(index,2)' wpt.pos.y((wp_index+1):end)];
            wpt.pos.x=[wpt.pos.x(1:wp_index) path(index,1)' wpt.pos.x((wp_index+1):end)];
            
            obstacle_list = [obstacle_list, [x_c_obstacle;y_c_obstacle]];
        end
    end
end
end