function[RO ,obstacle] = obstacle_generator(R_min,R_max,num_obstacles)
global wpt

num_wpt= length(wpt.pos.x);

obstacle=zeros(2,num_obstacles);
RO = zeros(1,num_obstacles);
index= sort(randperm(num_wpt-2,num_obstacles)+1);

for i=1:num_obstacles
    space_y=(wpt.pos.y(index(i)+1)-wpt.pos.y(index(i)))/2;
    space_x=(wpt.pos.x(index(i)+1)-wpt.pos.x(index(i)))/2;
    
    obstacle_y=randi(sort([wpt.pos.y(index(i))+round(space_y/1) wpt.pos.y(index(i)+1)-space_y]));
    obstacle_x=randi(sort([wpt.pos.x(index(i))+round(space_x/1) wpt.pos.x(index(i)+1)-space_x]));
    
    RO(i)=randi([R_min,R_max]);
    obstacle(:,i)= [obstacle_y; obstacle_x];
end

end