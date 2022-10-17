function [chi_desired,e,wp_LOSindex] = LOS_avoidance(wp_LOSindex,p, WPx,WPy, R, delta,Rm,x_c_obstacle, y_c_obstacle,weight)
global wpt obstacle_list orig_wpt;
wp_dx = WPx(wp_LOSindex+1)-WPx(wp_LOSindex);
wp_dy = WPy(wp_LOSindex+1)-WPy(wp_LOSindex);

alfa_k = atan2(wp_dy, wp_dx);

%Cross-track error
e = -(p(1)-WPx(wp_LOSindex))*sin(alfa_k) + (p(2)-WPy(wp_LOSindex))*cos(alfa_k);

chi_r = atan(-e/abs(delta));

x_error = -WPx(wp_LOSindex+1) + p(1);
y_error = -WPy(wp_LOSindex+1) + p(2);

x_oa_error=p(1)-x_c_obstacle;
y_oa_error=p(2)-y_c_obstacle;

%Waypoint switching
if (x_error^2 + y_error^2 <= R^2) && wp_LOSindex<(length(wpt.pos.x)-1)
     wp_LOSindex = wp_LOSindex + 1;
end

%Generate waypoints when closer to obstacle than the next waypoint
if ((x_oa_error^2 + y_oa_error^2 <= x_error^2 + y_error^2) && ~any(all(obstacle_list ==[x_c_obstacle;y_c_obstacle])))
    path_replanning(Rm,x_c_obstacle, y_c_obstacle,weight,wp_LOSindex,p);
end

%Desired course
chi_desired = alfa_k + chi_r;
end