function [UAV_next_state]  = max_search(UAV_state,env)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% [x_l,y_d] = env.xy_to_grid(UAV_state.position_x-5,UAV_state.position_y-5);
% [x_r,y_u] = env.xy_to_grid(UAV_state.position_x+5,UAV_state.position_y+5);
% UAV_state.visited_map(y_u:y_d,x_l:x_r) = 0;
if(UAV_state.target_position_x~=0&&UAV_state.target_position_y~=0&&sqrt((UAV_state.position_x-UAV_state.target_position_x)^2+(UAV_state.position_y-UAV_state.target_position_y)^2)>0.5)
    vector = [UAV_state.target_position_x,UAV_state.target_position_y] - [UAV_state.position_x,UAV_state.position_y];
    norm_vector=vector/norm(vector);
    UAV_state.next_position_x = UAV_state.position_x + norm_vector(1);
    UAV_state.next_position_y = UAV_state.position_y + norm_vector(2);
else
    tmp_map = env.visited_map.*env.distribution_map;
    [y, x] = find(tmp_map==max(max(tmp_map)));
    [target_position_x, target_position_y] = env.grid_to_xy(x(randperm(length(x), 1)),y(randperm(length(y), 1)));
    %[target_position_x, target_position_y] = env.grid_to_xy(x(1),y(1));
    vector = [target_position_x,target_position_y] - [UAV_state.position_x,UAV_state.position_y];
    norm_vector=vector/norm(vector);
    UAV_state.next_position_x = UAV_state.position_x + norm_vector(1);
    UAV_state.next_position_y = UAV_state.position_y + norm_vector(2);
    UAV_state.target_position_x = target_position_x;
    UAV_state.target_position_y = target_position_y; 
end
UAV_next_state = UAV_state;
end