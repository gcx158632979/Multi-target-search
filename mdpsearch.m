function [UAV_next_state]  = mdpsearch(UAV_state,env)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
if(UAV_state.target_position_x~=0&&UAV_state.target_position_y~=0&&sqrt((UAV_state.position_x-UAV_state.target_position_x)^2+(UAV_state.position_y-UAV_state.target_position_y)^2)>0.5)
    vector = [UAV_state.target_position_x,UAV_state.target_position_y] - [UAV_state.position_x,UAV_state.position_y];
    norm_vector=vector/norm(vector);
    UAV_state.next_position_x = UAV_state.position_x + norm_vector(1);
    UAV_state.next_position_y = UAV_state.position_y + norm_vector(2);
else
    tmp_map = env.visited_map.*env.distribution_map;
    %tmp_map = env.distribution_map;
    [x,y] = env.xy_to_grid(UAV_state.position_x,UAV_state.position_y);
    current_pos = [x,y];
    [height, width] = size(tmp_map);
    [prob,path] = look_ahead(tmp_map,current_pos,5,height, width);
    [target_position_x, target_position_y] = env.grid_to_xy(path(6,1),path(6,2));
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

function [max_prob, best_path] = look_ahead(prob_map, current_pos, steps, height, width)
    if steps == 0
        x_l = max(1,current_pos(1)-30);
        x_r = min(width,current_pos(1)+30);
        y_u = max(1,current_pos(2)-30);
        y_d = min(height,current_pos(2)+30);
        tmp_prob = prob_map(y_u:y_d, x_l:x_r);
        max_prob = sum(sum(tmp_prob));
        best_path = current_pos;
    else
        max_prob = -1;
        best_path = [];
        

        for dx = -1:1
            for dy = -1:1
                next_pos = current_pos + [dx, dy];

                if next_pos(1) >= 1 && next_pos(1) <= height && next_pos(2) >= 1 && next_pos(2) <= width
                    [prob, path] = look_ahead(prob_map, next_pos, steps - 1,height,width);

                    if prob > max_prob
                        max_prob = prob;
                        best_path = [current_pos; path];
                    end
                end
            end
        end
    end
end

