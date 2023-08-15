function [UAV_next_state]  = singleUAV(UAV_state,env)
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
    [max_y, max_x] = find(tmp_map==max(max(tmp_map)));
    %[x,y] = env.xy_to_grid(UAV_state.position_x,UAV_state.position_y);
    current_pos = [UAV_state.position_x,UAV_state.position_y];
    [height, width] = size(tmp_map);
    [prob,path] = look_ahead(tmp_map,current_pos,2,max_y,max_x,env);
    if(length(path)==0)
        [target_position_x, target_position_y] = env.grid_to_xy(max_x(1),max_y(1));
    else
        [target_position_x, target_position_y] = env.grid_to_xy(path(3,1),path(3,2));
    end
    %UAV_state.path = path(3:6,:);
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

function [directions] = sample_direction(prob_map, current_pos, number,env)
    direction_number = 30;
    direction_value = [];
    direction_space = [];
    for angle = 1: 360/direction_number: 360
        stepy = sin(angle);
        stepx = cos(angle);
        posx = current_pos(1) +stepx;
        posy = current_pos(2) +stepy;
        [pos_grid_x,pos_grid_y] = env.xy_to_grid(posx,posy);
        value = 0;
        while(pos_grid_x<env.width&&pos_grid_y<env.length&&pos_grid_x>1&&pos_grid_y>1)
            value = value + prob_map(pos_grid_y,pos_grid_x);
            posx = posx +stepx;
            posy = posy +stepy;
            [pos_grid_x,pos_grid_y] = env.xy_to_grid(posx,posy);
        end
        direction_value = [direction_value,value];
        direction_space = [direction_space,angle];
    end
    tmp_value = normalize(direction_value,"norm",1);
    direction_value(1) = tmp_value(1);
    for i = 2 : length(direction_value)
        direction_value(i) = direction_value(i-1)+tmp_value(i);
    end
    directions = [];
    for i = 1:number
        rr = rand();
        tmp = find(direction_value>rr);
        index = tmp(1);
        directions = [directions,direction_space(index)];
    end
end
function [max_prob, best_path] = look_ahead(prob_map, current_pos, steps,max_y,max_x,env)
    if steps == 0
        [current_pos_grid_x,current_pos_grid_y] = env.xy_to_grid(current_pos(1),current_pos(2));
        x_l = max(1,current_pos_grid_x-30);
        x_r = min(env.width,current_pos_grid_x+30);
        y_u = max(1,current_pos_grid_y-30);
        y_d = min(env.length,current_pos_grid_y+30);
        tmp_prob = prob_map(y_u:y_d, x_l:x_r);
        dis = [current_pos_grid_y,current_pos_grid_x]-[max_y,max_x];
        dis_prob = min(sqrt(dis(:,1).^2+dis(:,2).^2));
        max_prob = sum(sum(tmp_prob));%+2*dis_prob;
        best_path = [current_pos_grid_x,current_pos_grid_y];
    else
        max_prob = 0;
        best_path = [];
        number = 9;
        directions = sample_direction(prob_map, current_pos, number,env);
        for i = 1: number
            dy = sin(directions(i));
            dx = cos(directions(i));
            next_pos = current_pos + [dx, dy];
            [next_pos_grid_x,next_pos_grid_y] = env.xy_to_grid(next_pos(1),next_pos(2));
            if next_pos_grid_x >= 1 && next_pos_grid_x <= env.length && next_pos_grid_y >= 1 && next_pos_grid_y <= env.width
                [prob, path] = look_ahead(prob_map, next_pos, steps - 1,max_y,max_x,env);
                if prob > max_prob
                    max_prob = prob;
                    best_path = [current_pos; path];
                end
            end
        end
    end
end

