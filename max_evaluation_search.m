function [UAV_next_state]  = max_evaluation_search(UAV_state,env)
    %UNTITLED2 Summary of this function goes here
   %   Detailed explanation goes here

if(UAV_state.target_position_x~=0&&UAV_state.target_position_y~=0&&sqrt((UAV_state.position_x-UAV_state.target_position_x)^2+(UAV_state.position_y-UAV_state.target_position_y)^2)>0.5)
    vector = [UAV_state.target_position_x,UAV_state.target_position_y] - [UAV_state.position_x,UAV_state.position_y];
    norm_vector=vector/norm(vector);
    UAV_state.next_position_x = UAV_state.position_x + norm_vector(1);
    UAV_state.next_position_y = UAV_state.position_y + norm_vector(2);
else
    tmp_map = env.visited_map.*env.distribution_map;
    [y, x] = find(tmp_map==max(max(tmp_map)));
    maxScore = 0;
    maxIndex = 1;
    for i = 1:length(x)
        [target_position_x, target_position_y] = env.grid_to_xy(x(i),y(i));
        vector = [target_position_x,target_position_y] - [UAV_state.position_x,UAV_state.position_y];
        norm_vector=vector/norm(vector);
        tmp_x = UAV_state.position_x;
        tmp_y = UAV_state.position_y;
        score = 0;
        step = 1/env.resolution;
        while(tmp_x<target_position_x&&tmp_y<target_position_y)
            [tmp_gird_x,tmp_grid_y]=env.xy_to_grid(tmp_x,tmp_y);
            score = score + env.distribution_map(tmp_grid_y,tmp_gird_x);
            tmp_x = tmp_x + step*norm_vector(1);
            tmp_y = tmp_y + step*norm_vector(2);
        end
        if(score>maxScore)
            maxScore = score;
            maxIndex = i;
        end
    end
    [target_position_x, target_position_y] = env.grid_to_xy(x(maxIndex),y(maxIndex));
    vector = [target_position_x,target_position_y] - [UAV_state.position_x,UAV_state.position_y];
    norm_vector=vector/norm(vector);
    UAV_state.next_position_x = UAV_state.position_x + norm_vector(1);
    UAV_state.next_position_y = UAV_state.position_y + norm_vector(2);
    UAV_state.target_position_x = target_position_x;
    UAV_state.target_position_y = target_position_y;
end
UAV_next_state = UAV_state;
end