function [UAV_next_state] = zigzag(UAV_state,env)
%UNTITLED3 此处提供此函数的摘要
%   此处提供详细说明
%step = 5;
if(mod(UAV_state.position_y,2)==1)
    UAV_state.next_position_x = UAV_state.position_x - 1;
else
    UAV_state.next_position_x = UAV_state.position_x + 1;
end
UAV_state.next_position_y = UAV_state.position_y;
if(UAV_state.next_position_x>env.bound_x||UAV_state.next_position_x<1)
    UAV_state.next_position_y = UAV_state.position_y + 1;
    UAV_state.next_position_x = UAV_state.position_x;
end
if(UAV_state.next_position_y>env.bound_y)
    UAV_state.next_position_y = UAV_state.position_y;
end
UAV_next_state = UAV_state;
end