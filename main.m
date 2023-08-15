
aa = env(100);
UAV_state = struct;
UAV_state.position_x = 1;
UAV_state.position_y = 1;
UAV_state.next_position_x = 1;
UAV_state.next_position_y = 1;
UAV_state.target_position_x = 0;
UAV_state.target_position_y = 0;
UAV_state.path = [];
item = 0;
history=[];
while(length(aa.distribute_x)>=1&&item<10000&&max(max(aa.visited_map))==1)
    %cla;
%     if(item==4000)
%         ansofno =1;
%     end
    [UAV_state] = singleUAV(UAV_state,aa);
    aa = aa.update(UAV_state.next_position_x,UAV_state.next_position_y);
    history = [history;UAV_state.next_position_x,UAV_state.next_position_y];
    %aa.plot();
    %hold on;
    %imshow(visited_map.*aa.distribution_map);
    UAV_state.position_x = UAV_state.next_position_x;
    UAV_state.position_y = UAV_state.next_position_y;
    %pause(0.05);
    item = item +1;
    length(aa.distribute_x)
    item
end
%history = aa.xy_to_grid(history(:,1),history(:,2));
plot(history(:,1),history(:,2))


% x=1:1:150;
% 
% p=binopdf(x,250,0.48)*10;
% 
% % plot(x/100-0.9,p);
% % axis([0.14 1 0 0.6])
% % title('二项分布')
% 

