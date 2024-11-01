classdef env
    properties
        Total_Number; 
        map;
        distribute_x = [];
        distribute_y = [];
        distribution_map;
        width;
        length;
        UAV_current_position_x;
        UAV_current_position_y;
        UAV_grid_position_x;
        UAV_grid_position_y;
        bound_x;
        bound_y;
        resolution = 6;
        sensor_range = 50;
        visited_map;
    end
    methods
        function obj = env(val)
            if nargin == 1
                obj.Total_Number = val;
            end
            A = imread('6.PNG');
            obj.map = A;
            I = rgb2gray(A);
            % figure
            % imshow(I)
            M = 0.6*(double(I)-76)/(225-76);
            % figure
            % imshow(M)
            P_M = binopdf(ceil((M+0.9)*100),250,0.48)*10;
            obj.distribution_map = P_M;
            obj.width = length(M);
            obj.length = length(M(:,1));
            obj.bound_x = obj.width/obj.resolution;
            obj.bound_y = obj.length/obj.resolution;
            obj.visited_map = ones(obj.length,obj.width);
            i=0;
            while(i<obj.Total_Number)
                rr = rand();
                Sample_M = find(P_M>rr);
                if(numel(Sample_M)==0)
                    continue;
                end
                random_num = Sample_M(randi(numel(Sample_M),1,1));
                obj.distribute_x = [obj.distribute_x,floor(random_num/ length(P_M(:,1)))];
                obj.distribute_y = [obj.distribute_y,mod(random_num,length(P_M(:,1)))];
                %plot(x,y,'Pentagram','MarkerSize',10,'MarkerEdgeColor','r');
                %hold on;
                i=i+1;
            end
%             imshow(A)
%             hold on;
%             plot(obj.distribute_x,obj.distribute_y,'Pentagram','MarkerSize',10,'MarkerEdgeColor','r');
        end
        function obj = update(obj,next_position_x,next_position_y)
            [obj.UAV_grid_position_x,obj.UAV_grid_position_y] = xy_to_grid(obj,next_position_x,next_position_y);
            stepx = (next_position_x-obj.UAV_current_position_x)/obj.resolution;
            stepy = (next_position_y-obj.UAV_current_position_y)/obj.resolution;
            tmpy = obj.UAV_current_position_y;
            for tmpx = obj.UAV_current_position_x+stepx:stepx:next_position_x
                tmpy = tmpy + stepy;
                [position_distribute_x,position_distribute_y] = grid_to_xy(obj,obj.distribute_x,obj.distribute_y);
                relate_position = [position_distribute_x-tmpx;position_distribute_y-tmpy];
                distance = sqrt(relate_position(1,:).^2+relate_position(2,:).^2);
                index = find(distance<=6);
                obj.distribute_x(index)=[];
                obj.distribute_y(index)=[];
                [x_l,y_d] = xy_to_grid(obj,tmpx-5,tmpy-5);
                [x_r,y_u] = xy_to_grid(obj,tmpx+5,tmpy+5);
                obj.visited_map(y_u:y_d,x_l:x_r) = 0;
            end
            obj.UAV_current_position_x = next_position_x;
            obj.UAV_current_position_y = next_position_y;
        end
        function plot(obj)
            imshow(obj.map);
            hold on;
            plot(obj.distribute_x,obj.distribute_y,'Pentagram','MarkerSize',10,'MarkerEdgeColor','r');
            hold on;
            scatter(obj.UAV_grid_position_x,obj.UAV_grid_position_y);
            hold on;
            rectangle('Position',[obj.UAV_grid_position_x-obj.sensor_range/2 obj.UAV_grid_position_y-obj.sensor_range/2 obj.sensor_range obj.sensor_range]);
            title(['Find item is ',num2str(obj.Total_Number-length(obj.distribute_x))])
            %rectangle('Position',[-3 -9 20 20]);
        end
        function [grid_x,grid_y]=xy_to_grid(obj,position_x,position_y)
            grid_x = max(min(ceil(position_x * obj.resolution),obj.width),1);
            grid_y = min(max(ceil(obj.length-position_y * obj.resolution),1),obj.length);
        end
        function [position_x,position_y]=grid_to_xy(obj,grid_x,grid_y)
            position_x = grid_x/obj.resolution;
            position_y = (obj.length-grid_y)/obj.resolution;
        end
    end
end