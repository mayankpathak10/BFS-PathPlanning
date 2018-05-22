%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 661 Planning For Autonomous Robotics
% Project 2 - Spring 2018
% Given the Arena Dimentions,create Obstacle Space using Half Plane Method
% , generate a optimal path using Breadth First Search (BFS). The Start and
% Goal Points are given interactively by the User.All the expanded Nodes
% are displayed as white dots.
% 
% Code By: Mayank Pathak
%       
%
% Dependencies: This code uses a function named 'ObstacleSpace_Generator,
% which is saved as 'ObstacleSpace_Generator.m' with this file.
%
% Notes: This code runs instantly, if Live Dispay is off. To turn it ON,
% Change the value of variable 'Display_Live' to 1.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Defining Obstacle Parameters and Initializing Variables
clear all
clc 

size_x = 250;
size_y = 150;
rect_x = [55,55,105,105];
rect_y = [67,112,112,67];
poly_x = [120,158,165,188,168,145,120];
poly_y = [55,51,89,51,14,14,55];
circ_x = 180;
circ_y = 120;
circ_r = 15;
Orectangle = [rect_x;rect_y];
Opolygon = [poly_x;poly_y];
Ocircle = [circ_x;circ_y;circ_r];
Arena = zeros(size_y,size_x);
Visited_x = [];
Visited_y = [];
Display_Live = 0;

%  Calling the function to make Obstacle Space
[Arena] = ObstacleSpace_generator(Orectangle,Opolygon,Ocircle,Arena);
                         

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET THE MAP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
% draw grid environment
xp3 = [0 ,250, 250, 0 ,0];
yp3 = [0 ,0, 150 ,150 ,0];
drawnow
plot(xp3,yp3);
alpha(0.3);

V = [0 0; 0 size_y; size_x size_y; size_x 0];
F = [1 2 3 4];
patch('Faces',F,'Vertices',V,'FaceColor','blue','FaceAlpha',0.4);
hold on;



% Defining Rectangular and Polygonal Obstacle

o1x = 55; o1y = 67; o1xw = 50; o1yw = 45;
rect_pos = [ o1x o1y o1xw o1yw];
rectangle('Position',rect_pos,'FaceColor','b');

OX1 = [120 158 165 188 168 145 120];
OY1 = [55 51 89 51 14 14 55];
patch(OX1,OY1,'blue');


% Defining Circular Obstacle
center_x=180;
center_y = 120;
t = 0:0.01:2*pi;
x = 15*cos(t)+ center_x;
y = 15*sin(t) + center_y;
drawnow
plot(x,y),
fill(x,y,'b');

title(' \fontsize{20} Please Select a Start Point');
% start locations
[ sx,sy] = ginput(1);
sx = round(sx); sy = round(sy);
Start_Node = [sx sy];

%%% Check inputs for boundaries

while Arena(sy,sx) == 1
    title('\fontsize{20}Please Select a Correct Start Point');
    [sx,sy] = ginput(1);
    sx = round(sx); sy = round(sy);
    Start_Node = [sx sy];
end

title('\fontsize{20}Now Please Select a Goal Point');
[gx,gy] = ginput(1);
gx = round(gx); gy = round(gy);
Goal_Node = [gx gy];

%%% Check inputs for boundaries
while Arena(gy,gx) == 1
    title('\fontsize{20}Please Select a Correct Goal Point');
    [gx,gy] = ginput(1);
    gx = round(gx); gy = round(gy);
    Goal_Node = [gx gy];
end

title('\fontsize{20}Computing the Optimal Path');
plot(Start_Node(1),Start_Node(2),'g.','MarkerSize',20);
plot(Goal_Node(1),Goal_Node(2),'r.','MarkerSize',20);
drawnow;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% PERFORM SEARCH
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Nodes = [];
Nodes_Info = [];
% [ Current_Node, Parent_Node,
Nodes_Info(:,:,1) = [1,1,0];
Nodes(:,:,1) = Start_Node;
ParentNode_Index = 1;
CostToCome = 0;
Current_Node = Nodes(:,:,ParentNode_Index);
CurrentNode_Index = 1;
N_Visited = 1;
Dist_Min = 500;


%%%% BFS for Creating Nodes for all points until target is reached 
while ~isequal(Current_Node,Goal_Node)
    
 Current_Node = Nodes(:,:,ParentNode_Index);
 c_x = Current_Node(1);
 c_y = Current_Node(2);
expanded = 0;
for i = -1:1
        for j = -1:1 
              Neighbor_Node = [c_x+i,c_y+j];
%         Enforcing Boundary Conditions and Not current condition.
              if ( ~((i == 0) && (j == 0)) && c_x+i>0 && c_y+j>0 &&...
                      c_x<size_x && c_y<size_y) 
                  
                  if (Arena(Neighbor_Node(1,2),Neighbor_Node(1,1))== 0)
                      N_Visited=N_Visited+1; 
                      Nodes(:,:,N_Visited)=Neighbor_Node;
                      Nodes_Info(:,:,N_Visited) = [N_Visited,ParentNode_Index,CostToCome];
                      Arena(Neighbor_Node(1,2),Neighbor_Node(1,1))=2;
                      Visited_x = [Visited_x,Neighbor_Node(1,1)];
                      Visited_y = [Visited_y,Neighbor_Node(1,2)];
                      Dist_Current = sqrt((gx-Neighbor_Node(1))^2 + (gy-Neighbor_Node(2))^2);
%         Condition for Live Displaying while Expanding
                       if Display_Live == 1
                           drawnow
                          plot(Neighbor_Node(1,1),Neighbor_Node(1,2),'.','Color',[0.9 0.9 0.9]);
                       end
                  end
                  if Dist_Min>Dist_Current
                      Dist_Min = Dist_Current;
                  end
              end
        end
end

ParentNode_Index = ParentNode_Index+1;
end
   
%  Backtracking the nodes to plot the optimal path found
ParentNode_Index = ParentNode_Index - 1;

i=2;
path(:,:,1) = Current_Node;
Index = ParentNode_Index;
TrackInfo =[];
TrackInfo(:,:,1)= [1,Index];
 while  ~isequal(Current_Node,Start_Node)
     CurrentChildren = [];  
     MinDist = 2500;
     P = Nodes_Info(1,2,Index);
    
     for z = 1:size(Nodes_Info,3)
         if Nodes_Info(1,2,z) == P
            CurrentChildren = [CurrentChildren,Nodes_Info(1,1,z)];
         end
     end
     for a = 1:size(CurrentChildren)
         Node = Nodes(:,:,CurrentChildren(a));
         Current_Dist = sqrt((sx-Node(1))^2 + (sy-Node(2))^2);
     end
     [M,I] = min(Current_Dist);
     Best_Child = CurrentChildren(I);
     
     Current_Node = Nodes(:,:,Best_Child);
     path(:,:,i) = Nodes(:,:,Best_Child);
     Nodes(:,:,Index);
     TrackInfo(:,:,i)= [Index,Best_Child];
     Index = Nodes_Info(1,2,Best_Child);
     i = i+1;
     

 end
 %%% plot the optimal path
 for u = 1:i-1
 Pathx(u) = path(1,1,u);
 Pathy(u) = path(1,2,u);
 end
 
 title('\fontsize{20}Result');
 if Display_Live == 0
      drawnow
      plot(Visited_x,Visited_y,'.','Color',[0.9 0.6 0.6]);
 end
 drawnow
 plot(Pathx,Pathy,'linewidth',2);
 hold off


