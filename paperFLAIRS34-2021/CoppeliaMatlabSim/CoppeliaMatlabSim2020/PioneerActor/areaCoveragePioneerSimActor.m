% Sample script for simulating the full area coverage algorithm
% using three Pioneer robots
% Amr Elhussein 29/3/2020

clear
clc
close all

disp('Please wait while the simulation runs ...');

%==========================================================================
% SIMULATION PARAMETERS
%==========================================================================
% Simulation time
t0 = 0; tf = 4;    % Initial and final simulation time [s]
Ts = 0.01;          % Sampling time [s]
Ti = floor((tf-t0)/Ts); % number of time steps
dt = Ts*(0:Ti)';    % Discrete time vector (include time t0 to plot initial state too) 
%==========================================================================
Kg = 0.65;    % formerly 1.0 proportional gain on Gamma
vk = 0.20; % [m/s] Velocity of agents - NOTE: the speed is much higher than it 
%                                          will be in implementation for 
%                                          simulation purposes
%==========================================================================
l = 0.381; % [m]
rad = 0.0925; % [m] radius
%==========================================================================
k = 1; %main timing index
%data collection index
kappa1 = 1;
kappa2 = 1;
kappa3 = 1;
%==========================================================================
% ROS SETUP
%==========================================================================
%centroids position publishers
[vpub_4,msg_4] = rospublisher('/cent1_pose','geometry_msgs/Vector3'); %Publishes on topic '/desiredVel_1'
[vpub_5,msg_5] = rospublisher('/cent2_pose','geometry_msgs/Vector3'); %Publishes on topic '/desiredVel_1'
[vpub_6,msg_6] = rospublisher('/cent3_pose','geometry_msgs/Vector3'); %Publishes on topic '/desiredVel_1'

posesub_1 = rossubscriber('/pioneerPose_1'); %Subscribes to topic '/pioneerPose_1'
[vpub_1,msg_1] = rospublisher('/desiredVel_1','geometry_msgs/Vector3'); %Publishes on topic '/desiredVel_1'

posesub_2 = rossubscriber('/pioneerPose_2');
[vpub_2,msg_2] = rospublisher('/desiredVel_2','geometry_msgs/Vector3');

posesub_3 = rossubscriber('/pioneerPose_3');
[vpub_3,msg_3] = rospublisher('/desiredVel_3','geometry_msgs/Vector3');
%==========================================================================


%==========================================================================
% INITIALIZATION
%==========================================simTime = rossubscriber('/simulationTime')================================
area = [0,5,5,0;
        0, 0, 5,5];  % [m]; vertices of area 
agents = [1,   4, 3.25;
          2.5, 3, 0.75]; % [m] agents position in cm
      
gridSize = 0.05; % [m]; grid size of the area 
 
q = 1; % Variables used to index in for loops
r = 1;
s = 1;

yy = min(area(2,:)):gridSize:max(area(2,:));
xx = min(area(1,:)):gridSize:max(area(1,:));

sigma = 0.50;       % Formerly 0.35 - Density factor 
n = 3;              % Number of agents
alpha = 5;          % Scaling variable used in sensor function
beta = 0.5;         % Scaling variable used in sensor function

qBar = [2.5, 4,    1.5; 
        4,   0.75, 1];  

positions(1,:) = [agents(1,1), agents(2,1)];
positions(2,:) = [agents(1,2), agents(2,2)];
positions(3,:) = [agents(1,3), agents(2,3)];

gridArray = zeros(1,2);
for i=1:length(xx)
    gridArray = [gridArray;repmat(xx(i),length(yy),1),yy'];     
end

gridArray = gridArray(2:end,:);

phi = getDensity(gridArray,qBar,sigma); % Phi function to calculate density

gridArrayDensity = [gridArray phi];

%figure (1) %Begin plotting x-y position for the three agents

% vid = VideoWriter('OUT/areaCoverageMatlabXY.avi');
% vid.Quality = 100;
% vid.FrameRate = 5;
% open(vid)
% 
% fig=figure (1);
% clf reset;   
%=============================================================
%initilize weight matrices
 P1 =  [5    0   0   2   0.5;
        0   3   0   0.5   0.5;
        0   0   3   0.5    2;
        2   1  5   0.9      0;
        2 1  5    0      0.9];
     P1 = 20*P1+50*rand;
%  P1 =[38.2523   17.6192   16.0915   12.6800    6.0706;
%     17.6192   69.7282   24.6281   21.2167   14.6073;
%     16.0915   24.6281   75.8153   19.6889   13.0795;
%     12.6800   21.2167   19.6889   33.9500    9.6681;
%      6.0706   14.6073   13.0795    9.6681   43.4588];
%P1 = P1 + 0.5*rand(5,5);  
P2 =  [3    0   0   2   0.5;
        0   3   0   0.5   0.5;
        0   0   3   0.5    2;
        2   2  1   3      0;
        1 1  2    0      3];
     P2 = 20*P2+50*rand;
%P2 = P1; 
P3 =  [3    0   0   2   0.5;
        0   3   0   0.5   0.5;
        0   0   3   0.5    2;
        2   1  1   0.7      0;
        2   1 1    0      0.7];
 P3 = 20*P3+50*rand;
%P2 = P2 + rand(5,5);
%P3 = P1;
%initlize actor weights
Wa1=1.2*rand(2,3) ;
Wa2=0.5*rand(2,3) ;
Wa3=0.2*rand(2,3) ;
%initilize z1 with a value that is going to be replaced with a the true value 
z1(1,:) = [0 0 0 0 0] ;
z2(1,:) = [0 0 0 0 0] ;
z3(1,:) = [0 0 0 0 0] ;

for t = 0:Ts:tf % Main timing loop 
    t
%    k
%    kappa1
%P1
    agents = [positions(1,1), positions(2,1), positions(3,1);
              positions(1,2), positions(2,2), positions(3,2)];
    
    %Receiving robot_1's current pose
    posedata_1 = receive(posesub_1,10);
    positions(1,1) = posedata_1.X;
    positions(1,2) = posedata_1.Y;
    orientations(1,1) = posedata_1.Z;
    %saving positions
    q1(k,:) = [positions(1,:) , orientations(1,1) ];
    
    %Receiving robot_2's current pose
    posedata_2 = receive(posesub_2,10);
    positions(2,1) = posedata_2.X;
    positions(2,2) = posedata_2.Y;
    orientations(1,2) = posedata_2.Z;
    %saving positions
    q2(k,:) = [positions(2,:) , orientations(1,2) ];
    
    %Receiving robot_3's current pose
    posedata_3 = receive(posesub_3,10);
    positions(3,1) = posedata_3.X;
    positions(3,2) = posedata_3.Y;
    orientations(1,3) = posedata_3.Z;
     %saving positions
     q3(k,:) = [positions(3,:) , orientations(1,3) ] ;
    %======================================================================
    %======================================================================
    for i=1:size(gridArray,1)
        dist2Agent1(i) = sqrt(((gridArray(i,1) - agents(1,1))^2) + ((gridArray(i,2) - agents(2,1))^2));
        dist2Agent2(i) = sqrt(((gridArray(i,1) - agents(1,2))^2) + ((gridArray(i,2) - agents(2,2))^2));
        dist2Agent3(i) = sqrt(((gridArray(i,1) - agents(1,3))^2) + ((gridArray(i,2) - agents(2,3))^2)); 
        
        %For V1
        if ((dist2Agent1(i) < dist2Agent2(i)) && (dist2Agent1(i) < dist2Agent3(i))) 
            index(i) = 1; 
            tempPointsV1(i,:) = gridArray(i,:);
        end
        %For V2
        if ((dist2Agent2(i) < dist2Agent1(i)) && (dist2Agent2(i) < dist2Agent3(i))) 
            index(i) = 2;
            tempPointsV2(i,:) = gridArray(i,:);
        end     
        %For V3
        if ((dist2Agent3(i) < dist2Agent1(i)) && (dist2Agent3(i) < dist2Agent2(i))) 
            index(i) = 3;
            tempPointsV3(i,:) = gridArray(i,:);
        end     
                      
    end
    %======================================================================
    %======================================================================
    for i = 1:size(tempPointsV1)
        %==================================================================
        if (index(i) == 1)
           r_i_1(q) = sqrt(((gridArrayDensity(i,1) - agents(1,1))^2) + ((gridArrayDensity(i,2) - agents(2,1))^2));
           f_r1(q) = alpha * exp(-beta*(r_i_1(q)^2));
           pointDensityV1(q,:) = gridArrayDensity(i,:);
           q = q+1;
        end
    end 
    for i = 1:size(tempPointsV2)
        %==================================================================
        if (index(i) == 2)
           r_i_2(r) = sqrt(((gridArrayDensity(i,1) - agents(1,2))^2) + ((gridArrayDensity(i,2) - agents(2,2))^2));
           f_r2(r) = alpha * exp(-beta*(r_i_2(r)^2));
           pointDensityV2(r,:) = gridArrayDensity(i,:);
           r = r+1;
        end
    end
    for i = 1:size(tempPointsV3)
        %==================================================================
        if (index(i) == 3)
           r_i_3(s) = sqrt(((gridArrayDensity(i,1) - agents(1,3))^2) + ((gridArrayDensity(i,2) - agents(2,3))^2));
           f_r3(s) = alpha * exp(-beta*(r_i_3(s)^2));
           pointDensityV3(s,:) = gridArrayDensity(i,:);
           s = s+1;
        end
    end 
    q = 1; % Resetting looping variables 
    r = 1;
    s = 1;
    %======================================================================
    %======================================================================
    phiV1 = pointDensityV1(:,3);
    phiV2 = pointDensityV2(:,3);
    phiV3 = pointDensityV3(:,3);
    %======================================================================
    Mv_1 = sum(phiV1);
    Mv_2 = sum(phiV2);
    Mv_3 = sum(phiV3);
    %For V1
    xCv_1 = sum(pointDensityV1(:,1) .* phiV1)/(Mv_1);
    yCv_1 = sum(pointDensityV1(:,2) .* phiV1)/(Mv_1);
    %For V2
    xCv_2 = sum(pointDensityV2(:,1) .* phiV2)/(Mv_2);
    yCv_2 = sum(pointDensityV2(:,2) .* phiV2)/(Mv_2);
    %For V3
    xCv_3 = sum(pointDensityV3(:,1) .* phiV3)/(Mv_3);
    yCv_3 = sum(pointDensityV3(:,2) .* phiV3)/(Mv_3);

    cent1 = [xCv_1, yCv_1];
    cent2 = [xCv_2, yCv_2];
    cent3 = [xCv_3, yCv_3];
    
    %saving the centroid for every time step
    QD1(k,:) = [cent1 0];
    QD2(k,:) = [cent2 0];
    QD3(k,:) = [cent3 0];
    
    %saving the thetaprime for each robot
    thetaPrime1(k,:) = atan2(QD1(k,2)-q1(k,2),QD1(k,1)-q1(k,1));
    thetaPrime2(k,:) = atan2(QD2(k,2)-q2(k,2),QD2(k,1)-q2(k,1));
    thetaPrime3(k,:) = atan2(QD3(k,2)-q3(k,2),QD3(k,1)-q3(k,1));
    %saving the tracking error for each robot
    trackingError1(k,:) = [QD1(k,1)-q1(k,1), QD1(k,2)-q1(k,2), pi2pi(thetaPrime1(k,:)-q1(k,3))];
    trackingError2(k,:) = [QD2(k,1)-q2(k,1), QD2(k,2)-q2(k,2), pi2pi(thetaPrime2(k,:)-q2(k,3))];
    trackingError3(k,:) = [QD3(k,1)-q3(k,1), QD3(k,2)-q3(k,2), pi2pi(thetaPrime3(k,:)-q3(k,3))];

    %======================================================================
    %======================================================================  
        
    H1 = sum((f_r1) * phiV1);
    H2 = sum((f_r2) * phiV2);
    H3 = sum((f_r3) * phiV3);
    Hk = H1+H2+H3; % Total coverage at time instant 't'

    %======================================================================
    % FINDING THE NEW DESIRED POSITIONS AND ORIENTATIONS
    %====================================================================== 
    %determining policy for Robot1
    [Vk1,gammak1,P1,kappa1,Wa1] = modelfree(q1,Ts,QD1,k,trackingError1,Ti,z1,P1,kappa1,Wa1);
    P1 = P1; %saving the p form last iterations 
    Wa1=Wa1;
    Vk1= Vk1;
    %save the z vector
    z1(kappa1,:) = [trackingError1(k,:) Vk1 gammak1];
    %determining policy for Robot2
    [Vk2,gammak2,P2,kappa2,Wa2] = modelfree(q2,Ts,QD2,k,trackingError2,Ti,z2,P2,kappa2,Wa2);
    P2 = P2;
    Wa2=Wa2;
    Vk2=  Vk2;
    z2(kappa2,:) = [trackingError2(k,:) Vk2 gammak2];
    %determining policy for Robot3
    [Vk3,gammak3,P3,kappa3,Wa3] = modelfree(q3,Ts,QD3,k,trackingError3,Ti,z3,P3,kappa3,Wa3);
    Vk3=  -Vk3;
    P3 = P3;
    Wa3=Wa3
    z3(kappa3,:) = [trackingError3(k,:) Vk3 gammak3];
    
    angleToCentroid_1 = moveAgent(positions(1,:), cent1);
    angleToCentroid_2 = moveAgent(positions(2,:), cent2);
    angleToCentroid_3 = moveAgent(positions(3,:), cent3);
    
    dTheta_1 = angleToCentroid_1 - orientations(1,1); 
    dTheta_2 = angleToCentroid_2 - orientations(1,2);
    dTheta_3 = angleToCentroid_3 - orientations(1,3);
    
    
    %======================================================================
    % LIMITING dTheta BETWEEN PI AND -PI %
    %======================================================================
    if (dTheta_1 > pi)
        dTheta_1 = (dTheta_1 - 2*pi);
    end
    if (dTheta_1 < -pi)
        dTheta_1 = (dTheta_1 + 2*pi);
    end
    %======================================================================
    if (dTheta_2 > pi)
        dTheta_2 = (dTheta_2 - 2*pi);
    end
    if (dTheta_2 < -pi)
        dTheta_2 = (dTheta_2 + 2*pi);
    end
    %======================================================================
    if (dTheta_3 > pi)
        dTheta_3 = (dTheta_3 - 2*pi);
    end
    if (dTheta_3 < -pi)
        dTheta_3 = (dTheta_3 + 2*pi);
    end

    %======================================================================
    
    %======================================================================
    %======================================================================
    
    wk_1 = gammak1;%Kg*dTheta_1; %update robot_1's angular speed
    nuR_1 = (2*Vk1+l*wk_1)/2; % [m/s] robot_1's right wheel's linear speed
    nuL_1 = (2*Vk1-l*wk_1)/2; % [m/s] robot_1's left wheel's linear speed 
   
    wk_2 = gammak2;%Kg*dTheta_2; %update robot_1's angular speed
    nuR_2 = (2*Vk2+l*wk_2)/2; % [m/s] robot_1's right wheel's linear speed
    nuL_2 = (2*Vk2-l*wk_2)/2; % [m/s] robot_1's left wheel's linear speed 
    
    wk_3 = gammak3;%Kg*dTheta_3; %update robot_1's angular speed
    nuR_3 = (2*Vk3+l*wk_3)/2; % [m/s] robot_1's right wheel's linear speed
    nuL_3 = (2*Vk3-l*wk_3)/2; % [m/s] robot_1's left wheel's linear speed 

    %====================================================================== 
    %PUBLISH ROBOT'S DESIRED VELOCITY
    %======================================================================
    %centroid1
    msg_4.X = xCv_1;
    msg_4.Y = yCv_1;
    msg_4.Z = 0;
    
    %centroid2
    msg_5.X = xCv_2;
    msg_5.Y = yCv_2;
    msg_5.Z = 0;
    
    %centroid3
    msg_6.X = xCv_3;
    msg_6.Y = yCv_3;
    msg_6.Z = 0;
    
    %send centroid position
    send(vpub_4,msg_4);
    send(vpub_5,msg_5);
    send(vpub_6,msg_6);
    
    %Robot 1
    msg_1.X = nuR_1;
    msg_1.Y = nuL_1;
    msg_1.Z = 0;
    
    %Robot 2
    msg_2.X = nuR_2;
    msg_2.Y = nuL_2;
    msg_2.Z = 0;
    
    %Robot 3
    msg_3.X =nuR_3;
    msg_3.Y = nuL_3;
    msg_3.Z = 0;
    
    send(vpub_1,msg_1);
    send(vpub_2,msg_2);
    send(vpub_3,msg_3);
    
    kappa1 = kappa1+1; %data collection index
    kappa2 = kappa2+1;
    kappa3 = kappa3+1;
    k = k+1;
    %======================================================================
    %====================================================================== 
    
    %figure (1)
    %directions = zeros(1,n);
    %set(gca,'Color',[0.8,0.8,0.8]); % figure's background color
    %xlabel('X [m]','Interpreter','latex','fontsize',14);
    %ylabel('Y [m]','Interpreter','latex','fontsize',14);
    %colormap(flipud(gray))
    %caxis([0 1]);
    %colorbar;

    %======================================================================
    %====================================================================== 

    %Plotting density points in real time
%     hold on
%     scatter(gridArrayDensity(:,1), gridArrayDensity(:,2),10,gridArrayDensity(:,3),'filled');

    %Plotting the modified centroids in time
%     plot(xCv_1, yCv_1,'bo')
%     plot(xCv_2, yCv_2,'bo')
%     plot(xCv_3, yCv_3,'bo')

    %Plotting the Voronoi regions in time
%     regions = voronoi_andrew(area,agents);
%     drawRegions(area,regions,orientations); %directions
%    
%     hold on;
%     F = getframe(fig);
%     writeVideo(vid,F);    
    
    pause(Ts); % Outside range of real time operation already
    
end

% savefilename = 'OUT/areaCoverageMatlabXY_Final';
% saveas(gcf, savefilename, 'fig');
% print('-depsc2', '-r300', [savefilename, '.eps']);
% 
% close(vid);

%====================================================================== 
%STOP ROBOTS
%======================================================================
%Robot 1
msg_1.X = 0;
msg_1.Y = 0;
msg_1.Z = 0;

%Robot 2
msg_2.X = 0;
msg_2.Y = 0;
msg_2.Z = 0;

%Robot 3
msg_3.X = 0;
msg_3.Y = 0;
msg_3.Z = 0;

send(vpub_1,msg_1);
send(vpub_2,msg_2);
send(vpub_3,msg_3);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%       FUNCTIONS USED IN CODE          %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function regionData = voronoi_andrew(bounds, points)
    % VORONOI   ordinary Voronoi partition generator
    %   VORONOI will take in a bounding region along with a set of generator
    %   points and produce the ordinary Voronoi partition defined by
    %       |q - p_i| <= |q - p_j|
    %   for all j ~= i.
    %
    %   ***** Inputs *****
    %   bounds - a 2 x n_b array specifying the positively oriented bounding
    %       polygon in 2D
    %   points - the 2 x n_p array specifying generator points.
    %
    %   ***** Outputs *****
    %   regionData - a n x 3 cell with:
    %       -- the {i,1} element containing generator point information, 
    %       -- the {i,2} element containing the arc information. For the
    %          ordinary Voronoi partition, this is always empty.
    %       -- the {i, 3} element containing edge segments. These are 
    %          oriented, and each row represents a segment:
    %          [x1, y1, x2, y2]
    %          And the region is always to the left of the vector from (x1, y1)
    %          to (x2, y2).

    n = size(points, 2);
    regionData = cell(n, 3);
   
    for i = 1:n
        p1 = points(:,i);
        regionData{i, 1} = p1;

        vi = bounds;
        for j = 1:n
            if (j ~= i)
                p2 = points(:, j);
                [s1, s2] = bisectPlane(p1, p2);
                vi = intersect_andrew(vi, s1(1:2), s2(1:2));
            end
        end

        % Create the edges from the polygon data
        edges = zeros(size(vi, 2), 4);
        vi = [vi, vi(:, 1)]';
        for j = 1:size(edges, 1)
            edges(j, :) = [vi(j, :), vi(j+1, :)];
        end
        regionData{i, 2} = [];
        regionData{i, 3} = edges;
    end
end
% Helper function to determine the bisection plane
function [segStart, segEnd] = bisectPlane(p1, p2)
    d = p2 - p1;
    c = [-d(2); d(1)];
    segStart = 0.5 * d + p1;
    segEnd = segStart + c;
end
function vertices = intersect_andrew(polygon, b0, b1)

    % INTERSECT    Geometric intersection of a polygon and halfplane
    %   ***** Inputs *****
    %   polygon - a 2 x n array defining the vertices of a polygon in 2D
    %   b0 - a point on the dividing line
    %   b1 - a second point on the dividing line.  The vector b1-b0 defines the
    %       intersecting halfplane.  The intersection of the ray with the
    %       polygon would define a new positvely oriented edge.  If the polygon
    %       is to the left of the ray, the intersection is the entire polygon.
    %       If the polygon is to the right of the ray, the intersection is the
    %       null set.
    %
    %   ***** Outputs *****
    %   vertices - the vertices of the intersection

    b0 = b0(:);
    b1 = b1(:);
    n = size(polygon, 2);
    if (n == 0)
        vertices = [];
        return;
    end

    % Determine which segments the halfplane intersects
    intersections = zeros(n, 1);
    newPoints = zeros(2, 1, n);
    for i = 1:n
        a0 = polygon(:, i);
        if (i == n)
            a1 = polygon(:, 1);
        else
            a1 = polygon(:, i+1);
        end
        [bool, pt] = isIntersect(a0, a1, b0, b1);
        if (bool)
            intersections(i) = 1;
            newPoints(:,:,i) = pt;
        end
    end

    % Now figure out which half of the polygon to cut.
    if (max(intersections) == 1)    % if the halfplane cuts thru the polygon
        intersectIndex = [0, 0]';
        ctr = 1;
        for i = 1:n
            if (intersections(i) == 1)
                intersectIndex(ctr) = i;
                ctr = ctr + 1;
            end
        end
        % Check positive orientation of the intersected points
        p1 = newPoints(:,:,intersectIndex(1));
        p2 = newPoints(:,:,intersectIndex(2));
        vec = p2 - p1;
        if ((vec' * (b1 - b0)) < 0) % in the wrong direction
            p1 = p2;
            p2 = newPoints(:,:,intersectIndex(1));
            temp = intersectIndex(1);
            intersectIndex(1) = intersectIndex(2);
            intersectIndex(2) = temp;
        end
        % Begin definition of new polygon starting with the cutting segment
        vertices = [p1, p2];
        polygon = circshift(polygon, [0, -intersectIndex(2)]);
        if (intersectIndex(2) > intersectIndex(1))
            cutSize = intersectIndex(2) - intersectIndex(1);
        else
            cutSize = intersectIndex(2) - intersectIndex(1) + n;
        end
        vertices(:, end+1:end+n-cutSize) = polygon(:, 1:n-cutSize);
    else
        % no intersection with polygon, but depending on orientation, the 
        % halfplane will either include or exclude the polygon.  Check using
        % cross-product.
        v1 = b1 - b0;
        for i = 1:n
            p = polygon(:,i) - b0;
            % c = cross(v1, p);
            c = v1(1)*p(2) - v1(2)*p(1);
            if (c < 0)
                vertices = [];
                return
            end
        end
        vertices = polygon;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% helper function to detect intersection of a segment u0->u1 with a line
% v0->v1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [bool, pt] = isIntersect(u0, u1, v0, v1)
    a1 = u1(1) - u0(1);
    a2 = u1(2) - u0(2);
    b1 = v1(1) - v0(1);
    b2 = v1(2) - v0(2);
    c1 = v0(1) - u0(1);
    c2 = v0(2) - u0(2);
    D = a2*b1 - a1*b2;
    if (D == 0)     % parallel lines
        bool = 0;
        pt = NaN;
        return;
    end
    t = -b2*c1/D + b1*c2/D;
    % s = -a2*c1/D + a1*c2/D
    % deal with numerical precision
    if (t < -eps(10) || t > 1 + eps(10))
        bool = 0;
        pt = 0;
    elseif (0<=t && t<=1)
        bool = 1;
        pt = u0 + (u1 - u0)*t;
    elseif (abs(t) <= eps(10))
        bool = 1;
        pt = u0;
    elseif (abs(1-t) <= eps(10))
        bool = 1;
        pt = u1;
    end
end 
function drawRegions(bounds, regionData, orientations)
    % DRAWREGIONS   Voronoi graphical realization
    %   This function will take in a set of generator points and regions and
    %   will draw the associated graph
    %   
    %   ***** Inputs *****
    %   bounds - the bounding positively oriented polygon
    %   regionData - the regions are a nx3 cell array with the {i,1} element 
    %       containing generator point information, the {i,2} element 
    %       containing the arc information, and the {i,3}
    %       containing edge segments (from intersections with the boundary)
    %
    %   Author: Andrew Kwok, University of California at San Diego
    %   Date: 17 August 2009

    n = size(regionData, 1);

    %hold on     %COMMENTED OUT 6/5/2018
    % Define agent label position
    delta = max(max(bounds(1,:)) - min(bounds(1,:)), ...
        max(bounds(2,:)) - min(bounds(2,:))) / 60;
    for i = 1:n
        pt = regionData{i, 1};
        arcs = regionData{i, 2};
        edges = regionData{i, 3};

        % Draw arcs and edges
        for j = 1:size(arcs, 1)
            drawArc(arcs(j, 1:2), arcs(j, 3), arcs(j, 4), arcs(j, 5));
        end
        for j = 1:size(edges, 1)
            line(edges(j, [1, 3]), edges(j, [2, 4]));
        end    
        % Plot the agent position
        [Xa,Ya] = plotUnicycle([pt;orientations(i)],axis(gca)); % DDMR => Differential drive mobile robot
        fill(Xa,Ya,'r');
        % lblintcpt = plot(pt(1), pt(2), 'r.', 'MarkerSize', 16);
        % Label the agent
        label = strcat(num2str(i));
        text(pt(1)+delta, pt(2)+delta, label);
    end

    % Plot the bounding polygon
    bounds(:, end+1) = bounds(:,1);
    lblbounds = plot(bounds(1,:), bounds(2,:), 'r', 'LineWidth', 2);

    % axis equal
    axis square
    xlim([min(bounds(1,:)-0.5),max(bounds(1,:)+0.5)]);
    ylim([min(bounds(2,:)-0.5),max(bounds(2,:)+0.5)]);
    xlabel('X [km]');
    ylabel('Y [km]');


    % legend([lblintcpt lblbounds], 'Interceptor', 'harbour boundary');
    grid on
    %axis off
    %hold off
end
function [X,Y] = plotUnicycle(Q,AX)
    % PLOT_UNICYCLE   Function that generates lines for plotting a unicycle.
    %
    %    PLOT_UNICYCLE(Q,AX) takes in the axis AX = axis(gca) and the unicycle
    %    configuration Q and outputs lines (X,Y) for use, for example, as:
    %       fill(X,Y,'b')
    %    This must be done in the parent script file.

    x     = Q(1);
    y     = Q(2);
    theta = Q(3);

    l1 = 0.02*max([AX(2)-AX(1),AX(4)-AX(3)]);
    X = [x,x+l1*cos(theta-2*pi/3),x+l1*cos(theta),x+l1*cos(theta+2*pi/3),x];
    Y = [y,y+l1*sin(theta-2*pi/3),y+l1*sin(theta),y+l1*sin(theta+2*pi/3),y];
end
function angleToCentroid = moveAgent(position, centroid)
    %This is a function that calculates the next position of an agent...
    %given the current pose of the agent and the corresponding centroid...
    %for the voronoi region.
    
    %centroid = centroid that the agent will move to [x y] [m]
    %Ts = time step of the simulation [s]
    %direction = current azimuth of the agent [rad] -pi:pi
    %position = current position of the agent [x y] [m]
    angleToCentroid = atan2((centroid(2) - position(2)), (centroid(1) - position(1)));
    
end

function phi = getDensity(gridArray,qBar,sigma)

    phi = zeros(size(gridArray,1),1);
    dist = zeros(size(gridArray,1),2,size(qBar,2));
    tmp = zeros(size(gridArray,1),1,size(qBar,2));
        for i = 1:size(qBar,2)
        
        dist(:,:,i) = gridArray-qBar(:,i)';
        end
        
        for j = 1:size(qBar,2)
            for i = 1:size(gridArray,1)
                tmp(i,1,j) = exp((-0.5 * (norm(dist(i,:,j))/sigma)^2));
            end
        end
        phi = sum(tmp,3);
    
end

function [Vk,gammak, Pcalculated,kappa,Wa] = modelfree(q,Ts,QD,k,trackingError,Ti,z,P,kappa,Wa)
%This a function that uses the model free reiforcment learning algorithm to find the optimum
%policy (linear veocity and steering angle) according to the target
%position (centroid)
%initilize the weight matrices
kappa = kappa;
%  Pinit =[38.2523   17.6192   16.0915   12.6800    6.0706;
%     17.6192   69.7282   24.6281   21.2167   14.6073;
%     16.0915   24.6281   75.8153   19.6889   13.0795;
%     12.6800   21.2167   19.6889   33.9500    9.6681;
%      6.0706   14.6073   13.0795    9.6681   43.4588];
Pinit =  [5    0   0   2   0.5;
     0   3   0   0.5   0.5;
     0   0   3   0.5    2;
     2   2  0.5   3      0;
    0.5 0.5  2    0      3];
eig(P); %tracking the eigen values of the matrix
% Q = 0.0000001*[10 0 0;
%                   0 10 0;
%                   0 0   5];
% R = 0.000000000000001*[1 0;
%                     0 1];
% Pbar = blkdiag(Q,R);
%choose your learning rate
lc = 0.000001;
la = 0.05; %actor learning rate
%reconstructing the P matrix
       w(1,1) = 0.5 * P(1,1);
       w(2:5,1)=P(1,2:5);
       w(6,1) = 0.5*P(2,2);
       w(7:9,1) =P(2,3:5) ;
       w(10,1) = 0.5*P(3,3);
       w(11:12,1) =P(3,4:5) ;
       w(13,1) = 0.5*P(4,4);
       w(14,1) = P(4,5); 
       w(15,1)=0.5*P(5,5);
       PWc = w;
% Store weight vector over time (every (n+m)*(n+m+1)/2 time steps).        
%W = zeros((N+M)*(N+M+1)/2,floor(Ti/((N+M)*(N+M+1)/2))+1);

W(:,1) = w;

u(k,:) = Wa*trackingError(k,:)';%finding the policy';%finding the policy;
%setting the limit for the policy
u(k,1) = sign(u(k,1))*min(0.5,abs(u(k,1)));%limiting the linear speed 
u(k,2) = sign(u(k,2))*min(80*(pi/180),abs(u(k,2)));%limiting the steering angle
Vk= u(k,1);
gammak = u(k,2);

 
    if kappa == 17;
        [rho,rhoTelda,valueFnDesired] = NNCalculations(z);
        PWc = w;
        kappa = 1; %restarting data collection
        Pprevious = P;
        
        w = PWc - lc*rhoTelda'*(rhoTelda*PWc-valueFnDesired); % critic wheight matrix
        %W(:,indexWeight+1) = w;
        %indexWeight = indexWeight + 1;
        %current wc
        CWc = w; 
       
       %checking convergence
       err = norm (CWc - PWc);
       if err < 0.0001
            disp('Congratz converging whgihts')
        end
        
    
       
        % Reconstruct  the P matrix
        P(1,1) = 2*w(1,1);
        P(1,2:5) = w(2:5); 
        P(2,2) = 2*w(6,1);
        P(2,3:5) = w(7:9);
        P(3,3) = 2 * w(10,1);
        P(3,4:5) = w(11:12); 
        P(4,4) = 2 * w (13,1);
        P(4,5) = w(14,1); 
        P(5,5) = 2*w(15);
        
        for m = 1:5
            for n = 1:5
                P(n,m) = P(m,n);  
            end
        end
        
        %evaluate the new policy
%         if z(k,:)*Pprevious*z(k,:)' > z(k,:)*P*z(k,:)'
%             disp('found better policy')
%         end
        Pcalculated = P;
        %update policy
       % u(k+1,:) =  -1*inv(P(4:5,4:5))*P(4:5,1:3)*trackingError(k+1,:)';
        uk =  1*inv(P(4:5,4:5))*P(4:5,1:3)*trackingError(k,:)';
        Vk= uk(1,1);
        gammak = uk(2,1);
        %limit control inputs 
%         uk(1) = sign(uk(1))*min(vMax,abs(uk(1)));%limiting the linear speed 
%         uk(2) = sign(uk(2))*min(80*(pi/180),abs(uk(2)));%limiting the steering angle        
%         
        %updating actor weights
        Waa=((Wa*trackingError(k,:)')-(uk+randn(2,1)));
        Wa=Wa-la*Waa*trackingError(k,:);        
        %tracking the eigen values
        eig(P)
        %update the rho 
        c = 0;
%        for i =1:5
%            for j=i:5
%             c=c+1;
%             rho(k+1,c) = z(k+1,i)*z(k+1,j);  % neuron activation functions 
%            end
%        end  
    else
%         k=k+1;
%         kappa = kappa + 1; % Data collection index
    end
    Pcalculated = P;
end
function [rho,rhoTelda,valueFnDesired] = NNCalculations(z)
 %This function records and calculates all the required matrices for the
 %reinforcment learning algorithm utilizing graduent descent
 
 %intilize the Pbar matrix
    Q = 0.0001*[10 0 0;
                      0 10 0;
                      0 0   5];
    R = 0.0001*[1 0;
                        0 1];
    Pbar = blkdiag(Q,R);
    %calulate the pawise product
    rho = paiproduct(z);
    
    for m = 1:15
        rhoTelda(m,:) = rho(m,:)-rho(m+1,:); %need for lambda matrix
        valueFnDesired(m,1) = z(m,:)*Pbar*z(m,:)';%the v vector
    end
end

function [C] = paiproduct(z)
   %this function is used to calculate the paiwaise product of a vector
    i=1;
    for i = 1:16
        D = z(i,:).*z(i,:)';
        D = triu(D);
        A = D.';
        B = tril(true(size(A)));
        D = A(B).';
        C(i,:) = D;
    end
end 

function angle = pi2pi(angle)

%function angle = pi_to_pi(angle)
% Input: array of angles.
% Tim Bailey 2000

angle = mod(angle, 2*pi);

i=find(angle>pi);
angle(i)=angle(i)-2*pi;

i=find(angle<-pi);
angle(i)=angle(i)+2*pi;
end

