%% TODO
% [X] generate occupancy grid from IGVCmap.m
% [X] modify problem parameters to use occ_grid dimensions
% [X] convert inpolygon() functoin (check if point is in occupied tile)
% [ ] convert CheckCollision() function (check edge for collision)
% [ ] change plotting code to use occupancy grid
% [ ] remove unnecessary polygon related code
% [ ] 
%% Probabilistic Road Map example - batch form
clear; clc;

%% Planning map


I = imread('IGVCmap.jpg');
map = im2bw(I, 0.7); % Convert to 0-1 image
map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size

% Robot start position
dxy = 0.1;
startpos = [40 5 pi];

% Target location
searchgoal = [50 10];

% Plotting
figure(1); clf; hold on;
colormap('gray');
imagesc(1-map');
plot(startpos(1)/dxy, startpos(2)/dxy, 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(searchgoal(1)/dxy, searchgoal(2)/dxy, 'gx', 'MarkerSize',10, 'LineWidth', 3 );
axis equal

%% Problem parameters
%tic;

% Set up the map
xMax = [M N]; % State bounds
xMin = [1 1];
xR = xMax-xMin;

% Set up the goals
x0 = startpos(1:2);
xF = searchgoal;
% toc;

%% Multi-query PRM, created in batch
%tic;

% Get milestones
nS = 200;
samples = [xR(1)*rand(nS,1)+xMin(1) xR(2)*rand(nS,1)+xMin(2)];

milestones = zeros(size(samples));
count = 0;
% CHECK if point falls w/n obstacle
for i=1:length(samples)
    x_index = round(samples(i,1));
    y_index = round(samples(i,2));
    occ_grid_val = map(x_index, y_index);
    if(occ_grid_val == 0)
        count = count + 1;
        milestones(count, :) = [x_index y_index];
        % free, so we add it to our list of milestones
    end
end


figure(1); hold on;
plot(samples(:,1),samples(:,2),'k.');
plot(milestones(:,1),milestones(:,2),'m.');
nM = length(milestones(:,1));
disp('Time to generate milestones');
% toc;

% Attempt to add closest p edges
%tic;
p = 20;
e = zeros(nM,nM);
D = zeros*ones(nM,nM);
for i = 1:nM
    % Find closest neighbours
    for j = 1:nM
        d(j) = norm(milestones(i,:)-milestones(j,:));
    end
    [d2,ind] = sort(d);
    % Check for edge collisions (no need to check if entire edge is
    % contained in obstacles as both endpoints are in free space)
    for j=1:p
        cur = ind(j);
        if (i<cur)
            if (~CheckCollision(milestones(i,:),milestones(cur,:), obsEdges))
                e(i,cur) = 1;
                e(cur,i) = 1;
                plot([milestones(i,1) milestones(cur,1)],[milestones(i,2) milestones(cur,2)],'m');
            end
        end
    end
end
disp('Time to connect roadmap');
% toc;

% Find shortest path
%tic;
[sp, sd] = shortestpath_mr(milestones, e, 1, 2, 1, 1, 0);
disp('Time to find shortest path');
% toc;
for i=1:length(sp)-1
    plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',3);
end
