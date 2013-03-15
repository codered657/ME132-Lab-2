function [] = lab2_part2_plot(simulation)
% This function takes a data file of occupancy grid values and plots it

% Define constants
if (simulation == 1)
    % Use simulation data and simulated lab environment
    data_file = 'test_data.txt';
    map_file = 'lab_obs.png';
else
    % Use real data with "empty lab environment
    data_file = 'lab_data.txt';
    map_file = 'lab_empty.png';
end

x_size = 101 - 1;
y_size = 101 - 1;
x_start = -2.5;
y_start = -2.5;
grid_size = 0.05;
% Compute end points
x_end = x_start + x_size * grid_size;
y_end = y_start + y_size * grid_size;
% Define world size
world_start_x = -4.0;
world_start_y = -4.0;
world_end_x = 4.0;
world_end_y = 4.0;

map_image = imread(map_file);
map_image = map_image(:,:,1);
imshow(map_image);
axis on;
axis([x_start x_end y_start y_end])
hold on;

% Generate x,y grid, rescale to match plotting
% Note that x,y convention is swapped for plottng
mesh_start_x = size(map_image, 1) * (x_start - world_start_x) / (world_end_x - world_start_x);
mesh_end_x = size(map_image, 1) * (x_end - world_start_x) / (world_end_x - world_start_x);
mesh_grid_x = (mesh_end_x-mesh_start_x)/x_size;
mesh_start_y = size(map_image, 2) * (y_start - world_start_y) / (world_end_y - world_start_y);
mesh_end_y = size(map_image, 2) * (y_end - world_start_y) / (world_end_y - world_start_y);
mesh_grid_y = (mesh_end_y-mesh_start_y)/y_size;
[X,Y] = meshgrid(mesh_end_y:-mesh_grid_y:mesh_start_y, mesh_start_x:mesh_grid_x:mesh_end_x);
X = reshape(X,1,[]);
Y = reshape(Y,1,[]);
X = round(X);
Y = round(Y);

% Read in data
data = csvread(data_file);
data = data(:,1:size(data,2)-1);
len_data = floor(size(data,1) / (y_size+1));
% Transform back to probabilities
data = exp(data);
data = data ./ (1+data);

% Plot room data
for t = 0:10:len_data-1
    disp(t)
    curr_data = data(1+t*(y_size+1):(t+1)*(y_size+1), :);
    curr_data = reshape(curr_data,1,[]);
    color = curr_data' * [1, 0, 0];
    clf
    imshow(map_image);
    hold on;
    scatter(Y, X, 8, color);
    drawnow
end
