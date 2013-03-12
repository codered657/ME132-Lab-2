function [] = lab2_part1_plot()
% This function takes a data file of occupancy grid values and plots it

% Define constants
data_file = 'data.txt';
map_image = 'lab_obs.png';
x_size = 101;
y_size = 101;
x_start = -2.5;
y_start = -2.5;
grid_size = 0.05;

x_end = x_start + x_size * grid_size;
y_end = y_start + y_size * grid_size;

% Read in data
data = csvread(data_file);
len_data = size(data,2) / x_size
len_data = floor(len_data)

% Generate x,y grid
[X,Y] = meshgrid(x_start:grid_size:x_end, y_start:grid_size:y_end);
X = reshape(X,1,[]);
Y = reshape(Y,1,[]);
size(X)
size(Y)
% Plot room data
figure;
for t = 0:len_data-1
    curr_data = data(1+t*x_size:(t+1)*x_size, :);
    curr_data = reshape(curr_data,1,[]);
    size(curr_data)
    scatter(X, Y, 2, [curr_data * 255, 0, 0]);
    title('Room Occupancy Grid')
end
