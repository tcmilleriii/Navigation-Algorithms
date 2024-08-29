clear all
close all
clc

% This file can be used to generate a set of input data and then save it
% off in the appropriate .txt format for use by my Iterative Closest Point
% (ICP) C++ files.

% NOTE: The data here is only being rotated and moved. I'm not adding in
% any biases that might offset the shape itself of the data.

n = 1;
while n < 3
    n = randi(20);
end

truth_data = [];
for i = 1:n
    truth_data = [truth_data; randi(100) randi(100) randi(100)];
end

% make a random offset distance.
offset_distances = [randi(10); randi(10); randi(10)];

% make a random 3D rotation. (at most 10deg).
a = 0;
b = deg2rad(10);
theta = a + (b-a)*rand(3,1);

% Construct x, y, z rotation matrices.
C_x = rotate_x(theta(1));
C_y = rotate_y(theta(2));
C_z = rotate_z(theta(3));
C = C_x * C_y * C_z;

% Assemble a final transform
transf_truth_to_meas = [C offset_distances; 0 0 0 1];
transf_meas_to_truth = inv(transf_truth_to_meas);

% Apply this transform to the truth data to produce the "measurement" data.
measurement_data = [];
for i = 1:n
    curr_vec = transf_truth_to_meas * [truth_data(i,:)'; 1];
    measurement_data = [measurement_data; curr_vec(1:3,:)'];
end

% Do a final check using Absolute Orientation Algorithm to verify that the
% calculated transform, transf, is viable.
[regParams,Bfit,ErrorStats]=absor(truth_data',measurement_data');
for i = 1:numel(transf_truth_to_meas)
    if (regParams.M(i) - transf_truth_to_meas(i)) > 0.0000000001
        error('Mismatch between calculated transform and Abs Orientation.');
    end
end

% Now as a last step to make things harder in C++, shuffle the rows
% of the measurement data.
measurement_data = measurement_data(randperm(size(measurement_data, 1)), :);

% centroids for debugging.
truth_data_centroid = mean(truth_data);
measurement_data_centroid = mean(measurement_data);

% Save off the truth/measurement data to a singular .txt file as well as a
% .mat file containing the known, true transform.
x_dat = [truth_data(:,1); NaN; measurement_data(:,1)];
y_dat = [truth_data(:,2); NaN; measurement_data(:,2)];
z_dat = [truth_data(:,3); NaN; measurement_data(:,3)];
tab = table(x_dat,y_dat,z_dat);
dat_name = sprintf('%s%s%s','iterative_closest_point_data_',datetime("today"),'.txt');
transf_name = sprintf('%s%s%s','iterative_closest_point_matlab_',datetime("today"),'.mat');
writetable(tab,dat_name,'Delimiter','\t','WriteRowNames',true);
save(transf_name);

plot_ze_data(truth_data,transf_truth_to_meas,measurement_data,n);


% ----------------FUNCTIONS-----------------------

function plot_ze_data(truth_data,transf,measurement_data,n)

    % Nice reusable function if you want to plot your data.
    figure
    plot3(truth_data(:,1),truth_data(:,2),truth_data(:,3),'bo');
    title('Truth Data')
    figure
    plot3(measurement_data(:,1),measurement_data(:,2),measurement_data(:,3),'ro');
    title('Measurement Data')
    figure 
    plot3(truth_data(:,1),truth_data(:,2),truth_data(:,3),'bo');
    hold on
    plot3(measurement_data(:,1),measurement_data(:,2),measurement_data(:,3),'ro');
    title('Mesured and Truth')
    legend('Truth Data','Measurement Data')
    
    % Transform the measurement data back to truth using inverse of the
    % transform then plot it.
    meas_to_truth = [];
    for i = 1:n
        m_to_t = inv(transf) * [measurement_data(i,:)'; 1];
        meas_to_truth = [meas_to_truth; m_to_t(1:3)'];
    end
    
    figure 
    plot3(truth_data(:,1),truth_data(:,2),truth_data(:,3),'bo');
    hold on
    plot3(meas_to_truth(:,1),meas_to_truth(:,2),meas_to_truth(:,3),'gx');
    title('Mesured (transformed back to truth) and Truth')
    legend('Truth Data','Transformed Measurement Data')

end

function rot_matrix = rotate_x(theta)
    
    rot_matrix = [1 0 0;
                  0 cos(theta) -sin(theta);
                  0 sin(theta) cos(theta)];

    % demand orthonormal.
    if sqrt((cos(theta)^2)+(sin(theta)^2)) < 0.99
        error('X isnt looking NORMAL')
    end
end

function rot_matrix = rotate_y(theta)
    
    rot_matrix = [cos(theta) 0 sin(theta);
                  0 1 0;  
                  -sin(theta) 0 cos(theta)];
    if sqrt((cos(theta)^2)+(sin(theta)^2)) < 0.99
        error('Y isnt looking NORMAL')
    end
end

function rot_matrix = rotate_z(theta)
    
    rot_matrix = [cos(theta) -sin(theta) 0;
                  sin(theta) cos(theta) 0;
                  0 0 1];
    if sqrt((cos(theta)^2)+(sin(theta)^2)) < 0.99
        error('Z isnt looking NORMAL')
    end
end