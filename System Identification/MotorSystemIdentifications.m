%% Load training data
file_name_training = 'pbrs500hz.txt';
data_matrix_training = readmatrix(file_name_training);

% Extract columns
timestamps_training = data_matrix_training(:,1);
pwm_data_training = data_matrix_training(:,3);
rpm_data_training = data_matrix_training(:,2);

% Convert timestamps to seconds (optional for plotting)
TimeVector_training = (timestamps_training - timestamps_training(1)) * 1e-6;

% Use uniform sampling period for iddata
Ts = 0.002; % 2 ms
system_data_training = iddata(rpm_data_training, pwm_data_training, Ts);

%% Load testing data
file_name_testing = 'pbrs500hztestdata.txt';
data_matrix_testing = readmatrix(file_name_testing);

timestamps_testing = data_matrix_testing(:,1);
pwm_data_testing = data_matrix_testing(:,3);
rpm_data_testing = data_matrix_testing(:,2);

TimeVector_testing = (timestamps_testing - timestamps_testing(1)) * 1e-6;

system_data_testing = iddata(rpm_data_testing, pwm_data_testing, Ts);
