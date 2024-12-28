function KalmanFilterDemo()
    % Define constants
    H = 1;
    A = 1;
    R = 3;
    Q = 0.05;
    Total_volume = 0;
    reading = 0;

    % Initialize parameters
    KF_param.estimate = 2659; % Initial estimate
    KF_param.err_covariance = 9; % Initial error covariance

    % Example measurements
    % note : "d:/mathlab/raw_value.txt" replace with your path of raw_value file.
    measurements = load("d:/mathlab/raw_value.txt");
    %measurements = measurements(1:3:end);
 % Preallocate for storage
    estimates = zeros(size(measurements));
    volume = zeros(size(measurements));

    % Run Kalman filter on measurements
    for i = 1:length(measurements)
        KF_param = Kalman_filter(measurements(i), KF_param, A, H, Q, R);
        estimates(i) = round(KF_param.estimate); % Store the estimate
        reading = (1 - 0.2) * reading + (0.2 * estimates(i));
        dp = ( estimates(i) - 2656) / 100.0;
        if(dp < 0)
            flow =0;
        elseif(dp < 0.46)
            flow = (0.1512 * dp * dp * dp) - (3.3424 * dp * dp) + (41.657 * dp);
        else
            flow = ((0.72 * dp * dp * dp) - (8.01 * dp * dp) + (56.7 * dp) - 6.18);
        end
        volume(i) = volume(i) + (flow/60);
        Total_volume = Total_volume + volume(i);
        % fprintf('Iteration %d: Measured = %f, Estimate = %f\n', i, measurements(i), KF_param.estimate);
    end
    fprintf('volume = %d', Total_volume);

    % Plotting the results
    figure;
    plot(1:length(measurements), measurements, '-o', 'DisplayName', 'Measured Value');
    hold on;
    plot(1:length(measurements), estimates,'-*', 'DisplayName', 'Estimated Value');
    xlabel('Iteration');
    ylabel('Value');
    title('Kalman Filter: Measured vs Estimated');
    legend('Location', 'Best');
    grid on;
end

function KF_param = Kalman_filter(measurement, KF_param, A, H, Q, R)
    % Kalman filter main function
    
    % Prediction step
    KF_predict.state = A * KF_param.estimate;
    KF_predict.err_covariance = A* KF_param.err_covariance * A + Q;

    % Update step
    kalman_gain = (KF_predict.err_covariance * H) / (H * KF_predict.err_covariance * H + R);
    KF_param.estimate = KF_predict.state + kalman_gain * (measurement - H * KF_predict.state); 
    KF_param.err_covariance = KF_predict.err_covariance * (1 - kalman_gain * H);
end
