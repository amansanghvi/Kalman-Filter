function [x, y, theta, time, N] = PositionData
    file1 = load("IMU_dataC.mat");
    IMUData = file1.IMU;
    time = double((IMUData.times - IMUData.times(1)))' ./ 10000.0; % in seconds
    N = IMUData.N;
    
    file2 = load("Speed_dataC.mat");
    EncoderData = file2.Vel;
    
    v = EncoderData.speeds' - mean(EncoderData.speeds(1:200));
    omega = double(IMUData.DATAf(6,:)'); % in rad/s
    omega = omega - mean(omega(1:200));
    
    x = zeros(size(time));
    y = zeros(size(time));
    theta = zeros(size(time));
    theta(1) = pi/2; % Start at 90 degrees.
    
    for i = 2:N
        dt = time(i) - time(i-1);
        theta(i) =  theta(i-1) + dt*(omega(i) + omega(i-1))/2; % Using midpoint rule.
        avgTheta = mean(theta(i-1:i));
        x(i) = x(i-1) + dt*(cos(avgTheta)*(v(i) + v(i-1))/2);
        y(i) = y(i-1) + dt*(sin(avgTheta)*(v(i) + v(i-1))/2);
    end
    
%     plot(x, y);
end