
function IMUAnalysis
    file = load("IMU_dataC.mat");
    IMUData = file.IMU;

    time = double((IMUData.times - IMUData.times(1)))' ./ 10000.0; % in seconds
    N = IMUData.N;
    omega = double(IMUData.DATAf(6,:)'); % in rad/s
    omega = omega - mean(omega(1:200));
    
    theta = zeros(size(time));
    theta(1) = pi/2; % Start at 90 degrees.
    
    figure(1) ; clf();
    handle = plot(0, 0, 'b-');
    
    zoom on ;  grid on;
    for i = 2:N
        theta(i) =  theta(i-1) + (time(i) - time(i-1))*(omega(i) + omega(i-1))/2; % Using midpoint rule.
    end
    set(handle, 'xdata', 1:N,'ydata',theta .* (180/pi));

end

function PlotCallback
    global state
    state.pause = ~state.pause;
end