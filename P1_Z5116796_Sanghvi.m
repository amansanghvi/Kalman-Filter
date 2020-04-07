
function Project1(file)

global state;            % I use a global variable, to easily share it, in many functions. You may obtain similar functionality using other ways.
global DEFS;

state.flagPause = 0;
state.completed = 0;


if ~exist('file','var'), file ='Laser__2C.mat'; end
  
    file1 = load("IMU_dataC.mat");
    IMUData = file1.IMU;
    posTimes = (IMUData.times - IMUData.times(1)); % in seconds
    Npositions = IMUData.N;
    
    ScanData = load(file);
    dataL = ScanData.dataL;
    scanTimes = double(dataL.times - IMUData.times(1));

    file2 = load("Speed_dataC.mat");
    EncoderData = file2.Vel;

    v = EncoderData.speeds' - mean(EncoderData.speeds(1:1000)); % Noise over the first 5 seconds.
    omega = double(IMUData.DATAf(6,:)'); % in rad/s
    omega = omega - mean(omega(1:1000));

    DEFS.countsPerSec = 10000;
    figure(1) ; clf();
    zoom on ;  grid on;
    plt.allPoints = plot(0,0,'.b');      % to be used for showing the laser points
    hold on;
    plt.hr = plot(0,0, '+r');      % to be used for showing the laser points
    hold on;
    plt.poleCentre = plot(0,0, '*g');      % to be used for showing the laser points
    
    axis([-10,10,0,20]);                         % focuses the plot on this region (of interest, close to the robot)
    xlabel('x (meters)');
    ylabel('y (meters)');
    
    plt.title = title('');           % create an empty title..
    zoom on ;  grid on;
    
    disp('Showing laser scans, in CARTESIAN representation');
    
    fprintf('\nThere are [ %d ] laser scans in this dataset (file [%s])\n',dataL.N,file);
    
    
    uicontrol('Style','pushbutton','String','Pause/Cont.','Position',[10,1,80,20],'Callback',{@MyCallBackA,1});
    uicontrol('Style','pushbutton','String','END Now','Position',[90,1,80,20],'Callback',{@MyCallBackA,2});

    figure(2) ; clf();
    plt.position = plot(0,0, '-k');
    hold on;
    plt.absolutePoleLocation = plot(0, 0, '+k');
    hold on;
    plt.measuredPoleLocation = plot(0, 0, '+r');
    
    DEFS.Npoles = 5;
    axis([-5,5,-2,8]);                         % focuses the plot on this region (of interest, close to the robot)
    xlabel('x (meters)');
    ylabel('y (meters)');
    
    plt.title = title('Position');           % create an empty title..
    for i=1:DEFS.Npoles
        plt.absPoles(i) = text(0, 0, '', 'VerticalAlignment', 'top', 'FontWeight', 'bold');
        plt.relPoles(i) = text(0, 0, '', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'Color', 'red', 'Tag', string(i));
    end
   
    zoom on ;  grid on;
    
    
    % Now, loop through the avaialable scans..
    x = zeros([Npositions, 1]);
    y = zeros([Npositions, 1]);
    theta = zeros([Npositions, 1]);
    theta(1) = pi/2; % Start at 90 degrees.
    
    Nscans = dataL.N;

    MyProcessingOfScan(dataL.Scans(:,1982),100,plt,1);   % some function to use the data...
    numProcessed = 1;
    poleEstimations = zeros([DEFS.Npoles, 300, 2]);
    polePos = zeros(DEFS.Npoles, 2);
    posScan = 1;
    for i = 0:10:2228850

        if (state.completed), break; end
        while (state.flagPause), pause(0.2); end

        scanTime = find(scanTimes == i);
        posTime = find(posTimes == i);
        
        if (numel(posTime) && posTime > 1)
            posScan = posTime;
            [x, y, theta] = getNextPos(posScan, posTimes, x, y, v, theta, omega); 
            if (mod(posScan, 10))
                carFrame = getCarFrame(x(posScan), y(posScan), theta(posScan) -pi/2);
                set(plt.position, 'xdata', [x(1:posScan); carFrame(:,1)], 'ydata', [y(1:posScan); carFrame(:,2)]);
            end
        end
        if (numel(scanTime)) 
            scan_i = dataL.Scans(:,scanTime);
            
            [num, centres, ~, ~] = MyProcessingOfScan(scan_i,double(scanTimes(scanTime))/DEFS.countsPerSec,plt,scanTime);   % some function to use the data...
            if (scanTime <= 300) % First 300 scans to estimate pole placement.
                [poleEstimations, polePos(:,1), polePos(:,2)] = estimateInitPolePos(poleEstimations, numProcessed, centres);
                set(plt.absolutePoleLocation, 'xdata', polePos(:,1), 'ydata', polePos(:,2));
                for pole=1:DEFS.Npoles
                   set(plt.absPoles(pole), 'Position', [polePos(pole,1), polePos(pole,2)], 'String', char('A' + pole - 1)); 
                end
            elseif num > 0
                relativePoles = getRelativePoles(centres, x(posScan), y(posScan), theta(posScan) - pi/2);
                set(plt.measuredPoleLocation, 'xdata', relativePoles(:, 1), 'ydata', relativePoles(:, 2)); 
                for pole=1:num
                    distances = pdist2(relativePoles(pole, :), polePos); % Each new pole overwrites previous pole.
                    closestPole = find(distances < 0.4);
                    if numel(closestPole) > 0
                        if (numel(closestPole) > 1) 
                            closestPole = find(distances == min(distances));
                        end
                        set(plt.relPoles(closestPole), 'Position', relativePoles(pole, :), 'String', char('A' + closestPole - 1), 'Tag', string(scanTime + closestPole))
                    end
                end
                for pole=1:DEFS.Npoles
                    if plt.relPoles(pole).Tag ~= string(scanTime + pole)
                        set(plt.relPoles(pole), 'String', '');
                    end
                end
            end

            numProcessed = numProcessed + 1;
            pause(0.01);                   % wait for ~1ms (approx.)
        end
    end

    fprintf('\nDONE!\n');

    % Read Matlab Help for explanation of FOR loops, and function double( ) and pause()


    return;
end
%-----------------------------------------

function [x, y, theta] = getNextPos(posScan, posTimes, x, y, v, theta, omega)

    global DEFS;
    dt = (double(posTimes(posScan)) - double(posTimes(posScan-1)))/DEFS.countsPerSec;
            
    theta(posScan) =  theta(posScan-1) + dt*(omega(posScan-1)); % Using midpoint rule.

    avgTheta = mean(theta(posScan-1:posScan));

    x(posScan) = x(posScan-1) + dt*(cos(avgTheta)*(v(posScan-1)));
    y(posScan) = y(posScan-1) + dt*(sin(avgTheta)*(v(posScan-1)));

end

function relativePoles = getRelativePoles(centres, x, y, theta)
    relativePoles = zeros(size(centres));
    for pole=1:size(centres(:,1))
        rotatedPos = [cos(theta) -sin(theta); sin(theta) cos(theta)]*centres(pole,:)';
        translatedPos = rotatedPos' + [x y];
        relativePoles(pole, :) = translatedPos;
    end
end

function [n, centres, diameters, colors]= MyProcessingOfScan(scan,t,mh,i)
    MaskLow13Bits = uint16(2^13-1); % mask for extracting the range bits.
    maskE000 = bitshift(uint16(7),13);
    % the lower 13 bits are for indicating the range data (as provided by this sensor)
    
    rangesA = bitand(scan,MaskLow13Bits);
    % rangesA now contains the range data of the scan, expressed in CM, having uint16 format.
    
    % now I convert ranges to meters, and also to floating point format
    ranges    = 0.01*double(rangesA); 

    angles = (0:0.5:180)'* pi/180;         % associated angle, for each individual range in a scan
    X = cos(angles).*ranges;
    Y = sin(angles).*ranges + 0.46; % This is the offset of laser scanner from base.
    
    intensities = bitand(scan,maskE000);
    highReflect = find(intensities ~= 0); % Highly Reflective points

    % and then refresh the data in the plots...
    set(mh.hr, 'xdata', X(highReflect) ,'ydata', Y(highReflect));
    set(mh.allPoints, 'xdata', X ,'ydata', Y);
    
    clusterAssignment = createClusters(X, Y, 0.2);
    clustersOfInterest = unique(clusterAssignment(highReflect));    
    
    clusters = arrayfun(@(c) [X(clusterAssignment == c) Y(clusterAssignment == c)], ...
        clustersOfInterest, 'UniformOutput', false);
    
    [cellCentres, cellDiameters] = arrayfun(@getClusterCentre, clusters, 'UniformOutput', false);

    allDiameters = cell2mat(cellDiameters);
    allCentres = cell2mat(cellCentres);
    validSize = find(allDiameters < 0.2);
    
    centres = allCentres(validSize, :);
    diameters = allDiameters(validSize);
    colors = zeros(size(validSize)) + 1;
    n = numel(validSize);
    
    
    if (n > 0)
        set(mh.poleCentre, 'xdata', centres(:,1) ,'ydata', centres(:,2));
    else
        set(mh.poleCentre, 'xdata', [] ,'ydata', []);
    end
   
    % Find clusters with at least one reflective point.
    % Then we find the indices in X & Y of the points in the cluster.

    s= sprintf('Laser scan # [%d] at time [%.3f] secs',i,t);
    set(mh.title,'string',s);

    return;
end

function clusterAssignment = createClusters(X, Y, cutoff)
    cutoff = cutoff^2;
    tolerance = 2^2; % Can tolerate 200% difference between distances between points
    clusterAssignment = zeros(size(X));
    cluster = 0;
    rangeTolerance = min([((X(1) - X(2))^2 + (Y(1) - Y(2))^2)*tolerance, ((X(2) - X(3))^2 + (Y(2) - Y(3))^2)*tolerance, cutoff]);
    
    % Iterate from start to finish. If the distance from the next point is
    % significantly shorter than the distance from the previous point, the
    % next point cluster number is incremented.
    for i=2:(size(X) - 1)
        prevDist = (X(i) - X(i-1))^2 + (Y(i) - Y(i-1))^2;
        nextDist = (X(i) - X(i+1))^2 + (Y(i) - Y(i+1))^2;
        if (nextDist < prevDist)
           rangeTolerance = min([nextDist*tolerance, cutoff]);
        end
        if (prevDist < rangeTolerance) 
            clusterAssignment(i) = cluster;
        else
            cluster = cluster + 1;
            clusterAssignment(i) = cluster;
        end
        
    end
    
    if (((X(end-1) - X(end))^2 + (Y(end-1) - Y(end))^2)*tolerance < rangeTolerance)
        clusterAssignment(end) = cluster;
    else
        clusterAssignment(end) = cluster+1;
    end
end

function [centre, diameter] = getClusterCentre(pointsCell)
    
    points = cell2mat(pointsCell);
    R = sqrt(points(:, 1).^2 + points(:, 2).^2);
    theta = atan2(points(:, 2), points(:, 1));
    
    diameter = pdist([points(1, :) ; points(end, :)]);
    newTheta = mean(theta);   
    newR = min(R) + diameter/2;
    
    centre = [newR*cos(newTheta) newR*sin(newTheta)];
    
end

function [poleEstimations, x, y] = estimateInitPolePos(poleEstimations, numProcessed, centres)
    global DEFS;
    for pole=1:DEFS.Npoles
        poleEstimations(pole, numProcessed, 1:2) = [centres(pole, 1) centres(pole, 2)];
    end
    x = mean(poleEstimations(:, 1:numProcessed, 1), 2);
    y = mean(poleEstimations(:, 1:numProcessed, 2), 2);
end

function carFrame = getCarFrame(x, y, theta)
    carFrame = [0, 0; 0.12, 0 ; 0, 0.46; -0.12, 0; 0, 0];
    carFrame = ([cos(theta) -sin(theta); sin(theta) cos(theta)]*carFrame')';
    carFrame(:,1) = carFrame(:, 1) + x;
    carFrame(:,2) = carFrame(:, 2) + y;
end

function MyCallBackA(~,~,x)   
    global state;
    switch(x)
        case(1)
            state.flagPause = ~state.flagPause; %Switch ON->OFF->ON -> and so on.
        case(2)
            disp('you pressed "END NOW"');
            state.completed = 1;  
    end

end
