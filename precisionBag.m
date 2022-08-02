clear all

% This file shows how to process rosbag files in matlab. The script
% loads 4 files from the Precision dataset/Data (July 2021)/Day 1 and then 
% plots the gps coordinates in cartesian space

% load rosbag files
% to see the available topics use 'bag.AvailableTopics'
bag_0 = rosbag("2021-07-28-11-39-41_0.bag");
bag_1 = rosbag("2021-07-28-11-40-41_1.bag");
bag_2 = rosbag("2021-07-28-11-41-41_2.bag");
bag_3 = rosbag("2021-07-28-11-42-41_3.bag");

%% Process RTK GPS measurements

% extract the rtk gps topic into a new bag object
bagGps_0 = select(bag_0,"Topic","/gps_message");
bagGps_1 = select(bag_1,"Topic","/gps_message");
bagGps_2 = select(bag_2,"Topic","/gps_message");
bagGps_3 = select(bag_3,"Topic","/gps_message");

% read the rtk gps messages
msgsGps_0 = readMessages(bagGps_0,"DataFormat","struct");
msgsGps_1 = readMessages(bagGps_1,"DataFormat","struct");
msgsGps_2 = readMessages(bagGps_2,"DataFormat","struct");
msgsGps_3 = readMessages(bagGps_3,"DataFormat","struct");

% create an nmeaParser system object specifying the message ID as "GGA"
pnmea = nmeaParser("MessageID","GGA");

% set the first message from the first bag as the origin
gps = pnmea(msgsGps_0{1,1}.Sentence_);
origin = [gps.Latitude, gps.Longitude, gps.Altitude];

% extract rtk gps data
for i = 1:length(msgsGps_0)
    gps = pnmea(msgsGps_0{i,1}.Sentence_);
    [x(i), y(i)] = convert(gps, origin);
end
for j = 1:length(msgsGps_1)
    gps = pnmea(msgsGps_1{j,1}.Sentence_);
    [xx(j), yy(j)] = convert(gps, origin);
end
for k = 1:length(msgsGps_2)
    gps = pnmea(msgsGps_2{k,1}.Sentence_);
    [xxx(k), yyy(k)] = convert(gps, origin);
end
for m = 1:length(msgsGps_3)
    gps = pnmea(msgsGps_3{m,1}.Sentence_);
    [xxxx(m), yyyy(m)] = convert(gps, origin);
end
% Plot the gps measurements from each files in a separate color
figure
plot(x,y,'k')
hold on
plot(xx,yy,'b')
plot(xxx,yyy,'r')
plot(xxxx,yyyy,'g')
hold off
axis('equal')
xlabel('x (meters)')
ylabel('y (meters)')

%% Process IMU measurements

% extract IMU topics into new bag objects
bagGyro_0 = select(bag_0,"Topic","/camera/gyro/sample");
bagAccel_0 = select(bag_0,"Topic","/camera/accel/sample");

% read the IMU messages
msgsGyro_0 = readMessages(bagGyro_0,"DataFormat","struct");
msgsAccel_0 = readMessages(bagAccel_0,"DataFormat","struct");

% extract angular velocity and time
for i = 1:length(msgsGyro_0)
    gX(i,1) = msgsGyro_0{i,1}.AngularVelocity.X;
    gY(i,1) = msgsGyro_0{i,1}.AngularVelocity.Y;
    gZ(i,1) = msgsGyro_0{i,1}.AngularVelocity.Z;
    gT(i,1) = msgsGyro_0{i,1}.Header.Stamp.Sec;
end

% extract linear acceleration and time
for i = 1:length(msgsAccel_0)
    a(i,1) = msgsAccel_0{i,1}.LinearAcceleration.X;
    a(i,2) = msgsAccel_0{i,1}.LinearAcceleration.Y;
    a(i,3) = msgsAccel_0{i,1}.LinearAcceleration.Z;
    aT(i) = msgsAccel_0{i,1}.Header.Stamp.Sec;
end

%% Function to convert geographic coordinates to local Cartesian coordinates
function [xEast, yNorth] = convert(gps, origin)
     
    [xEast, yNorth] = latlon2local(gps.Latitude, gps.Longitude,...
        gps.Altitude, origin);

end


