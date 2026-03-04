%Read in Vicon data for TeXploR and plot that shiz

opts = detectImportOptions('impact_full.csv');
%preview('impact_full.csv',opts)
% opts.SelectedVariableNames = [3:50]; 
% opts.DataRange = '6:845';
% M = readmatrix('impact_full.csv',opts)
M = readmatrix('impact_full.csv','Range','C6:AX845');

%Arc radius
r = 260; %Maybe 260mm?  Not entirely sure

%Read arc 1 data:
arc1Left1 = M(1:840, 1:3);
arc1Left2 = M(1:840, 4:6);
arc1Left3 = M(1:840, 7:9);
arc1Left4 = M(1:840, 10:12);

arc1Right1 = M(1:840, 13:15);
arc1Right2 = M(1:840, 16:18);
arc1Right3 = M(1:840, 19:21);
arc1Right4 = M(1:840, 22:24);

%Combine all of arc 1 points?
arc1LeftX = [arc1Left1(:,1),arc1Left2(:,1),arc1Left3(:,1),arc1Left4(:,1)];
arc1LeftY = [arc1Left1(:,2),arc1Left2(:,2),arc1Left3(:,2),arc1Left4(:,2)];
arc1LeftZ = [arc1Left1(:,3),arc1Left2(:,3),arc1Left3(:,3),arc1Left4(:,3)];

arc1RightX = [arc1Right1(:,1),arc1Right2(:,1),arc1Right3(:,1),arc1Right4(:,1)];
arc1RightY = [arc1Right1(:,2),arc1Right2(:,2),arc1Right3(:,2),arc1Right4(:,2)];
arc1RightZ = [arc1Right1(:,3),arc1Right2(:,3),arc1Right3(:,3),arc1Right4(:,3)];

arc1X = [(arc1Left1(:,1)+arc1Right1(:,1))/2, (arc1Left2(:,1)+arc1Right2(:,1))/2, (arc1Left3(:,1)+arc1Right3(:,1))/2, (arc1Left4(:,1)+arc1Right4(:,1))/2];
arc1Y = [(arc1Left1(:,2)+arc1Right1(:,2))/2, (arc1Left2(:,2)+arc1Right2(:,2))/2, (arc1Left3(:,2)+arc1Right3(:,2))/2, (arc1Left4(:,2)+arc1Right4(:,2))/2];
arc1Z = [(arc1Left1(:,3)+arc1Right1(:,3))/2, (arc1Left2(:,3)+arc1Right2(:,3))/2, (arc1Left3(:,3)+arc1Right3(:,3))/2, (arc1Left4(:,3)+arc1Right4(:,3))/2];

arc1p1 = [(arc1Left1(:,1)+arc1Right1(:,1))/2, (arc1Left1(:,2)+arc1Right1(:,2))/2, (arc1Left1(:,3)+arc1Right1(:,3))/2];
arc1p2 = [(arc1Left2(:,1)+arc1Right2(:,1))/2, (arc1Left2(:,2)+arc1Right2(:,2))/2, (arc1Left2(:,3)+arc1Right2(:,3))/2];
arc1p3 = [(arc1Left3(:,1)+arc1Right3(:,1))/2, (arc1Left3(:,2)+arc1Right3(:,2))/2, (arc1Left3(:,3)+arc1Right3(:,3))/2];
arc1p4 = [(arc1Left4(:,1)+arc1Right4(:,1))/2, (arc1Left4(:,2)+arc1Right4(:,2))/2, (arc1Left4(:,3)+arc1Right4(:,3))/2];

arc2p1 = [(arc2Left1(:,1)+arc2Right1(:,1))/2, (arc2Left1(:,2)+arc2Right1(:,2))/2, (arc2Left1(:,3)+arc2Right1(:,3))/2];
arc2p2 = [(arc2Left2(:,1)+arc2Right2(:,1))/2, (arc2Left2(:,2)+arc2Right2(:,2))/2, (arc2Left2(:,3)+arc2Right2(:,3))/2];
arc2p3 = [(arc2Left3(:,1)+arc2Right3(:,1))/2, (arc2Left3(:,2)+arc2Right3(:,2))/2, (arc2Left3(:,3)+arc2Right3(:,3))/2];
arc2p4 = [(arc2Left4(:,1)+arc2Right4(:,1))/2, (arc2Left4(:,2)+arc2Right4(:,2))/2, (arc2Left4(:,3)+arc2Right4(:,3))/2];

arc1 = [arc1p1, arc1p2, arc1p3, arc1p4];
arc2 = [arc2p1, arc2p2, arc2p3, arc2p4];

%Read arc 2 data:
arc2Left1 = M(1:840, 25:27);
arc2Left2 = M(1:840, 28:30);
arc2Left3 = M(1:840, 31:33);
arc2Left4 = M(1:840, 34:36);

arc2Right1 = M(1:840, 37:39);
arc2Right2 = M(1:840, 40:42);
arc2Right3 = M(1:840, 43:45);
arc2Right4 = M(1:840, 46:48);

%Combine all of arc 2 points?
arc2LeftX = [arc2Left1(:,1),arc2Left2(:,1),arc2Left3(:,1),arc2Left4(:,1)];
arc2LeftY = [arc2Left1(:,2),arc2Left2(:,2),arc2Left3(:,2),arc2Left4(:,2)];
arc2LeftZ = [arc2Left1(:,3),arc2Left2(:,3),arc2Left3(:,3),arc2Left4(:,3)];

arc2RightX = [arc2Right1(:,1),arc2Right2(:,1),arc2Right3(:,1),arc2Right4(:,1)];
arc2RightY = [arc2Right1(:,2),arc2Right2(:,2),arc2Right3(:,2),arc2Right4(:,2)];
arc2RightZ = [arc2Right1(:,3),arc2Right2(:,3),arc2Right3(:,3),arc2Right4(:,3)];

arc2X = [(arc2Left1(:,1)+arc2Right1(:,1))/2, (arc2Left2(:,1)+arc2Right2(:,1))/2, (arc2Left3(:,1)+arc2Right3(:,1))/2, (arc2Left4(:,1)+arc2Right4(:,1))/2];
arc2Y = [(arc2Left1(:,2)+arc2Right1(:,2))/2, (arc2Left2(:,2)+arc2Right2(:,2))/2, (arc2Left3(:,2)+arc2Right3(:,2))/2, (arc2Left4(:,2)+arc2Right4(:,2))/2];
arc2Z = [(arc2Left1(:,3)+arc2Right1(:,3))/2, (arc2Left2(:,3)+arc2Right2(:,3))/2, (arc2Left3(:,3)+arc2Right3(:,3))/2, (arc2Left4(:,3)+arc2Right4(:,3))/2];

%Plot the discrete points from arc 1
figure(1)
plot3(arc1Left1(:,1),arc1Left1(:,2),arc1Left1(:,3))
hold on
plot3(arc1Left2(:,1),arc1Left2(:,2),arc1Left2(:,3))
hold on
plot3(arc1Left3(:,1),arc1Left3(:,2),arc1Left3(:,3))
hold on
plot3(arc1Left4(:,1),arc1Left4(:,2),arc1Left4(:,3))
hold on
j=0;

% And then the "straight rod" representation
for i=1:10:840
    plot3(arc1X(i,1:4),arc1Y(i,1:4),arc1Z(i,1:4),Color=[0 j 1])
    hold on
    plot3(arc2X(i,1:4),arc2Y(i,1:4),arc2Z(i,1:4),Color=[1 0 j])
    hold on
    j = j+0.012;
end




%Plot the discrete points from arc 1
figure(2)
plot3(arc1Left1(:,1),arc1Left1(:,2),arc1Left1(:,3))
hold on
plot3(arc1Left2(:,1),arc1Left2(:,2),arc1Left2(:,3))
hold on
plot3(arc1Left3(:,1),arc1Left3(:,2),arc1Left3(:,3))
hold on
plot3(arc1Left4(:,1),arc1Left4(:,2),arc1Left4(:,3))
hold on
j=0;

% And then the realistic curved representation
for i=1:10:840
    [center1, radius1, xarc1, yarc1, zarc1] = triangle2circle(arc1p1(i,:), arc1p2(i,:), arc1p4(i,:));
    plot3(xarc1,yarc1,zarc1, 'Color', [0 j 1])
    hold on
    [center2, radius2, xarc2, yarc2, zarc2] = triangle2circle(arc2p1(i,:), arc2p2(i,:), arc2p4(i,:));
    plot3(xarc2,yarc2,zarc2, 'Color', [1 0 j])
    hold on
    j = j+0.012;
end


%Plot the discrete points from arc 1
figure(3)
plot3(arc1Left1(:,1),arc1Left1(:,2),arc1Left1(:,3))
hold on
plot3(arc1Left2(:,1),arc1Left2(:,2),arc1Left2(:,3))
hold on
plot3(arc1Left3(:,1),arc1Left3(:,2),arc1Left3(:,3))
hold on
plot3(arc1Left4(:,1),arc1Left4(:,2),arc1Left4(:,3))
hold on
j=0;

% And then the realistic curved representation
for i=1:10:840
    [center1, radius1, xarc1, yarc1, zarc1] = triangle2semicircle(arc1p1(i,:), arc1p2(i,:), arc1p4(i,:));
    plot3(xarc1,yarc1,zarc1, 'Color', [0 j 1])
    hold on
    [center2, radius2, xarc2, yarc2, zarc2] = triangle2semicircle(arc2p1(i,:), arc2p2(i,:), arc2p4(i,:));
    plot3(xarc2,yarc2,zarc2, 'Color', [1 0 j])
    hold on
    j = j+0.012;
end


%Plot the discrete points from arc 1
figure(4)
plot3(arc1Left1(:,1),arc1Left1(:,2),arc1Left1(:,3))
hold on
plot3(arc1Left2(:,1),arc1Left2(:,2),arc1Left2(:,3))
hold on
plot3(arc1Left3(:,1),arc1Left3(:,2),arc1Left3(:,3))
hold on
plot3(arc1Left4(:,1),arc1Left4(:,2),arc1Left4(:,3))
hold on
j=0;

% And then the realistic curved representation
for i=1:10:840
    semicircle(arc1(i,:));
    hold on
    semicircle(arc2(i,:));
    hold on
    j = j+0.012;
end
% 
% % figure(2)
% % 
% % % Plotting the curved strut
% semiCircle2D = @(r, center) [r*cos(pi:0.01:2*pi);r*sin(pi:0.01:2*pi);...
%     zeros(size(cos(pi:0.01:2*pi)));ones(size(cos(pi:0.01:2*pi)))];
% plotXYZ = @(XYZ) plot3(XYZ(:,1),XYZ(:,2),XYZ(:,3));
% xyz = semiCircle2D(r, center);
% h=plotXYZ(xyz');
% % xlim
% hold on
% set(h,'Color','b','LineWidth',10)
% xyzRotated = TMatB*xyz;
% h=plotXYZ(xyzRotated');
% set(h,'Color','k','LineWidth',10)

% Specify circle vectors
function [center, radius, x, y, z] = triangle2circle(P1, P2, P3)
    % Describe the edges of the triangle by defining two vectors from P1
    u = P2 - P1;
    v = P3 - P1;

    % Define normal vector (orientation in 3D)
    norm_temp = cross(u,v);
    normal = norm_temp / norm(norm_temp);

    % Set up linear set of equations based off three points and solve it to
    % generate the circle center, then use that to calculate the radius
    A = [2*u; 2*v; normal];
    b = [dot(P2, P2) - dot(P1, P1); dot(P3, P3) - dot(P1, P1); dot(normal, P1)];
    center = (A \ b)'; % Solve the linear system
    radius = norm(center - P1);

    % 6 variables to parameterize 3D circle
    phi = atan2(normal(2),normal(1)); % azimuth angle, in [-pi, pi]
    theta = atan2(sqrt(normal(1)^2 + normal(2)^2), normal(3)); % zenith angle, in [0,pi]    
    t = 0:pi/32:2*pi;
    x = center(1) - radius*( cos(t)*sin(phi) + sin(t)*cos(theta)*cos(phi) );
    y = center(2) + radius*( cos(t)*cos(phi) - sin(t)*cos(theta)*sin(phi) );
    z = center(3) + radius*sin(t)*sin(theta);
    % plot3(x,y,z,'r')
    % patch(x,y,z,'r')
    % hold on
end

% Specify semi-circle vectors?
function [center, radius, x, y, z] = triangle2semicircle(P1, P2, P3)
    % Describe the edges of the triangle by defining two vectors from P1
    u = P2 - P1;
    v = P3 - P1;

    % Define normal vector (orientation in 3D)
    norm_temp = cross(u,v);
    normal = norm_temp / norm(norm_temp);

    % Set up linear set of equations based off three points and solve it to
    % generate the circle center, then use that to calculate the radius
    A = [2*u; 2*v; normal];
    b = [dot(P2, P2) - dot(P1, P1); dot(P3, P3) - dot(P1, P1); dot(normal, P1)];
    center = (A \ b)'; % Solve the linear system
    radius = norm(center - P1);

    % 6 variables to parameterize 3D circle
    phi = atan2(normal(2),normal(1)); % azimuth angle, in [-pi, pi]
    theta = atan2(sqrt(normal(1)^2 + normal(2)^2), normal(3)); % zenith angle, in [0,pi]    
    t = pi/2:pi/32:3*pi/2;
    x = center(1) - radius*( cos(t)*sin(phi) + sin(t)*cos(theta)*cos(phi) );
    y = center(2) + radius*( cos(t)*cos(phi) - sin(t)*cos(theta)*sin(phi) );
    z = center(3) + radius*sin(t)*sin(theta);


    % plot3(x,y,z,'r')
    % patch(x,y,z,'r')
    % hold on
end


function semicircle(P)
    % 2. Find Plane and Center (Simplified fit)
    C = mean(P, 1); % Initial guess for center
    P_centered = P - C;
    [~, ~, V] = svd(P_centered);
    normal = V(:,3); % Plane normal
    u = V(:,1);      % Basis vector 1
    v = V(:,2);      % Basis vector 2
    
    % 3. Calculate Radius (Average distance from mean center)
    R = mean(sqrt(sum(P_centered.^2, 2)));
    
    % 4. Generate Semi-circle points (0 to pi radians)
    theta = linspace(0, pi, 100);
    arc_points = C + R*cos(theta)'*u' + R*sin(theta)'*v';
    
    % 5. Plot
    plot3(arc_points(:,1), arc_points(:,2), arc_points(:,3), 'r', 'LineWidth', 2);
    hold on;
    scatter3(P(:,1), P(:,2), P(:,3), 'filled', 'b'); % Original points
    axis equal; grid on;

end