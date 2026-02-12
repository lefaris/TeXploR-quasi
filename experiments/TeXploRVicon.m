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

j=0;
figure(2)
for i=1:10:840
    plot3(arc1X(i,1:4),arc1Y(i,1:4),arc1Z(i,1:4),Color=[0 j 1])
    hold on
    plot3(arc2X(i,1:4),arc2Y(i,1:4),arc2Z(i,1:4),Color=[1 0 j])
    hold on
    j = j+0.012;
end


% figure(2)
% for i=1:10:840
%     plot3(arc2Left1(i,1),arc2Left1(i,2),arc2Left1(i,3), 'LineWidth',4)
%     hold on
%     plot3(arc1Left2(i,1),arc1Left2(i,2),arc1Left2(i,3))
%     hold on
%     plot3(arc1Left3(i,1),arc1Left3(i,2),arc1Left3(i,3))
%     hold on
%     plot3(arc1Left4(i,1),arc1Left4(i,2),arc1Left4(i,3))
%     %hold on
% end
% 
% figure(3)
% plot3(arc2Left1(:,1),arc2Left1(:,2),arc2Left1(:,3), 'LineWidth',4)

% figure(2)
% 
% % Plotting the curved strut
% semiCircle2D = @(r) [r*cos(pi:0.01:2*pi);r*sin(pi:0.01:2*pi);...
%     zeros(size(cos(pi:0.01:2*pi)));ones(size(cos(pi:0.01:2*pi)))];
% plotXYZ = @(XYZ) plot3(XYZ(:,1),XYZ(:,2),XYZ(:,3));
% xyz = semiCircle2D(r);
% h=plotXYZ(xyz');
% % xlim
% hold on
% set(h,'Color','b','LineWidth',10)
% xyzRotated = TMatB*xyz;
% h=plotXYZ(xyzRotated');
% set(h,'Color','k','LineWidth',10)