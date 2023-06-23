load('Path.mat')
figure("Name","RRT Path Planned")
    hMap=show(Env3D)
    hold on
    scatter3(hMap,Start(1),Start(2),Start(3),30,"cyan","filled")
    scatter3(hMap,Goal(1),Goal(2),Goal(3),30,"green","filled")
    scatter3(waypoints(2:end-1,1),waypoints(2:end-1,2),waypoints(2:end-1,3),"y","filled")
    plot3(path.States(:,1),path.States(:,2),path.States(:,3),"r-",LineWidth=2)
    hold off
