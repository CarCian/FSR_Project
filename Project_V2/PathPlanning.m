clc
clear
clc

%% Loading the omap object
% A desired omap is loaded

% run("OMap_3D.m")
% or
load("Env3D.mat");

Env_3D = Env3D;
Env_3D.FreeThreshold = Env_3D.OccupiedThreshold; % Consider unknown spaces to 
                                                 % be unoccupied

%% Definition of the starting and final poses

% Define a couple of position as Start and Goal ones and check if they are
% free. If they are not new starting and final positions are found through
% randomic generation and their availability is checked.

Start=[0 0 20];
Goal=[80 60 100];

while checkOccupancy(Env_3D,Start)
    Start=[randi([1 10],1) randi([1 10],1) randi([1 20],1)];
    checkOccupancy(Env_3D,Start);
end

while checkOccupancy(Env_3D,Goal)
    Goal=[randi([50 100],1) randi([50 100],1) randi([1 150],1)];
    checkOccupancy(Env_3D,Goal);
end

% Add starting and goal yaw angles

Start=[Start 1 0 0 0];
Goal=[Goal 1 0  0 0];

% The Orientation in the navPath obj. is specified though quaternions. We
% can convert it to Euler Angles through:
% quat=[path.States(1,4) path.States(1,5) path.States(1,6) path.States(1,7)]
% eul=quat2eul(quat,'XYZ')

xSt=Start(1);
ySt=Start(2);
zSt=Start(3);

xGo=Goal(1);
yGo=Goal(2);
zGo=Goal(3);

qi=[xSt ySt zSt];
qf=[xGo yGo zGo];

N=100;
K=0;
delta=20;
FOUND=0;
MaxHeight=100;

while(FOUND==0)&&(K<15)
    FAILURE=0;
    j=1;
    i=1;
    C_Free=[qi;qf];
    RoadMap=[qi 0];

    for j=1:N
        still_not_delta=1;
        free=1;
        q_rand=[randi([1 mapW],1) randi([1 mapL],1) randi([1 MaxHeight],1)];
        dist=[];
        
        for i=1:size(RoadMap,1)
            dist=[dist sqrt((q_rand(1)-RoadMap(i,1))^2+(q_rand(2)-RoadMap(i,2))^2+(q_rand(3)-RoadMap(i,3))^2)];
        end

        [M,I]=min(dist);
        q_near=RoadMap(I,:);

        % q_new will be the point at distance delta along the line to which q_near
        % and q_rand both belong.

        if M>0
            alpha=atan2(q_rand(2)-q_near(2),q_rand(1)-q_near(1));
            beta=atan2(q_rand(3)-q_near(3),sqrt((q_rand(1)-q_near(1))^2+(q_rand(2)-q_near(2))^2));
            q_new=[q_near(1)+delta*cos(alpha) q_near(2)+delta*sin(alpha) q_near(3)+delta*tan(beta)];

        % If q_new is inside the map and is an admissible configuration, the cycle
        % verifies that there are no inadmissible points along the line between
        % q_near and q_new, while temporarily saving these points in a vector. If
        % in the end all these points are admissible they are all added to C_Free,
        % while q_new is added to the roadmap.

        if (q_new(1)>0) && (q_new(1)<mapW) && (q_new(2)>0) && (q_new(2)<mapL) && (q_new(3)>0) && (q_new(3)<MaxHeight) && ~checkOccupancy(Env_3D,q_new)   
            while free && still_not_delta
                T=[];
                for r=1:delta
                    q_t=[(q_near(1)+r*cos(alpha)) (q_near(2)+r*sin(alpha)) (q_near(3)+r*tan(beta))];
    
                    if ~checkOccupancy(Env_3D,q_t)
                        T=cat(1,T,q_t);
                    else if checkOccupancy(Env_3D,q_t)
                            free=0;
                         end
                    end

                end
                if free==1
                    C_Free=cat(1,C_Free,T);
                    RoadMap=[RoadMap; [q_new I]];
                end
                still_not_delta=0;
            end

        end
        end
    end
    
    % After all the iterations we now need to link qf to our roadmap. This is
    % done by searching for the nearest point to qf of our roadmap and by
    % verifying that all the points along the line connecting them are
    % admissible, similarly to what we have done previously.

    distf=[];
    for i=1:size(RoadMap,1)
        distf=[distf sqrt((qf(1)-RoadMap(i,1))^2+(qf(2)-RoadMap(i,2))^2+(qf(3)-RoadMap(i,3))^2)];
    end

    [Mf,If]=min(distf);
    q_near_f=RoadMap(If,:);
    if Mf>0 && Mf<delta
        alpha_f=atan2(qf(2)-q_near_f(2),qf(1)-q_near_f(1));
        beta_f=atan2(qf(3)-q_near_f(3),sqrt((qf(1)-q_near_f(1))^2+(qf(2)-q_near_f(2))^2));
        still_not_Mf=1;
        free_f=1;
        while free_f && still_not_Mf
            Tf=[];
            for r=1:Mf
                q_t=[(q_near_f(1)+r*cos(alpha_f)) (q_near_f(2)+r*sin(alpha_f)) (q_near_f(3)+r*tan(beta_f))];
                if ~checkOccupancy(Env_3D,q_t)
                    Tf=cat(1,Tf,q_t);
                else if checkOccupancy(Env_3D,q_t)
                    free_f=0;
                end
                end
            end
            if free_f==1
                C_Free=cat(1,C_Free,Tf);
                RoadMap=[RoadMap; [qf If]];
                FOUND=1;
            end
            still_not_Mf=0;
        end
    end

    if FOUND==0
        FAILURE=1;
        N=N+20;
        K=K+1;
    end
end

if FOUND==1
    path_qs_qg=[];
    n=length(RoadMap);

    while n~=0
        path_qs_qg=[RoadMap(n,:); path_qs_qg];
        n=RoadMap(n,4);
    end

    ss = stateSpaceSE3([0 mapW; 0 mapL; 0 MaxHeight; inf inf; inf inf; inf inf; inf inf]);
    path = navPath(ss);
    waypoints=[Start];
    for i=2:length(path_qs_qg)-1
        waypoints = [waypoints; path_qs_qg(i,1) path_qs_qg(i,2) path_qs_qg(i,3) 1 0 0 0];
    end
    waypoints = [waypoints; Goal];
    append(path,waypoints);
    %interpolate(path,2*length(path.States)-2);
    
    figure("Name","RRT Path Planned")
    hMap = show(Env_3D);
    hold on
    scatter3(hMap,Start(1),Start(2),Start(3),30,"cyan","filled")
    scatter3(hMap,Goal(1),Goal(2),Goal(3),30,"green","filled")
    scatter3(waypoints(2:end-1,1),waypoints(2:end-1,2),waypoints(2:end-1,3),"y","filled")
    plot3(path.States(:,1),path.States(:,2),path.States(:,3),"r-",LineWidth=2)
    hold off

    save("Results_Map_Path.mat");
    
end

if FOUND==0
    disp('FAILURE = ');
    disp(FAILURE);
end