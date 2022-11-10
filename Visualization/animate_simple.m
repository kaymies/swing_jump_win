function animate_simple(t,z,p, speed)

    axis([-.2 .2 -.2 1])
    h_ground = plot([-1 1],[0 0],'k-','LineWidth',5);
    hold on
    h_leg    = plot([0],[0],'-b',...
                'LineWidth',2,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',4); 
    h_spring = plot([0],[0],'-o',...
                'LineWidth',2,...
                'MarkerEdgeColor','r',...
                'MarkerFaceColor','r',...
                'MarkerSize',4);

    tic                                             % start counter
    while toc < t(end)/speed                        % while there's real time left
        tsim = toc*speed;                           % determine the simulation time to draw
        zint = interp1(t',z',tsim', 'linear')';     % interpolate to get coordinates at that time
        draw_lines(zint,p,h_leg,h_spring);
    end
    draw_lines(z(:,end),p,h_leg,h_spring);
end

<<<<<<< Updated upstream
function draw_lines(z,p, h_leg)
    keypoints = keypoints_swing_jump_win(z,p);
=======
function draw_lines(z,p, h_leg,h_spring)
    keypoints = keypoints_swing_jump_win(z,p);
    h_spring.XData = keypoints(1,2:3);
    h_spring.YData = keypoints(2,2:3);
>>>>>>> Stashed changes
    h_leg.XData = keypoints(1,:);
    h_leg.YData = keypoints(2,:);
    drawnow
    axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched
%     axis([-.2 .2 -.1 0.6])
    %UPDATED - EK 
    %11 Nov 2022 - Increase y axis max value
    axis([-.2 .2 -.1 1.0]) 
end