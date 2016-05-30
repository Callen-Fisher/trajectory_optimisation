function animate(trajectory, sim_time, frame_rate, save_video)
%ANIMATE creates an animation from the given trajectory. The video 
%framerate can also be specified, as well as the option to save to the 
%output to an avi file.
%
%   animate(trajectory,frame_rate, save_video)
%
%       trajectory - the trajectory being animated. Each state must occupy
%                    a column (i.e. a trajectory with only one state must
%                    be [n, 1]).
%       frame_rate - the number of frames being displayed per second
%                    [default: 30]
%       sim_time   - the total simulation time. This is used to calculate
%                    the rate at which the animation progresses through the
%                    trajectory
%       save_video - boolean specifying whether the output is saved as an
%                    avi or not [default: false]



%% DISCLAIMER
% I DID NOT WRITE THIS CODE, IT WAS WRITTEN BY NEIL STEENKAMP, UCT, 2016
%% =============================== Set Up =================================

    % User Specified
    fileName = 'results\Animation';

    % Default Values
    if nargin == 1
        frame_rate = 30;
        sim_time = 1;
        save_video = false;
    elseif nargin == 3
        save_video = false;
    end

    % Video File Handling
    if save_video
        % Find New Animation Name
        vid_num = 0;
        numText = sprintf('%.5d',vid_num);  % Zero-pads number
        while(exist([fileName numText '.avi'],'file')==2)
            vid_num = vid_num+1;
            numText = sprintf('%.5d',vid_num);
        end
        vid_name = [fileName numText '.avi'];

        % Create Video Object
        aviobj=VideoWriter(vid_name);
        aviobj.FrameRate = frame_rate;
        open(aviobj);
    end

    %% ========================= Generate Animation ===========================

    frame_len = round(frame_rate*sim_time);
    q_len = size(trajectory,2);
    z1 = trajectory(1,:);
    z2 = trajectory(1,:) - trajectory(2,:);

    COM = (10*z1 + 1*z2)/11;

    % Find Drawing Nodes
    frame_ind = round(linspace(1,q_len,frame_len));

    % disp(anim_dat);
    h  = 0.2;
    w1 = 0.4;
    w2 = 2*w1/3;

%     xmin = min(min(x1,x2)) -max(w1,w2);
%     xmax = max(max(x1,x2)) +max(w1,w2);

    F(frame_len) = struct('cdata', [ ], 'colormap', [ ]);
    for i = 1:frame_len
        k = frame_ind(i);

        % System Model --------------------------------------------------------
        r = max(z1(k),z2(k))-min(z1(k),z2(k));
        clf;
        rectangle('Position',[-h/2,(z1(k)-w1/2), h, w1]);
        hold on;
        rectangle('Position',[-h/2,(z2(k)-w2/2), h, w2]);
        plot(0,z1(k),'ok','MarkerSize',4,'MarkerFaceColor','k');
        plot(0,z2(k),'ok','MarkerSize',4,'MarkerFaceColor','k');
        plot(0,COM(k),'ok','MarkerSize',4);

        nspring = 6;
        s1 = z1(k) -(1*r/3);
        s2 = z2(k) +(1*r/3);
        sp = linspace(s1,s2,nspring+2);
        sp = sp(2:end-1);
        plot([h/2,h/2,reshape(0.1*[ones(1,nspring/2);-ones(1,nspring/2)]+h/2,1,[]),h/2,h/2]-h/2,[z1(k),s1,sp,s2,z2(k)],'b-');
        hold off;

        % Frame Settings ------------------------------------------------------
        xmin = min(min(z1(k),z2(k))) -2*max(w1,w2);
        xmax = max(max(z1(k),z2(k))) +2*max(w1,w2);    
    %     xlabel('x distance (m)','FontSize',15,'FontWeight','bold') % x-axis label
    %     ylabel('height (m)','FontSize',15,'FontWeight','bold') % x-axis labely
        axis equal    
        axis([-h, h, COM(k)-2, COM(k)+2])
%         axis([-h h xmin xmax])
    %     axis([xmin xmax 0 2*h])

        % Video Settings ------------------------------------------------------

        F(k) = getframe(gcf);
    %     cdata = print('-RGBImage');
        if(save_video)
             writeVideo(aviobj, F(k));
    %        writeVideo(aviobj, cdata);
        end
    %     drawnow;
    %     pause(0.01);
    end

    if(save_video)
        close(aviobj);
    end

end

%         img = hardcopy(hFig, '-dzbuffer', '-r0');
%         writeVideo(aviobj, im2frame(img));
%         writeVideo(aviobj,F);
%     xlim([anim_datx(1,k)-xymax, anim_datx(1,k)+xymax]);
%     ylim([anim_datx(2,k)-xymax, anim_datx(2,k)+xymax]);
