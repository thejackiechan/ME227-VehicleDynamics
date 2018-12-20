function animateDrift(Ux, Uy, r, delta_rad, veh, dT)
% ANIMATEDRIFT animates drift vehicle simulation data
%
% the visualization can be moved forward or backward in time with the arrow keys or a scrollwheel
% animations can be started and paused by pressing 'p'
% videos can be recorded by pressing 'm' to start and stop

% Matt Brown, Vincent Laurense, John Subosits

    arrow_step = 1;                         % stepsize when arrow keys are pressed
    scroll_step = 30;                       % stepsize when mousewheel is scrolled
    timer_step = 5;                         % stepsize when animation is played

    body_color = [.8 0 .2];                 % color of body
    tire_color = [0 0 0];                   % color of tires

    animation_fps = 50;                     % approximate fps of animation
    animation_filename = 'myMovie.avi';     % movie filename
    delta_mag_factor = 1;                   % show animated steer angle as larger than modeled

    % integrate velocities to E-N  
    [posE_m, posN_m, psi_rad] = getGlobalCoordinates(Uy, r, Ux, dT);

    f = figure('Name', 'Simulation Data', 'IntegerHandle', 'off', 'WindowScrollWheelFcn', @mousescroll_callback, 'WindowKeyPressFcn', @keypress_callback);
    animate_timer = timer('ExecutionMode', 'fixedRate', 'Period', 1/animation_fps);
    set(animate_timer, 'TimerFcn', @timer_callback)
    recording_video = false;
    video_obj = [];

    ax_space = subplot(1,1,1);
    hold on;

    sim.body                = plot(0,0, 'Color', body_color, 'LineWidth', 2.5);
    sim.tire_f              = plot(0,0, 'Color', tire_color, 'LineWidth', 3);
    sim.tire_r              = plot(0,0, 'Color', tire_color, 'LineWidth', 3);
    sim.ref                 = plot(0,0, 'Color', 'k',        'LineWidth', 2);
    sim.traj                = plot(0,0, '--k',      'LineWidth', 2);

    curI = 1;
    maxI = length(Ux);

    xlabel('E [m]')
    ylabel('N [m]')
    axis equal;
    grid on;
    box on;
    %xticks, yticks was introduced in 2016b! use set(gca,'XTick') for backwards compatibility.
    set(gca,'YTick',-1000:5:1000);
    set(gca,'XTick',-1000:5:1000);


    update_axes();

    function keypress_callback(~, event)
        if strcmp(event.Key, 'rightarrow')
            increment_I(arrow_step);
        end
        if strcmp(event.Key, 'leftarrow')
            decrement_I(arrow_step);
        end
        if strcmp(event.Key, 'p')
            if strcmp(animate_timer.Running, 'on')
                stop(animate_timer);
            else
                start(animate_timer);
            end
        end
        if strcmp(event.Key, 'm')
            if recording_video                          % stop recording
                recording_video = false;
                close(video_obj);
                stop(animate_timer);
            elseif strcmp(animate_timer.Running, 'off') % start recording
                recording_video = true;
                video_obj = VideoWriter(animation_filename);
                video_obj.FrameRate = 30;
                video_obj.Quality = 100;
                open(video_obj);
                start(animate_timer);
            end
        end
        update_axes();
    end

    function mousescroll_callback(~,event)
        if event.VerticalScrollCount > 0
            decrement_I(scroll_step);
        else
            increment_I(scroll_step);
        end
        update_axes();
    end

    function timer_callback(~,~)
        if curI == maxI
            stop(animate_timer);
            if recording_video
                recording_video = false;
                close(video_obj);
                return;
            end
        end
        increment_I(timer_step);
        update_axes()
        if recording_video
            cur_frame = getframe(f);
            try % side effect of global recording_video is extra call to timer_callback before recording_video is updated
                writeVideo(video_obj, cur_frame);
            end
        end
    end

    function increment_I(step)
        curI = curI + step;
        if curI > maxI
            curI = maxI;
        end
    end

    function decrement_I(step)
        curI = curI - step;
        if curI < 1
            curI = 1;
        end
    end


    function update_axes()
        
        % Plot the path driven so far
%         plot(posE_m(1:curI), posN_m(1:curI), '--k');
  

        % Plot the vehicle
        update_vehicle(psi_rad(curI), posE_m(curI), posN_m(curI), delta_mag_factor*delta_rad(curI));

        factor = 2;
        xlim(ax_space, [ posE_m(curI)-factor*5 posE_m(curI)+factor*5 ]);
        ylim(ax_space, [ posN_m(curI)-factor*4 posN_m(curI)+factor*4 ]);
        set(f, 'Name', sprintf('Simulation Data, (%d/%d)', curI, maxI));
        drawnow;
    end

    function update_vehicle(psi, posE, posN, delta)
        a = veh.a;
        b = veh.b;
        rW = .34;       % tire radius

        % body
        body_front = [posE - a*sin(psi); posN + a*cos(psi)];
        body_rear = [posE + b*sin(psi); posN - b*cos(psi)];
        body = [body_front body_rear];

        % tires
        tire_f = [body_front(1)-rW*sin(psi+delta) body_front(1)+rW*sin(psi+delta);
                   body_front(2)+rW*cos(psi+delta) body_front(2)-rW*cos(psi+delta)];

        tire_r = [body_rear(1)-rW*sin(psi) body_rear(1)+rW*sin(psi);
                  body_rear(2)+rW*cos(psi) body_rear(2)-rW*cos(psi)];

        % ref point
        ref_rad = .1;
        ang = linspace(0,2*pi,20);
        set(sim.ref, 'Xdata', posE+ref_rad*cos(ang), 'YData', posN+ref_rad*sin(ang));
        set(sim.body, 'XData', body(1,:), 'YData', body(2,:));
        set(sim.tire_f, 'XData', tire_f(1,:), 'YData', tire_f(2,:));
        set(sim.tire_r, 'XData', tire_r(1,:), 'YData', tire_r(2,:));
        set(sim.traj, 'XData', posE_m(1:curI), 'YData', posN_m(1:curI));
        
    end

    function [posE, posN, psi] = getGlobalCoordinates(Uy, r, Ux, tstep)

        N = numel(r);
        posE = zeros(N,1);
        posN = zeros(N,1);
        psi = zeros(N,1);

        for i = 1:N-1

            dEdt = -Uy(i)*cos(psi(i))-Ux(i)*sin(psi(i));
            dNdt = Ux(i)*cos(psi(i)) - Uy(i)*sin(psi(i));
            dPsidt = r(i);

            posE(i+1) = posE(i) + tstep*dEdt;
            posN(i+1) = posN(i) + tstep*dNdt;
            psi(i+1)  = psi(i) + tstep*dPsidt;

        end
        
    end

end
