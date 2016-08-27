classdef CATBot < handle
    properties
        true_state;
        belief_state;
        map;
        sensor;
        particles;
        target_state;
        dt = 0.01;
        alphas = [0.01;0.01;0.01;0.01;0.01;0.01];
        z_params = [0.85; 0.03; 0.1; 0.02];
        sigma_hit = 0.05;
        lamda_short = 1;
        CATBot_plot; p; l;
        particles_plot; particles_p=[];
        rays_plot_handle;
        rays_plot_hggroup;
        M = 500; % Number of particles
    end
    
    methods
        function [CATBot] = spawn_CATBot(CATBot,map,belief_axes,true_axes)
            %-------- Spawning function --------%
            %{
                This function spawns a CATBot robot into the map
            %}
            CATBot.map          = map;
            CATBot.true_state   = [2+45*rand;2+45*rand;2*pi*rand];
            CATBot.particles    = [CATBot.true_state(1) + 0.1 * CATBot.true_state(1)*randn(1,CATBot.M); ...
                CATBot.true_state(2) + 0.1 * CATBot.true_state(2)*randn(1,CATBot.M); ...
                CATBot.true_state(3) + 0.1 * CATBot.true_state(3)*randn(1,CATBot.M)];
            CATBot.belief_state = mean(CATBot.particles,2);
            CATBot.plot_CATBot(true_axes);
            CATBot.plot_Particles(belief_axes);
        end
        
        function [CATBot] = get_sensor_data(CATBot)
            %-------- Sensing function --------%
            %{
            This function does two main roles 
                1) Simulates laser range finder sensor readings 
                2) Performs  ray casting algorithm
            
            The function performs the ray casting algorithm to calculate
            the "correct" range as indicated by the current state belief
            and the map according to the sensors measurement field. Then
            simulates sensor readings by performing the ray casing
            algorithm with the true state and the map then adds zero-mean
            normally distributed noise to resemble as close as possible
            real-world data
            %}
            range      = [];
            true_range = [];
            x      = CATBot.belief_state(1);     y      = CATBot.belief_state(2);      theta      = CATBot.belief_state(3);
            x_true = CATBot.true_state(1);       y_true = CATBot.true_state(2);        theta_true = CATBot.true_state(3);
            for c= -90:0.5:90
                range_k         = [];
                true_range_k    = [];
                rays            = [];
                lth_beam_x      = [x     , x + 10*cos(theta+c*pi/180)];
                lth_beam_y      = [y     , y + 10*sin(theta+c*pi/180)];
                lth_beam_x_true = [x_true, x_true + 10*cos(theta_true+c*pi/180)];
                lth_beam_y_true = [y_true, y_true + 10*cos(theta_true+c*pi/180)];
                for k=1:length(CATBot.map)
                    [x_intersect      ,      y_intersect] = polyxpoly(CATBot.map(k).XData,CATBot.map(k).YData,lth_beam_x,lth_beam_y);
                    [x_intersect_true , y_intersect_true] = polyxpoly(CATBot.map(k).XData,CATBot.map(k).YData,lth_beam_x_true,lth_beam_y_true);
                    if (~isempty(x_intersect) && ~isempty(y_intersect))
                        val   = sqrt((x_intersect - x)'*(x_intersect - x) + (y_intersect - y)'*(y_intersect - y));
                        range_k = [range_k min(val)];
                    else
                        range_k = [range_k 10];
                    end
                    
                    if (~isempty(x_intersect_true) && ~isempty(y_intersect_true))
                        val_true     = sqrt((x_intersect_true - x_true)'*(x_intersect_true - x_true) + (y_intersect_true - y_true)'*(y_intersect_true - y_true));
                        true_range_k = [true_range_k val_true];
                    else
                        true_range_k = [true_range_k 10];
                    end
                end
                range      = [range min(range_k)];
                true_range = [true_range min(true_range_k)];
            end
            CATBot.sensor.readings      = true_range + random('Normal',0,0.05,[1 361]);
            CATBot.sensor.angle         = [-90:0.5:90]*c*pi/180;
            CATBot.sensor.ray_casting   = range;
        end 
        
        function [q] = beam_range_finder_model(CATBot)
            %-------- Measurement model function --------%
            %{
            This function calculates the weight of measurement according to
            the beam-range-finder sensor model.
            %}  
            q = 1;
            for k = 1:length(CATBot.sensor.readings)
                z_t     = CATBot.sensor.readings(k);
                z_t_ast = CATBot.sensor.ray_casting(k);
                p_hit   = cdf('Normal',z_t,z_t_ast,CATBot.sigma_hit)/cdf('Normal',10,z_t_ast,CATBot.sigma_hit);
                p_short = cdf('Exponential',z_t,CATBot.lamda_short)/(1- exp(-CATBot.lamda_short * z_t_ast));
                p_max   = z_t==10;
                p_rand  = 0.1 *z_t_ast * (z_t > 0 & z_t < 10);
                
                p_tot   = CATBot.z_params' *[p_hit;p_short;p_max;p_rand];    
                q       = q * p_tot;
            end
        end
        
        function [curr_state] = sample_velocity_motion_model(CATBot,prev_state,u)
            %-------- Motion model function --------%
            %{
                This function does the motion samping procedure according
                to the vellocity motion model. It's used to generate
                hypotheses about the state of the system (generate
                predicted particles set from the previous patrticle set)
            %}
            v_bar = sample(CATBot.alphas(1)*abs(u(1))+CATBot.alphas(2)*abs(u(2))) + u(1);
            w_bar = sample(CATBot.alphas(3)*abs(u(1))+CATBot.alphas(4)*abs(u(2))) + u(2);
            y_bar = sample(CATBot.alphas(5)*abs(u(1))+CATBot.alphas(6)*abs(u(2)));

            curr_state(1) = prev_state(1) + v_bar/w_bar * ( sin(prev_state(3)+ w_bar * CATBot.dt) - sin(prev_state(3)));
            curr_state(2) = prev_state(2) + v_bar/w_bar * ( -cos(prev_state(3)+ w_bar * CATBot.dt) + cos(prev_state(3)));
            curr_state(3) = prev_state(3) + w_bar * CATBot.dt + y_bar * CATBot.dt;
        end
        
        function [CATBot] = move_CATBot(CATBot,u)
            %-------- Sensing function --------%
            %{
                This function applies the input signal to the CATBot to
                move it in space, the functino assumes a normally
                distributed noise on the input which has a standard
                deviation proportional to the input signal
            %}
            CATBot.true_state(3) = CATBot.true_state(3) + 0.1 * abs(u(2)) * randn * u(2) * CATBot.dt;
            CATBot.true_state(2) = CATBot.true_state(2) + 0.1 * abs(u(1)) * randn * u(1) * CATBot.dt * sin(CATBot.true_state(3));
            CATBot.true_state(1) = CATBot.true_state(1) + 0.1 * abs(u(1)) * randn * u(1) * CATBot.dt * cos(CATBot.true_state(3));
            if (CATBot.true_state(3) >= pi)
                CATBot.true_state(3) = CATBot.true_state(3) - pi;
            end
        end
        
        function [CATBot] = monte_carlo_localization(CATBot,u)
            %-------- Monte Carlo Localization function --------%
            %{
                This function performs the Monte Carlo localization method
                using the Particle Filter. This implementation is the most
                basic implementation of the method (not adaptive)
            %}
            temp_particle_set.state  = [];
            temp_particle_set.weight = [];
            for m = 1:CATBot.M
                particle = sample_velocity_motion_model(CATBot,CATBot.particles(:,m),u);
                weight   = beam_range_finder_model(CATBot);
                temp_particle_set.state  = [temp_particle_set.state , particle];
                temp_particle_set.weight = [temp_particle_set.weight, weight];
            end
            temp_particle_set.weight = temp_particle_set.weight / sum(temp_particle_set.weight);
            CATBot.particles = low_variance_sampler(temp_particle_set.state,temp_particle_set.weight);
        end
        
        function [CATBot] = plot_CATBot(CATBot,true_axes)
            %-------- CATBot plotting function --------%
            %{
                This function plots the CATBot on the axes specified by the
                handle {true_axes}
            %}
            x = CATBot.true_state(1);   y = CATBot.true_state(2);   theta = CATBot.true_state(3);
            if ~isempty(CATBot.CATBot_plot), delete(CATBot.CATBot_plot); end
            CATBot.CATBot_plot = hggroup(true_axes);
            CATBot.p = plot(CATBot.CATBot_plot,x,y,'ko','MarkerSize',20);
            CATBot.l = line(CATBot.CATBot_plot,[x x + 1.2*cos(theta)],[y y+1.2*sin(theta)],'LineWidth',3,'Color',[0 0 1]);
            drawnow
        end
        
        function [CATBot] = plot_Particles(CATBot,belief_axes)
            %-------- Particles plotting function --------%
            %{
                This function plots the particles set representing the
                hypotheses about the state on the axes specified by the
                handle {belief_axes}
            %}
            if ~isempty(CATBot.particles_plot), delete(CATBot.particles_plot); end
            CATBot.particles_plot = hggroup(belief_axes);
            CATBot.particles_p = [];
            for k = 1:length(CATBot.particles)
                x = CATBot.particles(1,k);  y = CATBot.particles(2,k);
                CATBot.particles_p = [CATBot.particles_p plot(CATBot.particles_plot,x,y,'k*','MarkerSize',2)];
            end
            drawnow
        end
        
        function [CATBot] = plot_rays(CATBot, true_axes)
            CATBot.rays_plot_hggroup = hggroup(true_axes);
            x = CATBot.true_state(1);   y = CATBot.true_state(3);
            for k = 1:36:361
                CATBot.rays_plot_handle = [CATBot.rays_plot_handle, line(CATBot.rays_plot_hggroup,[x CATBot.sensor.rays(1,k)],[y CATBot.sensor.rays(2,k)])];
            end
            drawnow
        end
        
        function [CATBot] = set_target(CATBot,goal)
            
        end

    end
    
end

function [out] = sample(in)
    out = randn * sqrt(in);
end

function [resampled_particles] = low_variance_sampler(particles,weights)
    resampled_particles = [];
    M = length(particles);
    r = rand/M;
    c = weights(1);
    i = 1;
    for m = 1:M
       u = r + (m-1)/M;
       while (u > c)
          i = i + 1;
          c = c + weights(i);
       end
       resampled_particles = [resampled_particles particles(:,i)];
    end
end