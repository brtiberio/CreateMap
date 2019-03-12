% Copyright (c) 2019 J. Sequeira
%                    Bruno Tiberio
%                    F. Ferreira da Silva
%
% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files (the
% "Software"), to deal in the Software without restriction, including
% without limitation the rights to use, copy, modify, merge, publish,
% distribute, sublicense, and/or sell copies of the Software, and to permit
% persons to whom the Software is furnished to do so, subject to the
% following conditions:
%
% The above copyright notice and this permission notice shall be included
% in all copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
% MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
% NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
% DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
% OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
% USE OR OTHER DEALINGS IN THE SOFTWARE.
%
%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef CreateMap < handle
    %CREATEMAP Create or load map points
    %   Create map waypoints using user input or load from gpx file.
    %   Waypoints will be interpolated based on 'makima' method by default
    
    properties
        %------------------------------------------------------------------
        % Constants section
        %------------------------------------------------------------------
        
        
        % sampling spacing between waypoints
        sampling_spacer = 0.01;
        % interpolated map points, resulting in reference path
        map_points = [];
        % waypoints used to create interpolated reference path
        waypoints = [];
        % method used in interpolation
        interp_method;
        % number of point in interpolated path
        num_points;
        % if necessary to create a car
        car_position;
        % path curvature
        path_curvature;
    end
    properties(SetAccess = private)
        %------------------------------------------------------------------
        % figure handles for rapid plotting and updating
        %------------------------------------------------------------------
        
        % handler for figure
        figure_handle;
        % handler for legend
        legend_handle;
        % handler for plotting waypoints and interpolated path
        plot_handle;
        % handler for creating car starting position
        car_position_handle;
        % get default plot colors. 
        co = get(groot,'defaultAxesColorOrder');
    end
    
    methods
        function obj = CreateMap(varargin)
            % just in case of using old matlab versions
            % update to the new color order which is more visual appealing
            if any(~( obj.co(1,:) == [ 0    0.4470    0.7410]))
                obj.co = [0    0.4470    0.7410;
                    0.8500    0.3250    0.0980;
                    0.9290    0.6940    0.1250;
                    0.4940    0.1840    0.5560;
                    0.4660    0.6740    0.1880;
                    0.3010    0.7450    0.9330;
                    0.6350    0.0780    0.1840];
            end
            %--------------------------------------------------------------
            % check if arguments were supplied or select default
            %--------------------------------------------------------------
            valid_methods ={'linear', 'nearest', 'next', 'previous',...
                'pchip', 'cubic', 'v5cubic', 'makima', 'spline'};
            
            args = inputParser();
            args.addParameter('method','makima',...
                @(x)~isempty(validatestring(x,valid_methods)));
            args.addParameter('waypoints', [],...
                @(x)validateattributes(x,{'numeric'},...
                {'real'}));
            args.parse(varargin{:});
            obj.interp_method = args.Results.method;
            obj.waypoints = args.Results.waypoints;
            
            %--------------------------------------------------------------
            % check if figure is still active or not
            %--------------------------------------------------------------
            if isempty(obj.figure_handle) || ~isvalid(obj.figure_handle)
                % create a new one
                obj.figure_handle = figure('Name', 'Create Map');
                obj.figure_handle.Visible = 'off';
                sizes=get(groot, 'ScreenSize');
                obj.figure_handle.Position = [1 1 sizes(3)*0.8 sizes(4)*0.8];
                movegui(obj.figure_handle, 'center');
            else
                % bring to front and clear it
                figure(obj.figure_handle);
                clf(obj.figure_handle);
            end
            %--------------------------------------------------------------
            % check if waypoints were supplied
            %--------------------------------------------------------------
            if(~isempty(obj.waypoints))
                obj.plot_handle = plot(obj.waypoints(:,1), obj.waypoints(:,2), '+');
                obj.plot_handle.Parent.NextPlot = 'add';
                obj.interpolate_path()
                daspect([1 1 1]);
                obj.figure_handle.Visible = 'on';
                ax_limits = [min(obj.waypoints(:,1))-50, max(obj.waypoints(:,1))+50, min(obj.waypoints(:,2))-50, max(obj.waypoints(:,2))+50];
                axis(ax_limits);
            end
        end
        
        function interpolate_path(obj)
            num_wapoints = length(obj.waypoints);
            circuit_index = 0:obj.sampling_spacer:num_wapoints-1;
            xx = interp1(0:num_wapoints-1, obj.waypoints(:,1), circuit_index, obj.interp_method);
            yy = interp1(0:num_wapoints-1, obj.waypoints(:,2), circuit_index, obj.interp_method);
            % revert to normal red line plot without markers
            obj.plot_handle.XData = xx;
            obj.plot_handle.YData = yy;
            obj.plot_handle.Marker = 'none';
            obj.plot_handle.LineStyle = '-';
            % axis auto;
            
            plot(obj.waypoints(1,1),obj.waypoints(1,2), 'Marker','s', 'MarkerFaceColor',[0.9290    0.6940    0.1250]);
            plot(obj.waypoints(end,1), obj.waypoints(end,2), 'Marker','s', 'MarkerFaceColor',[0.4660    0.6740    0.1880] );
            obj.legend_handle=legend('Ref', 'Start', 'Stop', 'location','best');
            obj.map_points= [xx' yy'];
            obj.num_points= length(obj.map_points);
            drawnow;
            % after interpolation, estimate curvature of path.
            obj.find_curvature();
        end
        
        function create_waypoints(obj)
            %--------------------------------------------------------------
            % define the reference trajectory using mouse
            %--------------------------------------------------------------
            % "real" trajectories are hard to project without axis equal.
            obj.figure_handle.Visible = 'on';
            clf(obj.figure_handle);
            ax_handle = newplot(obj.figure_handle);
            % axis square;
            daspect([1 1 1]);
            axis([-200,200,-200,200]);
            % necessary to not reset axis config
            ax_handle.NextPlot = 'add';
            ax_handle.ColorOrderIndex=2;
            disp('use the mouse to input via points for the reference trajectory');
            disp('Last point using right click');
            button = 1;
            % number of points used
            point_index = 1;
            while button==1
                [x(point_index),y(point_index),button] = ginput(1);
                if point_index == 1
                    obj.plot_handle = plot(ax_handle, x, y, '+');
                else
                    obj.plot_handle.XData = x;
                    obj.plot_handle.YData = y;
                    drawnow;
                end
                point_index = point_index + 1;
            end
            obj.waypoints = [x' y'];
        end
        function [x, y, psi] = create_car(obj)
            % bring figure to front
            figure(obj.figure_handle);
            % initial pose of the vehicle given by mouse
            disp('input the initial position for the car')
            [x,y] = ginput(1);
            obj.car_position =[x, y];
            obj.car_position_handle = plot(x, y, 'o', 'Color', [0    0.4470    0.7410], 'MarkerFaceColor',[0    0.4470    0.7410]);
            drawnow;
            % create direction vector
            [x2,y2] = ginput(1);
            directionVector = [x2-x,y2-y];
            psi = atan2(directionVector(2),directionVector(1));
        end
        function [map_points] = get.map_points(obj)
            map_points = obj.map_points;
        end
        function load_gpx(obj, filename)
            % if not defined, select file input
            if ~exist('filename','var')
                [filename, pathname]= uigetfile('.gpx');
            end
            % hitted cancel or closed?
            if filename == 0
                fprintf('No file selected... Exiting now\n');
                return
            else
                if exist('pathname','var')
                    fullfile_name = fullfile(pathname,filename);
                else
                    fullfile_name = filename;
                end
            end
            % load gpx file
            waypoints = gpxread(fullfile_name);
            % set reference point local ENU frame
            [xEast,yNorth,zUp]= geodetic2enu(waypoints.Latitude, waypoints.Longitude,waypoints.Elevation,...
                waypoints.Latitude(1), waypoints.Longitude(1), waypoints.Elevation(1),wgs84Ellipsoid);
            obj.waypoints = [xEast', yNorth'];
            obj.plot_handle = plot(obj.waypoints(:,1), obj.waypoints(:,2), '+', 'Color',obj.co(2,:));
            obj.plot_handle.Parent.NextPlot = 'add';
            daspect([1 1 1]);
            obj.figure_handle.Visible = 'on';
            ax_limits = [min(obj.waypoints(:,1))-50, max(obj.waypoints(:,1))+50, min(obj.waypoints(:,2))-50, max(obj.waypoints(:,2))+50];
            axis(ax_limits);
        end
    end
    methods(Access = private)
        function find_curvature(obj)
            
            %--------------------------------------------------------------
            % For reference and implementation see:
            % 
            % https://www.mathworks.com/matlabcentral/answers/58964-curvature-of-a-discrete-function
            
            x = obj.map_points(:,1);
            y = obj.map_points(:,2);
            dx = gradient(x);
            dy = gradient(y);
            ddx = gradient(dx);
            ddy = gradient(dy);
            num   = abs(dx .* ddy - ddx .* dy);
            denom = dx .* dx + dy .* dy;
            denom = sqrt(denom);
            denom = denom .* denom .* denom;
            curvature = num ./ denom;
            curvature(denom < 0) = NaN;
            obj.path_curvature = curvature;
        end
    end
end

