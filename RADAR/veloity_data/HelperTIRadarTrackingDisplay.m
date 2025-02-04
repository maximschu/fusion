classdef HelperTIRadarTrackingDisplay < matlab.System
    properties (Nontunable)
        XLimits = [0 6];
        YLimits = [-5 5];
        ZLimits = [-0.25 0.25];
        FieldOfView = [120 0];
        MaxRange = 25;
        MotionModel = 'constvel';
        PlotReferenceImage = true;
        RadarReferenceLines = [0 16.5 16.5;0 0 -3.5];
        ReferenceLineLength = [16.5 3.5];
    end

    properties (Access = protected)
        Axes
        RawDetectionPlotter
        ClusteredDetectionPlotter
        TrackPlotter
        ReferenceImagePlotter
    end

    methods
        function obj = HelperTIRadarTrackingDisplay(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj, detections, clusteredDets, tracks, refImage)
          
            f = figure('Visible','on','Units','normalized','Position',[0.1 0.1 0.8 0.8]);
            ax = axes(f);
         
            % Create theater plot
            tp = theaterPlot('Parent',ax,"XLimits",obj.XLimits,'YLimits',obj.YLimits,'ZLimits',obj.ZLimits);

            % Color order
            clrs = lines(7);

            % Create detection plotter
            dp = detectionPlotter(tp,'DisplayName','Point Cloud','MarkerFaceColor',clrs(3,:),'MarkerEdgeColor',clrs(3,:));

            % Create cluster plotter
            dcp = detectionPlotter(tp,'DisplayName','Centroid Estimate','MarkerFaceColor',clrs(2,:),'MarkerEdgeColor',clrs(2,:));

            % Create track plotter
            trkP = trackPlotter(tp,'DisplayName','Tracks','MarkerFaceColor',clrs(1,:),'MarkerEdgeColor',clrs(1,:),'ColorizeHistory','off','ConnectHistory','off','HistoryDepth',30,'FontSize',20);

            % Plot radar coverage
            cp = coveragePlotter(tp,'DisplayName','','Color',[0 0 0],'Alpha',[0.1 0.1]);
            fov = obj.FieldOfView;
            maxR = obj.MaxRange;
            scanLimits = [-1 1;-1 1].*([fov(1)/2;fov(2)/2]);
            cvg = struct('Index',1,'LookAngle',0,'FieldOfView',[120;0],'ScanLimits',scanLimits,'Range',maxR,'Position',[0 0 0],'Orientation',eye(3));
            cp.plotCoverage(cvg);

            % Top view
            view(ax,-90,90);

            % Plot reference linne if image is available
            hold(ax,'on');
            plot(ax,obj.RadarReferenceLines(1,:),obj.RadarReferenceLines(2,:),'LineWidth',2,'Color',[0 0 0],'LineStyle','-.');

            obj.RawDetectionPlotter = dp;
            obj.ClusteredDetectionPlotter = dcp;
            obj.TrackPlotter = trkP;
        end

        function stepImpl(obj, detections, clusteredDets, tracks, refImage)
            % Plot raw detections
            pos = zeros(3,numel(detections));
            vel = zeros(3,numel(detections));
            posCov = zeros(3,3,numel(detections));
            for i = 1:numel(detections)
                [pos(:,i),vel(:,i),posCov(:,:,i)] = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(detections{i},'radar','double');
            end
            obj.RawDetectionPlotter.plotDetection(pos',vel');
            setEdgeAlpha(obj.RawDetectionPlotter);

            % Plot clustered detectionsn
            pos = zeros(3,numel(clusteredDets));
            vel = zeros(3,numel(clusteredDets));
            posCov = zeros(3,3,numel(clusteredDets));
            for i = 1:numel(clusteredDets)
                [pos(:,i),vel(:,i),posCov(:,:,i)] = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(clusteredDets{i},'radar','double');
            end
            obj.ClusteredDetectionPlotter.plotDetection(pos',vel',posCov);
            setEdgeAlpha(obj.ClusteredDetectionPlotter);

            % Plot tracks
            [pos, posCov] = getTrackPositions(tracks,obj.MotionModel);
            vel = getTrackVelocities(tracks,obj.MotionModel);
            if size(pos,2) == 2
                pos = [pos zeros(numel(tracks),1)];
                vel = [vel zeros(numel(tracks),1)];
                posCov3 = zeros(3,3,numel(tracks));
                for i = 1:numel(tracks)
                    posCov3(:,:,i) = blkdiag(posCov(:,:,i),1);
                end
                posCov = posCov3;
            end

            labels = "T" + string([tracks.TrackID]);
            obj.TrackPlotter.plotTrack(pos,vel,posCov,labels);
            setEdgeAlpha(obj.TrackPlotter);

            % Plot reference image
            obj.ReferenceImagePlotter.CData = refImage;
        end
    end
end

function setEdgeAlpha(trkPlotter)
w = warning('off');
s = struct(trkPlotter);
warning(w);
for i = 1:numel(s.CovariancesPatches)
    set(s.CovariancesPatches(i),'EdgeAlpha',1);
    set(s.CovariancesPatches(i),'FaceAlpha',0);
end
end