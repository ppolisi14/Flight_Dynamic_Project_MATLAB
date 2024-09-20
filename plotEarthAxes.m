function [ h_fig ] = plotEarthAxes(h_fig, vXYZ0, vExtent)
%plotEarthAxes Plot Earth axes

%% Plot Earth axes
quiver3( ...
    vXYZ0(1),vXYZ0(2),vXYZ0(3), ...
    vExtent(1),0,0, ...
    'AutoScale', 'off', 'color',[1 0.3 0.3],'linewidth',2.5 ...
); hold on;
quiver3( ...
    vXYZ0(1),vXYZ0(2),vXYZ0(3), ...
    0,vExtent(2),0, ...
    'AutoScale', 'off', 'color',[0.3 1 0.3],'linewidth',2.5 ...
); hold on;
quiver3( ...
    vXYZ0(1),vXYZ0(2),vXYZ0(3), ...
    0,0,vExtent(3), ...
    'AutoScale', 'off', 'color',[0.3 0.3 1],'linewidth',2.5 ...
);

end

