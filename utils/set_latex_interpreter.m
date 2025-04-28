function [] = set_latex_interpreter()
% Set latex interpreter for figure export
%   Set latex interpreter for figure axes, legend and text 
%   No inputs needed 

set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

end