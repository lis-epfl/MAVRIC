function extract_data_from_sd(filename)
% DATA_EXTRACT Extract data from on-board log files from an SD card
%   @param (required) filename : The log file's name (.txt format)
%
%   @author : MAV'RIC Team
%   @author : Dylan Bourgeois
%
% Please follow the 2 steps to use this script accordingly.
%   1/2 : Declare the variables you wish to log if they are not defined in
%         the list. 
%         /!\ You must use the same name as appears in the log file. /!\
%
%   2/2 : Define which variables you would like to plot. To do so you can
%         use one of the built-in functions (or implement your own) or use
%         the generic 'plot_params(varargin)' function. For details see
%         the function definition

% Read logfile
hFile = fopen(filename, 'rt');
try text = textscan(hFile,'%s');
catch err
    error('Couldn''t open file : %s please check the specified path.', filename)
end

% Clean up 'NaN' from log file
numCols = getNumCols(filename);
lastNaNIndex = findLastNaNIndex(text, numCols);

% Read data starting after the last NaN's index
try data = dlmread(filename,'\t',lastNaNIndex,0);
catch err
    error('Couldn''t read data from the file. Please check the log format.')
end

% Count the number of logged variables
numLoggedVars = rows(data);
loggedVars = text{1:1}(1:numLoggedVars);


% ****************************** Step 1/2 ****************************** %
% Indices -- Store all the base indices in map
% Reminder : You must use the same name as is used in the log file
% otherwise the variable will not be recognized.
loggableVars = {            ...                                
                    'time', ...
                    'acc_x', ...
                    'acc_y', ...
                    'acc_z', ...
                    'Pos_X', ...
                    'Pos_Y', ...
                    'Pos_Z', ...
                    'Ori_Lat', ...
                    'Ori_Lon', ...
                    'Ori_Alt', ...
                    'mav_state', ...
                    'mav_mode', ...
                    'mode_custom', ...
                    'num_neighbors', ...
                    'Comf_Slider', ...
                    'CurrentWp', ...
                    'min_coll_dist', ...
                    'alt_consensus', ...
                    'mean_comm_freq', ...
                    'var_comm_freq', ...
                    % ' YOUR_VARIABLE_HERE ', ...
               };
% ****************************** Step 1/2 ****************************** %			

% Reset indices to zero        																			
idx = zeros(1,numel(loggableVars));
indices = containers.Map(loggableVars, idx);

% Update indices -- Match present values with coresponding index to update
% it to value found in log file
for i = 1:size(loggedVars)
    varname = loggedVars(i);
    if isKey(indices,varname)
        indices(char(varname)) = i;
    end
end

% Define time vector
timekeeper = data(1:rows(data), indices('time'));

% ****************************** Step 2/2 ****************************** %
% Determine which variables to plot

% Generic plot -- Add the names of variables you would like to plot, 
% grouped by figure
 plotParams('acc_x','acc_y','acc_z')

% Pre-existing plots -- Feel free to implement your own and make them
% available just below this step

% plotAcc()
% plotHeading()

% ****************************** Step 2/2 ****************************** %

	% Generic plot function : Takes the name of parameters available in the
	% log and plots them together (ie on the same figure)
    function plotParams(varargin)
        % Data
        plotData = [];
       for i = 1:numel(varargin)
           if indices(varargin{i}) == 0
               errstring = ['Trying to plot a variable ( %s ) that does' ...
                           ' not appear in the log'];
               error(errstring, varargin{i});
           end
           plotData = [plotData data(1:rows(data),indices(varargin{i}))];
       end
       
       % Plots
       colors = ['r', 'g', 'b', 'k', 'y', 'm'];
       
       legends = [];
       for i = 1:numel(varargin)
           legends = [legends, cellstr(varargin{i})];
       end
       
       figure
       hold on
       
       for i = 1:numel(varargin)
           plot(timekeeper, plotData(:,i),colors(i))
       end
       
       legend(legends)
       xlabel('Time (ms)')
       
       grid on
        
       hold off
       
    end

% ****************************** Plots ****************************** %
    % Custom plot functions -- You can implement your own plot     %
    % functions here.                                              %
% ****************************** Plots ****************************** %

    % Plots the acceleration in X,Y and Z
    function plotAcc()
        plotParams('acc_x','acc_y','acc_z')
    end

    % Plots the heading from the attitude logs
    function plotHeading()
        % Data
        plotData = data(1:rows(data), indices('yaw')) ./ (pi*180);
        
        % Plot
        figure
        plot(timekeeper, plotData, 'r')
        legend('yaw')
        xlabel('Time (ms)')
        ylabel('Heading (rad)')
        grid on
    end


% Utilities

    % Find number of columns in a given file
    function numCols = getNumCols(filename)
        delimiter = char(9); % '\t'
        fid = fopen(filename, 'rt');
        tLines = fgets(fid);
        numCols = numel(strfind(tLines,delimiter)) + 1;
    end

    function lastNaNIndex = findLastNaNIndex(text, numCols)
        lastNaNIndex=1;
        for i = 1:size(text{1})
            if strcmp(text{1}(i), 'NaN')
                lastNaNIndex = i;
            end
        end
        lastNaNIndex = floor(lastNaNIndex./numCols) * 2 + 1;
    end
    
    % Returns the number of rows in a matrix
    function rows = rows(x) 
        rows = size(x,1); 
    end

    % Returns the number of columns in a matrix
    function cols = cols(x) 
            cols = size(x,2);
    end
end