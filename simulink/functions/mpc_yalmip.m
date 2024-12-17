classdef mpc_yalmip < matlab.System
    % A linear MPC controller implemented using YALMIP

    % Public, tunable properties 
    % (can be set through the mask, default values are given here)
    properties
        Ts = 1e-3   % sampling time [s]
        nu = 4      % number of control inputs
        N = 10      % controller prediction horizon
    end
    
    % Private properties (invisible from the outside)
    properties(Access = private)
        controller  % the MPC object created by YALMIP
    end
    
    % Different functions that the block implements
    methods(Access = protected)

        % One-time setup of the block
        function setupImpl(obj)
            % create the controller
            obj.controller = mpc_setup(obj.N);
        end
        
        % Loop function, runs in every simulation step
        function [u_opt,exitflag,solvetime] = stepImpl(obj,x_current,x_ref)
            % perform an MPC step
            [u_opt,exitflag,~,~,~,info] = obj.controller({x_current, x_ref});
            solvetime = info.solvertime;
        end
        
        % Sets the sample time of the Simulink block
        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,'Type','Discrete',...
                                       'SampleTime',obj.Ts,...
                                       'OffsetTime',0);
        end

        % Define the type of data going out of the block
        function [dt1,dt2,dt3] = getOutputDataTypeImpl(~)
        	dt1 = 'double';
            dt2 = 'double';
            dt3 = 'double';
        end
        
        % Define the size of data going out of the block
        function [sz1,sz2,sz3] = getOutputSizeImpl(obj)
            sz1 = [obj.nu,1];           % optimal input
            sz2 = [1,1];                % exitflag
            sz3 = [1,1];                % solve time
        end
        
        % Define the complexity of the data
        function [cp1,cp2,cp3] = isOutputComplexImpl(~)
        	cp1 = false;
            cp2 = false;
            cp3 = false;
        end

        % Define the variability of the data size
        function [fz1,fz2,fz3] = isOutputFixedSizeImpl(~)
        	fz1 = true;
            fz2 = true;
            fz3 = true;
        end
    end
end
