classdef VRPN_ROS < matlab.mixin.SetGet
    % This class creates a matlab object that uses the MATLAB ROS toolbox
    % to interface with a running OptiTrack system that has detected
    % objects in the workspace.
    %
    %   H. Helmich, 30Aug2022, USNA
    
    properties(GetAccess = 'public', SetAccess = 'public')
        pSub
        pose(4,4)       double {mustBeReal, mustBeFinite}
        quaternion(4,1) double {mustBeReal, mustBeFinite}
        position(3,1)   double {mustBeReal, mustBeFinite}
        timeStamp(2,1)  double {mustBeReal, mustBeFinite}
        frameID
    end
    
    % Initialization methods:
    %   Constructor
    %   Destructor
    methods(Access='public')
        function obj = VRPN_ROS(varargin)
            % Initialize a VRPN_ROS object
            %   obj = VRPN_ROS
            %   obj = VRPN_ROS(rigidBody)
            %
            %   Input(s)
            %       rigidBody - character array specifying rigid body name.
            %                   If no name is specified, user is prompted
            %                   to select a rigid body name from the list
            %                   of available rigid bodies.
            %
            %   Output(s)
            %       obj - VRPN_ROS object for specified rigid body
            
            % Get available rigid bodies
            list = vrpn_listClients;
    
            % Find user specified rigid body in list
            if nargin > 0
                idx = find( lower(list) == lower(varargin{1}) );
                if isempty(idx)
                    warning('Specified rigid body is not visible.');
                end
            else
                idx = [];
            end

            if isempty(idx)
                idx = listdlg('ListString', list);
            end

            topic = char(list(idx)); % change to char
            topic = append('/vrpn_client_node/', topic, '/pose');
            obj.pSub = rossubscriber(topic, 'geometry_msgs/PoseStamped');
            
            drawnow

            % Get initial message and parse data
            msg = receive(obj.pSub);
            obj.pose = obj.rosPoseMsg2RBT(msg);
            obj.timeStamp(1,1) = msg.Header.Stamp.Sec;
            obj.timeStamp(2,1) = msg.Header.Stamp.Nsec;
            obj.frameID = msg.Header.FrameId;
        end
        
        function delete(obj)
            %clear obj.pSub
            clear obj.pSub
            % TODO - check if subscriber needs to be deleted

            drawnow

            delete(obj);
        end
        
    end
    
    % Object methods
    %   rosPointMsg2Array
    %   rosOrientMsg2SO
    %   rosPoseMsg2RBT
    methods(Access='private')
        function out = rosPointMsg2Array(obj, msg)
            % ROSPOINTMSG2ARRAY converts a ROS Point message to an array
            %   out = rosPointMsg2Array(msg)
            %
            %   Input(s)
            %       msg - ROS Point message
            %
            %   Output(s)
            %       out - 3-element array containing x/y/z data
            %
            %   M. Kutzer & H. Helmich (reluctantly), 30Aug2022, USNA
            
            % Check inputs
            narginchk(1,2)
            
            goodClass = 'ros.msggen.geometry_msgs.Point';
            switch lower( class(msg) )
                case lower( goodClass )
                    % Good message
                otherwise
                    badClass = class(msg);
                    error('Input message is class "%s", expected class "%s".',...
                        badClass,goodClass);
            end
            % Parse data
            out(1,1) = msg.X;
            out(2,1) = msg.Y;
            out(3,1) = msg.Z;
        end
        
        function quat = rosOrientMsg2SO(obj, msg)
            % ROSORIENTMSG2SO converts a ROS Orient message to an array
            %   out = rosOrientMsg2SO(msg)
            %
            %   Input(s)
            %       msg - ROS Quaternion message
            %
            %   Output(s)
            %       out - 3x3 -element array containing rotation (xyzw) data
            %
            %   M. Kutzer & H. Helmich (reluctantly), 30Aug2022, USNA
            
            % Check inputs
            narginchk(1,2)
            
            goodClass = 'ros.msg.geometry_msgs.Quaternion';
            switch lower( class(msg) )
                case lower( goodClass )
                    % Good message
                otherwise
                    badClass = class(msg);
                    error('Input message is class "%s", expected class "%s".',...
                        badClass, goodClass);
            end
            % Parse data
            X = msg.X;
            Y = msg.Y;
            Z = msg.Z;
            W = msg.W;
            
            %out = quat2rotm([X Y Z W]); % <--- MISTAKE (see doc quat2rotm)
            quat = [W; X; Y; Z];
        end
        
        function out = rosPoseMsg2RBT(obj, msg)
            % ROSPOSEMSG2RBT converts a ROS Pose message to 4x4 Rigid Body Transform
            %   out = rosPoseMsg2RBT(msg)
            %
            %   Input(s)
            %       msg - ROS Pose message
            %
            %   Output(s)
            %       out - 4x4 element array containing position & orientation data
            %
            %   M. Kutzer & H. Helmich, 30Aug2022, USNA
            
            % Check inputs
            narginchk(1,2)
            
            goodClass = 'ros.msggen.geometry_msgs.PoseStamped';
            switch lower( class(msg) )
                case lower( goodClass )
                    % Good message
                otherwise
                    badClass = class(msg);
                    error('Input message is class "%s", expected class "%s".',...
                        badClass, goodClass);
            end
            % Parse data
            out = eye(4);
            out(4,:) = [0 0 0 1];
            quat = obj.rosOrientMsg2SO(msg.Pose.Orientation);
            out(1:3, 1:3) = quat2rotm(quat');
            out(1:3, 4) = obj.rosPointMsg2Array(msg.Pose.Position);
        end

    end
    
    % Query methods
    %   get.pose
    %   get.timeStamp
    %   get.frameID
    methods
        function pose = get.pose(obj)
            % Direct query to update pose data
            tmp = receive(obj.pSub);
            pose = obj.rosPoseMsg2RBT(tmp);
        end

        function quaternion = get.quaternion(obj)
            % Direct query to update pose data
            tmp = receive(obj.pSub);
            quaternion = obj.rosOrientMsg2SO(tmp.Pose.Orientation);
        end

        function position = get.position(obj)
            % Direct query to update pose data
            tmp = receive(obj.pSub);
            position = obj.rosPointMsg2Array(tmp.Pose.Position);
        end

        function timeStamp = get.timeStamp(obj)
            % Direct query to update timeStamp data
            tmp = receive(obj.pSub);
            timeStamp(1,1) = tmp.Header.Stamp.Sec;
            timeStamp(2,1) = tmp.Header.Stamp.Nsec;
        end
        
        function frameID = get.frameID(obj)
            % Direct query to update frameID data
            tmp = receive(obj.pSub);
            frameID = tmp.Header.FrameId;
        end
        
    end
    
end