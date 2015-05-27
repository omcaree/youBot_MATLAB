classdef Youbot < handle
    %YOUBOT A class to handle ROS control of a Kuka youBot
    %   Instansiate the object with the ROS namespace of the youBot, if one
    %   has been set, otherwise no arguments are needed. For example:
    %
    %       myYoubot = Youbot('youbot1')
    %
    %   To move/orient the base of the youBot
    %
    %       myYoubot.BaseVelocity(xVel, yVel, omega)
    %
    %   where xVel and yVel are in meters/second and omega is
    %   radians/second
    %
    %   To stop the youBot at any time
    %
    %       myYoubot.Stop()
    %
    %   To move the arm
    %
    %       myYoubot.ArmPosition(jointPositions)
    %
    %   where jointPositions is a 5 element vector containing the
    %   individual join positions in radians
    %
    %   To move the arm back to a stowed position
    %
    %       myYoubot.StowArm()
    %
    %   To move the gripper to a specific position
    %
    %       myYoubot.MoveGripper(distance)
    %
    %   where position is the distance to move each finger in meters
    %
    %   To open/close the gripper
    %
    %       myYoubot.OpenGripper()
    %       myYoubot.CloseGripper()
    %
    %   Upon destruction of the object, the Stop and StowArm methods are
    %   called automatically
    %
    properties (SetAccess = private)
        Name
    end
    properties (Access = private)
        ArmPublisher
        BasePublisher
        GripperPublisher
    end
    properties (Access = private, Dependent)
        Namespace
        ArmTopic
        BaseTopic
        GripperTopic
    end 
    
    methods
        function this = Youbot(varargin)
            switch (nargin)
                case 0
                    this.Name = '';
                case 1
                    this.Name = varargin{1};
            end
            this.ArmPublisher = rospublisher(this.ArmTopic);
            this.BasePublisher = rospublisher(this.BaseTopic);
            this.GripperPublisher = rospublisher(this.GripperTopic);
        end
        function path = get.Namespace(this)
            if (isempty(this.Name))
                path = '/';
            else
                path = ['/' this.Name '/'];
            end
        end
        function path = get.ArmTopic(this)
            path = [this.Namespace 'arm_1/arm_controller/position_command'];
        end
        function path = get.BaseTopic(this)
            path = [this.Namespace 'cmd_vel'];
        end
        function path = get.GripperTopic(this)
            path = [this.Namespace 'arm_1/gripper_controller/position_command'];
        end
        
        function BaseVelocity(this, x, y, omega)
            message = rosmessage(this.BasePublisher);
            message.Linear.X = x;
            message.Linear.Y = y;
            message.Angular.Z = omega;
            send(this.BasePublisher,message);
        end
        function Stop(this)
            this.BaseVelocity(0,0,0);
        end
        function ArmPosition(this,armPosition)
            if length(armPosition) ~= 5
                return;
            end
            for i=1:length(armPosition)
                joints(i) = rosmessage('brics_actuator/JointValue');
                joints(i).JointUri = ['arm_joint_' num2str(i)];
                joints(i).Value = armPosition(i);
                joints(i).Unit = 'rad';
            end
            message = rosmessage(this.ArmPublisher);
            message.Positions = joints;
            send(this.ArmPublisher,message);
        end
        function StowArm(this)
            this.ArmPosition([0.011 0.011 -0.016 0.023 0.12]);
        end
        function MoveGripper(this, position)
            gripper(1) = rosmessage('brics_actuator/JointValue');
            gripper(1).JointUri = 'gripper_finger_joint_l';
            gripper(1).Value = position;
            gripper(1).Unit = 'm';
            gripper(2) = rosmessage('brics_actuator/JointValue');
            gripper(2).JointUri = 'gripper_finger_joint_r';
            gripper(2).Value = position;
            gripper(2).Unit = 'm';
            message = rosmessage(this.GripperPublisher);
            message.Positions = gripper;
            send(this.GripperPublisher,message);
        end
        function OpenGripper(this)
            this.MoveGripper(0.0114);
        end
        function CloseGripper(this)
            this.MoveGripper(0);
        end
        
        function delete(this)
            this.Stop();
            this.StowArm();
        end
    end
    
end

