# youBot_MATLAB
Control of a Kuka youBot from MATLAB using ROS and the Robotics System Toolbox

To use this class it is recommended to configure the ROS_MASTER_URI of your youBot to point at your MATLAB machine then start off by running (in MATLAB)

    >> rosinit()

Then on your youBot, launch the basic ROS interface with

    $ roslaunch youbot_driver_ros_interface youbot_driver.launch

Now you can use the Youbot class as detailed in the 'help'...

    >> help Youbot
     Youbot A class to handle ROS control of a Kuka youBot
        Instansiate the object with the ROS namespace of the youBot, if one
        has been set, otherwise no arguments are needed. For example:

            myYoubot = Youbot('youbot1')

        To move/orient the base of the youBot

            myYoubot.BaseVelocity(xVel, yVel, omega)

        where xVel and yVel are in meters/second and omega is
        radians/second

        To stop the youBot at any time

            myYoubot.Stop()

        To move the arm

            myYoubot.ArmPosition(jointPositions)

        where jointPositions is a 5 element vector containing the
        individual join positions in radians

        To move the arm back to a stowed position

            myYoubot.StowArm()

        To move the gripper to a specific position

            myYoubot.MoveGripper(distance)

        where position is the distance to move each finger in meters

        To open/close the gripper

            myYoubot.OpenGripper()
            myYoubot.CloseGripper()

        Upon destruction of the object, the Stop and StowArm methods are
        called automatically