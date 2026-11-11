package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.LinkedServos;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;


public class Tongue extends RobotContainer.HardwareDevices{
    public LinkedServos tongueServos;

    public Constants constants;

    public final RobotContainer robotContainer;

    public Status status;
    public Tongue(RobotContainer robotContainer, LinkedServos tongueServos) {
        this.robotContainer = robotContainer;
        this.tongueServos = tongueServos;
    }

    public void update(boolean teleop){

        if (teleop) {
            if (robotContainer.driverController.rightBumper.isHeld() || robotContainer.operatorController.rightBumper.isHeld()) {
                up();
            } else if (!Status.tongueDown) {
                down();
            }
        }
    }

    public void setPosition(double position){
        tongueServos.setPosition(HelperFunctions.clamp(position, Constants.Tongue.UP, Constants.Tongue.DOWN),0);
        Status.tongueDown = false;
    }
    public void up(){
        tongueServos.setOffsetMirroredPositions(Constants.Tongue.UP, Constants.Tongue.DOWN, Constants.Tongue.FOLLOWER_OFFSET);
//        RobotContainer.HardwareDevices.tongueServoLeader.updateSetPosition(0.15);
//        RobotContainer.HardwareDevices.tongueServoFollower.updateSetPosition(0.65);
        Status.tongueDown = false;
    }
    public void down(){
        tongueServos.setOffsetMirroredPositions(Constants.Tongue.DOWN, Constants.Tongue.UP, Constants.Tongue.FOLLOWER_OFFSET);
//        RobotContainer.HardwareDevices.tongueServoLeader.updateSetPosition(0.575);
//        RobotContainer.HardwareDevices.tongueServoFollower.updateSetPosition(0.225);
        Status.tongueDown = true;
    }
}
