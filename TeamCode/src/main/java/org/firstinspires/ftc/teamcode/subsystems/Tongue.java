package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptNullOp;
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
    private int tickNum;
    public Tongue(RobotContainer robotContainer, LinkedServos tongueServos) {
        this.robotContainer = robotContainer;
        this.tongueServos = tongueServos;
        this.tickNum = 0;
    }

    public void update(boolean teleop){

        if (teleop) {
            if (robotContainer.driverController.rightBumper.wasJustReleased()){
                tick();
            } else if (!Status.tongueDown) {
                down();
            }
        }
    }

    public void setPosition(double position){
        tongueServos.setPosition(HelperFunctions.clamp(position, Constants.Tongue.UP, Constants.Tongue.DOWN),0);
        Status.tongueDown = false;
    }

    public void tick(){
        if (tickNum != Constants.Tongue.TICK_NUMBER + 1) {
            robotContainer.delayedActionManager.schedule(() -> tick(), Constants.Flywheel.SHOOT_MS / Constants.Tongue.TICK_NUMBER);
        }

        tickNum %= Constants.Tongue.TICK_NUMBER + 1;
        double tickDiff = (Constants.Tongue.DOWN - Constants.Tongue.UP);
        double servoPose = (tickDiff / Constants.Tongue.TICK_NUMBER) * tickNum + Constants.Tongue.UP;
        double inverseServoPos = Constants.Tongue.DOWN - (tickDiff / Constants.Tongue.TICK_NUMBER) * tickNum;
        tongueServos.setOffsetMirroredPositions(inverseServoPos, servoPose, Constants.Tongue.FOLLOWER_OFFSET);

        tickNum++;
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
