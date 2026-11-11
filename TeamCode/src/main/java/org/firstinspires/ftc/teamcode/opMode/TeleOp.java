package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {
    private RobotContainer robotContainer;

    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();

        Status.currentPose = (Pose2D) blackboard.getOrDefault("Robot Pose", new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));
        RobotContainer.HardwareDevices.pinpoint.setPosition(Status.currentPose);
//        RobotContainer.HardwareDevices.betterIMU.resetAngle();
//        RobotContainer.HardwareDevices.betterIMU.setAngleOffset(-Status.currentHeading);
    }

    @Override
    public void init_loop() {
        robotContainer.initLoop();
    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        robotContainer.start(this, true);
        Status.isDrivingActive = true;
    }

    @Override
    public void loop() {
        robotContainer.update(true);
        Thread.yield();
    }

    @Override
    public void stop(){
        robotContainer.stop();
    }
}
