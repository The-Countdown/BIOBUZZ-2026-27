package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;


public class PositionProvider {
    private RobotContainer robotContainer;

    public PositionProvider(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    public void getRotatedRobotPose() {
        // Rotate the odPose by the betterIMU offset
        double newHeading = Status.currentHeading; //RobotContainer.HardwareDevices.betterIMU.getHeading();
        double headingDiff = Math.toRadians(newHeading - Status.currentHeading);
        double newX = Status.currentX * Math.cos(-headingDiff) - Status.currentY * Math.sin(-headingDiff);
        double newY = Status.currentY * Math.cos(-headingDiff) + Status.currentX * Math.sin(-headingDiff);

        Status.currentPose = new Pose2D(DistanceUnit.CM, newX, newY, AngleUnit.DEGREES, newHeading);
        Status.currentX = Status.currentPose.getX(DistanceUnit.CM);
        Status.currentY = Status.currentPose.getY(DistanceUnit.CM);
    }

    public void update() {
        Status.currentPose = RobotContainer.HardwareDevices.pinpoint.getPosition();
        Status.currentX = Status.currentPose.getX(DistanceUnit.CM);
        Status.currentY = Status.currentPose.getY(DistanceUnit.CM);
        Status.currentHeading = Status.currentPose.getHeading(AngleUnit.DEGREES);
        Status.currentVelocityX = RobotContainer.HardwareDevices.pinpoint.getVelX(DistanceUnit.CM);
        Status.currentVelocityY = RobotContainer.HardwareDevices.pinpoint.getVelY(DistanceUnit.CM);
        Status.currentVelocityHeading = RobotContainer.HardwareDevices.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        if (Status.useBetterIMU) getRotatedRobotPose();
        }
    }
