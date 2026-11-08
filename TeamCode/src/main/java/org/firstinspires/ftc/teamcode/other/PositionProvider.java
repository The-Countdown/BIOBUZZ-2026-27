package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;


public class PositionProvider {
    private RobotContainer robotContainer;
    private GoBildaPinpointDriver pinpoint;
    private Pose2D lastODPose;
    private Pose2D startODPose;

    public PositionProvider(RobotContainer robotContainer, GoBildaPinpointDriver pinpoint) {
        this.robotContainer = robotContainer;
        this.pinpoint = pinpoint;
        this.startODPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
    }

    public Pose2D getRotatedRobotPose() {
        // Rotate the odPose by the vision offset
        double newHeading = Status.currentHeading; //RobotContainer.HardwareDevices.betterIMU.getHeading();
        double headingDiff = Math.toRadians(newHeading - Status.currentHeading);
        double newX = Status.currentX * Math.cos(-headingDiff) - Status.currentY * Math.sin(-headingDiff);
        double newY = Status.currentY * Math.cos(-headingDiff) + Status.currentX * Math.sin(-headingDiff);
        return new Pose2D(DistanceUnit.CM, newX, newY, AngleUnit.DEGREES, newHeading);
    }

    public void update() {
        Status.currentPose = Status.useBetterIMU ? RobotContainer.HardwareDevices.pinpoint.getPosition() : getRotatedRobotPose();
        Status.currentX = Status.currentPose.getX(DistanceUnit.CM);
        Status.currentY = Status.currentPose.getY(DistanceUnit.CM);
        Status.currentHeading = Status.currentPose.getHeading(AngleUnit.DEGREES);
        Status.currentVelocityX = RobotContainer.HardwareDevices.pinpoint.getVelX(DistanceUnit.CM);
        Status.currentVelocityY = RobotContainer.HardwareDevices.pinpoint.getVelY(DistanceUnit.CM);
        Status.currentVelocityHeading = RobotContainer.HardwareDevices.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        }
    }
