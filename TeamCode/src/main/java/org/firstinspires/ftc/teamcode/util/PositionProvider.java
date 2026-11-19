package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;


public class PositionProvider {

    public PositionProvider() {

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

    public void updateWithPose2D(Pose2D pose){
        Status.currentPose = pose;
        Status.currentX = Status.currentPose.getX(DistanceUnit.CM);
        Status.currentY = Status.currentPose.getY(DistanceUnit.CM);
        Status.currentHeading = Status.currentPose.getHeading(AngleUnit.DEGREES);
        Status.currentVelocityX = RobotContainer.HardwareDevices.pinpoint.getVelX(DistanceUnit.CM);
        Status.currentVelocityY = RobotContainer.HardwareDevices.pinpoint.getVelY(DistanceUnit.CM);
        Status.currentVelocityHeading = RobotContainer.HardwareDevices.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    public void updateRobotPose() {
        Status.currentPose = RobotContainer.HardwareDevices.pinpoint.getPosition();
        Status.currentX = -Status.currentPose.getY(DistanceUnit.CM) * Constants.System.PINPOINT_Y_SCALER;
        Status.currentY = Status.currentPose.getX(DistanceUnit.CM) * Constants.System.PINPOINT_X_SCALER;
        Status.currentHeading = Status.currentPose.getHeading(AngleUnit.DEGREES);
        Status.currentVelocityX = RobotContainer.HardwareDevices.pinpoint.getVelX(DistanceUnit.CM);
        Status.currentVelocityY = RobotContainer.HardwareDevices.pinpoint.getVelY(DistanceUnit.CM);
        Status.currentVelocityHeading = RobotContainer.HardwareDevices.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        if (Status.useBetterIMU) getRotatedRobotPose();
        }

    public void updateGoalPose(){
        // Update Goal Pose
        double xMultiplier = Status.alliance == Constants.Game.ALLIANCE.BLUE ? 1 : -1;
        double yMultiplier = Math.signum(Status.currentY);
        double xShift = (Math.abs(Status.currentY) > Constants.Turret.Y_THRESHOLD && Math.abs(Status.goalPose.getX(DistanceUnit.CM) + Status.currentX) > Constants.Turret.X_THRESHOLD) ? 25 * Math.signum(Status.currentX) : 0;
        Status.goalPose = new Pose2D(DistanceUnit.CM, (Constants.Game.CELL_POSITION_X + xShift) * xMultiplier, Constants.Game.CELL_POSITION_Y * yMultiplier, AngleUnit.DEGREES, 0);

    }

    public void update(){
        updateRobotPose();
        updateGoalPose();
    }
}
