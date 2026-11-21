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
        Status.currentPose = HelperFunctions.audienceOriginToPinpoint(pose);
        Status.currentX = Status.currentPose.getX(DistanceUnit.CM);
        Status.currentY = Status.currentPose.getY(DistanceUnit.CM);
        Status.currentHeading = Status.currentPose.getHeading(AngleUnit.DEGREES);
        Status.currentVelocityX = RobotContainer.HardwareDevices.pinpoint.getVelX(DistanceUnit.CM);
        Status.currentVelocityY = RobotContainer.HardwareDevices.pinpoint.getVelY(DistanceUnit.CM);
        Status.currentVelocityHeading = RobotContainer.HardwareDevices.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
    }

    public void updateRobotPose() {
        Status.currentPose = HelperFunctions.pinpointToAudienceOrigin(RobotContainer.HardwareDevices.pinpoint.getPosition());
        Status.currentX = Status.currentPose.getX(DistanceUnit.CM) * Constants.System.PINPOINT_X_SCALER;
        Status.currentY = Status.currentPose.getY(DistanceUnit.CM) * Constants.System.PINPOINT_Y_SCALER;
        Status.currentHeading = Status.currentPose.getHeading(AngleUnit.DEGREES) - ((RobotContainer.HardwareDevices.pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES) / 360) * Constants.System.PINPOINT_PER_ROTATION_OFFSET);
        Status.currentVelocityX = RobotContainer.HardwareDevices.pinpoint.getVelX(DistanceUnit.CM);
        Status.currentVelocityY = RobotContainer.HardwareDevices.pinpoint.getVelY(DistanceUnit.CM);
        Status.currentVelocityHeading = RobotContainer.HardwareDevices.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        if (Status.useBetterIMU) getRotatedRobotPose();
        }

    public void updateGoalPose(){
        // Update Goal Pose
            // X Values
            double xMultiplier = Status.alliance == Constants.Game.ALLIANCE.BLUE ? 1 : -1;
            double goalX = xMultiplier * Constants.Game.CELL_POSITION_X;
            double xShift = Math.signum(Status.currentY) * 0;
            if (Status.shootOnTheMove) xShift -= Status.currentVelocityX;
            xShift += HelperFunctions.clamp((goalX - Status.currentX) / Constants.Turret.X_THRESHOLD, -1, 1) * Constants.Turret.GOAL_SHIFT_AMPLITUDE;

            // Y Values
            double goalY = Math.signum(Status.currentY) * Constants.Game.CELL_POSITION_Y ;
            double yShift = Math.signum(Status.currentX) * 0;
            if (Status.shootOnTheMove) yShift -= Status.currentVelocityY;

        Status.goalPose = new Pose2D(DistanceUnit.CM, goalX + xShift, goalY + yShift, AngleUnit.DEGREES, 0);
    }

    public void update(){
        updateRobotPose();
        updateGoalPose();
    }
}
