package org.firstinspires.ftc.teamcode.main;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * The `Status` class serves as a centralized repository for tracking the
 * operational state and positional context of various components. It
 * provides a unified interface for monitoring the current condition and
 * status of different parts of the robot, ensuring a cohesive understanding
 * of the robot's overall configuration at any given time.
 */
@Configurable
public class Status {
    // Robot Pose
        public static Pose2D currentPose = new Pose2D(DistanceUnit.CM,0,0, AngleUnit.DEGREES,0);
        public static double currentX = 0;
        public static double currentY = 0;
        public static double currentHeading = 0;
        public static double currentVelocityX = 0;
        public static double currentVelocityY = 0;
        public static double currentVelocityHeading = 0;

    // Main Toggles
        public static boolean competitionMode = true;
        public static boolean loggingToFile = false;
        public static boolean debugMode = false;
        public static boolean opModeIsActive = false;

    // Match Info

        public static Constants.Game.ALLIANCE alliance = Constants.Game.ALLIANCE.RED;
        public static Pose2D flowerResetPose;
        public static Pose2D goalPose;

    // Data Checks
        public static boolean isDrivingActive = false;
        public static boolean parked = false;
        public static boolean flywheelAtTargetSpeed = true;
        public static boolean wingsDown = false;
        public static boolean tongueDown = false;
        public static boolean proboscisDown = false;
    // Robot Toggles
        public static boolean useBetterIMU = false;
        public static boolean fieldOriented = false;
        public static boolean shootOnTheMove = false;
        public static boolean manualControl = false;
        public static boolean flywheelToggle = false;
}
