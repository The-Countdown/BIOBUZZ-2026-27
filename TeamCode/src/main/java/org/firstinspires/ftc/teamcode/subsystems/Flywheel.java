package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.LinkedMotors;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.FlywheelPDF;

public class Flywheel {
    private final LinkedMotors flyWheelMotors;
    private final RobotContainer robotContainer;
    private FlywheelPDF flywheelPDF;
    public double targetVelocity;
    public double error;
    public double powerModifier;

    public Flywheel(RobotContainer robotContainer, LinkedMotors flyWheelMotors) {
        this.flyWheelMotors = flyWheelMotors;
        this.robotContainer = robotContainer;
        this.flywheelPDF = new FlywheelPDF();
        this.targetVelocity = 0;
        this.error = 0;
        this.powerModifier = 0;
    }

    public void update(boolean teleop){
        if (Status.flywheelToggle) {
            targetVelocity = 1; // tableInterpolate(Constants.Flywheel.SPEED_TABLE_DISTANCES, Constants.Flywheel.SPEED_TABLE, HelperFunctions.disToGoal()) - (goalOrientedVelocityX * Constants.Turret.FLYWHEEL_POWER_VELOCITY_MULTIPLIER);
        } else {
            targetVelocity = -powerModifier;
        }

        if (teleop) {
            // This will either increase or decrease the flywheel power by small increments
            if (robotContainer.operatorController.dpadDown.wasJustPressed()){
                powerModifier -= 0.02;
            }

            if (robotContainer.operatorController.dpadUp.wasJustPressed()){
                powerModifier += 0.02;
            }

            if (robotContainer.operatorController.dpadUp.isHeldFor(1.0) || robotContainer.operatorController.dpadDown.isHeldFor(1.0)) {
                powerModifier = 0;
            }

            if (robotContainer.operatorController.cross.isHeldFor(1.0)) {
                Status.shootOnTheMove = !Status.shootOnTheMove;
            }

            if (robotContainer.driverController.cross.wasJustPressed() || robotContainer.operatorController.cross.wasJustPressed()) {
                Status.flywheelToggle = !Status.flywheelToggle;
            }
        }

        targetVelocity = (targetVelocity + powerModifier) * Constants.Flywheel.MAX_VELOCITY;

        if (targetVelocity > 10) {
            updateError();
        } else {
            error = 0;
        }

        Status.flywheelAtTargetSpeed = atTargetVelocity();

        flyWheelMotors.setPower(flywheelPDF.calculate(targetVelocity));
    }

    public boolean atTargetVelocity() {
        return Math.abs(error) < 60 && targetVelocity > 10;
    }
    public void updateError() {
        error = targetVelocity - flyWheelMotors.getVelocity();
    }

    public double getFlywheelVelocity() {
        return flyWheelMotors.getAverageVelocity();
    }

    public double getCurrent(){
        return flyWheelMotors.getCurrent(CurrentUnit.AMPS);
    }
    @Deprecated // Only use/update this for personal challenge
    public void updateLaunchValues(double distToGoal){
        double verticalVel = Math.sqrt(2 * Constants.Game.GRAVITY * (Constants.Flywheel.DESIRED_MAX_HEIGHT - Constants.Flywheel.HEIGHT));
        double estimatedTime = (verticalVel + Math.sqrt(verticalVel - 2 * (Constants.Game.GRAVITY) * (Constants.Game.GOAL_HEIGHT - Constants.Flywheel.HEIGHT))) / Constants.Game.GRAVITY;
        double horizonalVel = distToGoal / estimatedTime;
        double hoodAngle = Math.atan(verticalVel / horizonalVel);
        double xDiff;
        double yDiff;
        if (Status.alliance == Constants.Game.ALLIANCE.BLUE){
            xDiff = -Status.goalPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
            yDiff = Status.goalPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);
        } else {
            xDiff = Status.goalPose.getX(DistanceUnit.CM) - Status.currentPose.getX(DistanceUnit.CM);
            yDiff = -Status.goalPose.getY(DistanceUnit.CM) - Status.currentPose.getY(DistanceUnit.CM);
        }
        double angleToFaceGoal = Math.atan2(yDiff, xDiff);
        double initialVel = Math.sqrt(Math.pow((verticalVel), 2) * Math.pow((horizonalVel), 2));
        double rpm = initialVel * (5000 / 12.5); //TODO: Stephan have fun with your calc IA find that number, 5000/12.5 is an estimate from an online shooter calculator
        targetVelocity = (rpm / 60) * Constants.Robot.MOTOR_TICKS_PER_REVOLUTION;
    }

}
