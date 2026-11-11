package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.util.TurretPDF;

public class Turret extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final BetterDcMotor turretMotor;
    public final BetterServo hoodServo;
    private TurretPDF turretPDF;
    public double currentPosition;
    private double targetPosition;
    private double targetPositionDegrees;
    private double manualTurretPos;
    public double turretAngleOffset;
    private double goalAngle;
    public double goalOrientedVelocityX;
    public double goalOrientedVelocityY;
    public double goalOrientedVelocityRX;
    public double angleToFaceGoal;
    public double lastAngleToFaceGoal;
    public double turretPDFOutput;

    public Turret(RobotContainer robotContainer, BetterServo hoodServo, BetterDcMotor turretMotor) {
        this.robotContainer = robotContainer;
        this.hoodServo = hoodServo;
        this.turretMotor = turretMotor;
        this.turretPDF = new TurretPDF();
        this.turretPDFOutput = 0;
        this.targetPosition = 0;
        this.manualTurretPos = 0;
        this.turretAngleOffset = 0;
        this.currentPosition = turretMotor.getCurrentPosition();

        flywheelMotorLeader.start();
        flywheelMotorFollower.start();
        turretMotor.start();
    }

    public void update(boolean teleop) {
        //robotContainer.turret.hood.setPos(HelperFunctions.clamp(tableInterpolate(Constants.Turret.HOOD_TABLE_DISTANCES, Constants.Turret.HOOD_PRESETS, HelperFunctions.disToGoal()), Constants.Turret.HOOD_PRESETS[0], Constants.Turret.HOOD_PRESETS[Constants.Turret.HOOD_PRESETS.length-1]));
        currentPosition = turretMotor.getCurrentPosition();

        if (Status.currentY < 0) {
            goalAngle = 180;
        } else {
            goalAngle = 0;
        }

        if (teleop) {

            if (robotContainer.operatorController.dpadLeft.wasJustPressed()) {
                turretAngleOffset -= 2;
            }

            if (robotContainer.operatorController.dpadRight.wasJustPressed()) {
                turretAngleOffset += 1.5;
            }

            if (robotContainer.operatorController.dpadRight.isHeldFor(1.0) || robotContainer.operatorController.dpadLeft.isHeldFor(1.0)) {
                turretAngleOffset = 0;
            }

            // Manual Turret Hood
            if (robotContainer.operatorController.triangle.wasJustPressed()) {
//                double targetHoodPos = robotContainer.turret.hoodServo.getPosition() == Constants.Turret.HOOD_PRESETS[2]  ? Constants.Turret.HOOD_PRESETS[0] : Constants.Turret.HOOD_PRESETS[1];
//                robotContainer.turret.hood.setPos(targetHoodPos);
            }

            if (robotContainer.operatorController.triangle.isHeldFor(1.0)){
                Status.manualControl = !Status.manualControl;
            }

            // Turret turn - Right stick X
             if (Status.manualControl && robotContainer.operatorController.rightStickX() != 0) {
                 // Manual turret turning
                 manualTurretPos -= robotContainer.operatorController.rightStickX() != 0 ? (Constants.Turret.SPEED_FACTOR * robotContainer.CURRENT_LOOP_TIME_MS) * Math.pow(robotContainer.operatorController.rightStickX(), 3) : 0;
                 manualTurretPos = HelperFunctions.clamp(manualTurretPos, Constants.Turret.MIN_MOTOR, Constants.Turret.MAX_MOTOR);
                 robotContainer.turret.setTargetAngle(manualTurretPos);
             } else if (!Status.manualControl) {
                 robotContainer.turret.pointAtGoal();
             }
        } else {
            robotContainer.turret.pointAtGoal();
        }

        lastAngleToFaceGoal = angleToFaceGoal;
    }

    public double getTargetPosition(){ return targetPosition;}

    public double getTargetPositionDegrees(){return targetPositionDegrees;}

    public void setTargetAngle(double angleInDegrees) {

        targetPositionDegrees = HelperFunctions.normalizeAngle(angleInDegrees + turretAngleOffset);
        targetPosition = angleToMotor(targetPositionDegrees);

        double position = HelperFunctions.clamp(targetPosition, Constants.Turret.MIN_MOTOR, Constants.Turret.MAX_MOTOR);

        turretPDFOutput = Status.shootOnTheMove ? turretPDF.calculate(position, currentPosition, goalOrientedVelocityRX) : turretPDF.calculate(position, currentPosition, 0);

        turretMotor.updateSetPower(turretPDFOutput);
    }

    private double angleToMotor(double angle) {
        return ((HelperFunctions.normalizeAngle(angle) + 180) / Constants.Turret.DEGREES_PER_TICK) + Constants.Turret.MIN_MOTOR;
    }

    private double motorToAngle(double motorTicks) {
        return HelperFunctions.normalizeAngle((motorTicks - Constants.Turret.MIN_MOTOR) * Constants.Turret.DEGREES_PER_TICK);
    }

    public double getPosition() {
        return currentPosition;
    }

    public double getPositionDegrees() {
        return motorToAngle(currentPosition);
    }

    public void pointAtGoal() {
        goalOrientedVelocityX = (Status.currentVelocityX * Math.cos(Math.toRadians(goalAngle))) - (Status.currentVelocityY * Math.sin(Math.toRadians(goalAngle)));
        goalOrientedVelocityY = (Status.currentVelocityX * Math.sin(Math.toRadians(goalAngle))) + (Status.currentVelocityY * Math.cos(Math.toRadians(goalAngle)));

        double xDiff = Status.alliance == Constants.Game.ALLIANCE.RED ?
                Status.goalPose.getX(DistanceUnit.CM) - Status.currentX :
                Status.goalPose.getX(DistanceUnit.CM) + Status.currentX;
        xDiff += goalOrientedVelocityX;

        double yDiff = Status.alliance == Constants.Game.ALLIANCE.RED ?
                Status.goalPose.getY(DistanceUnit.CM) + Status.currentY :
                Status.goalPose.getY(DistanceUnit.CM) - Status.currentY;
        yDiff += goalOrientedVelocityY;

        angleToFaceGoal = ((Math.atan(yDiff/xDiff) * (180 / Math.PI)) + Status.currentHeading);
        goalOrientedVelocityRX =  ((angleToFaceGoal - lastAngleToFaceGoal) * 1000 )/ robotContainer.CURRENT_LOOP_TIME_MS;

        setTargetAngle(HelperFunctions.normalizeAngle(angleToFaceGoal));
    }

    public boolean atTarget() {
        return Math.abs(getPosition() - targetPosition) < 10;
    }
    public class Hood {
        public void setPos(double pos) {
            hoodServo.updateSetPosition(pos);
        }
    }
    public final Hood hood = new Hood();
}
