package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;

public class TurretPDF {
    private final RobotContainer robotContainer;
    private BetterDcMotor turretMotor;

    private double error;
    private double lastError;
    private double p;
    private double i;
    private double s;
    private double ff;
    private double d;
    private double pid;
    private double dt;
    private double errorDiff;
    private double lastTime;
    private double lastTargetPower;
    public TelemetryManager panelsTelemetry;

    public TurretPDF(RobotContainer robotContainer, BetterDcMotor turretMotor) {
        this.robotContainer = robotContainer;
        this.turretMotor = turretMotor;
        this.lastTargetPower = 0;
        this.lastTime = 0;

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /**
     * Calculates the PDF output for the servo.
     * @return The calculated PDF output.
     */
    public double calculate(double positionToFaceGoal, double goalOrientedRX) {
        if (positionToFaceGoal == turretMotor.getCurrentPosition()) {
            lastTargetPower = 0;
            return 0;
        }
        double error = positionToFaceGoal - turretMotor.getCurrentPosition();
        dt = System.currentTimeMillis() - lastTime;
        goalOrientedRX /= dt;
        errorDiff = (error - lastError) / dt; // differance between the errors scaled by time

            p = Constants.Turret.FLYWHEEL_KP * error; // proportional to error

            i += error * Constants.Turret.FLYWHEEL_KI;

            d = Constants.Turret.FLYWHEEL_KD  * errorDiff; // this slows the acceleration down

            ff = Constants.Turret.FLYWHEEL_KF + (goalOrientedRX / Constants.Turret.TURRET_GEAR_RATIO) * (Constants.Robot.MOTOR_TICKS_PER_REVOLUTION / (2 * Math.PI)); // 1. flat addition to the power 2. extra velocity in ticks per second for the motor


        pid = (p + i + d) + ff;

        lastTargetPower = pid;

        lastTargetPower = HelperFunctions.clamp(lastTargetPower, 0, 1);

//        if (!Status.competitionMode){
            panelsTelemetry.addData("Power", lastTargetPower);
            panelsTelemetry.addData("P", p);
            panelsTelemetry.addData("I", i);
            panelsTelemetry.addData("D", d);
            panelsTelemetry.addData("FF", ff);
            panelsTelemetry.addData("PID", pid);
            panelsTelemetry.addData("DT", dt);
            panelsTelemetry.addData("Target", positionToFaceGoal);
            panelsTelemetry.addData("Velocity", turretMotor.getVelocity());
            panelsTelemetry.addData("Error", error);
            panelsTelemetry.addData("error Diff", errorDiff);
//        }

        lastError = error;
        lastTime += dt;
        return lastTargetPower;
    }
}
