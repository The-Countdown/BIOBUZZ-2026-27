package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

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
    private double lastTargetPosition;
    private double lastTargetVelocity;
    public TelemetryManager panelsTelemetry;

    public TurretPDF(RobotContainer robotContainer, BetterDcMotor turretMotor) {
        this.robotContainer = robotContainer;
        this.turretMotor = turretMotor;
        this.lastTargetPosition = 0;
        this.lastTargetVelocity = 0;
        this.lastTime = 0;

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /**
     * Calculates the PDF output for the servo.
     * @return The calculated PDF output.
     */
    public double calculate(double targetPosition, double targetVelocity) {
        if (targetPosition == turretMotor.getCurrentPosition()  && targetVelocity == lastTargetVelocity) {
            lastTargetPosition = 0;
            return 0;
        }
        double error = targetPosition - turretMotor.getVelocity();
        dt = System.currentTimeMillis() - lastTime;
        errorDiff = (lastError - error) / dt; // differance between the errors scaled by time

            p = Constants.Turret.FLYWHEEL_KP * error; // proportional to error

            i += error * Constants.Turret.FLYWHEEL_KI;

            d = Constants.Turret.FLYWHEEL_KD  * -errorDiff; // this slows the acceleration down

            ff = (targetPosition * Constants.Turret.FLYWHEEL_KF) / Constants.Turret.FLYWHEEL_MAX_VELOCITY; // flat addition to the power


        pid = (p + i + d) * dt + ff;

        lastTargetPosition = pid;

        lastTargetPosition = HelperFunctions.clamp(lastTargetPosition, 0, 1);

//        if (!Status.competitionMode){
            panelsTelemetry.addData("Power", lastTargetPosition);
            panelsTelemetry.addData("P", p);
            panelsTelemetry.addData("I", i);
            panelsTelemetry.addData("D", d);
    //        panelsTelemetry.addData("S", s);
            panelsTelemetry.addData("FF", ff);
            panelsTelemetry.addData("PID", pid);
            panelsTelemetry.addData("DT", dt);
            panelsTelemetry.addData("Target", targetPosition);
            panelsTelemetry.addData("Velocity", turretMotor.getVelocity());
            panelsTelemetry.addData("Error", error);
            panelsTelemetry.addData("error Diff", errorDiff);
//        }

        lastError = error;
        lastTime += dt;
        return lastTargetPosition;
    }
}
