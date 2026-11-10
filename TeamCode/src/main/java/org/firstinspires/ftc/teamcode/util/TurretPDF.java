package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

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
    private double ffMultiplier;
    private double lastTime;
    private double lastTargetPower;
    public TelemetryManager panelsTelemetry;

    public TurretPDF(RobotContainer robotContainer, BetterDcMotor turretMotor) {
        this.robotContainer = robotContainer;
        this.turretMotor = turretMotor;
        this.lastTargetPower = 0;
        this.lastTime = 0;
        this.ffMultiplier = 0;

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /**
     * Calculates the PDF output for the servo.
     * @return The calculated PDF output.
     */
    public double calculate(double target, double currentPosition, double goalOrientedRX) {
        if (Math.abs(target - currentPosition) < Constants.Turret.TOLERANCE_TICKS) {
            lastTargetPower = 0;
            return 0;
        }

        ffMultiplier = Math.abs(target - currentPosition) < 2.5 * Constants.Turret.TOLERANCE_TICKS ? -1 :1;

        double error = target - currentPosition;
        dt = System.currentTimeMillis() - lastTime;
        goalOrientedRX /= dt;
        errorDiff = (error - lastError) / dt; // differance between the errors scaled by time

            p = Constants.Turret.KP * error; // proportional to error

            i += error * Constants.Turret.KI;

            d = Constants.Turret.KD * errorDiff; // this slows the acceleration down

            ff = Constants.Turret.KF * ffMultiplier + (goalOrientedRX / Constants.Turret.GEAR_RATIO) * (Constants.Robot.MOTOR_TICKS_PER_REVOLUTION / (2 * Math.PI)); // 1. flat addition to the power 2. extra velocity in ticks per second for the motor


        pid = (p + i + d) + ff * Math.signum(error);

        lastTargetPower = pid;

        lastTargetPower = HelperFunctions.clamp(lastTargetPower, -1, 1);

         if (!Status.competitionMode || Status.debugMode){
            panelsTelemetry.addData("Power", lastTargetPower);
            panelsTelemetry.addData("P", p);
            panelsTelemetry.addData("I", i);
            panelsTelemetry.addData("D", d);
            panelsTelemetry.addData("FF", ff);
            panelsTelemetry.addData("PID", pid);
            panelsTelemetry.addData("DT", dt);
            panelsTelemetry.addData("Target", target);
            panelsTelemetry.addData("Position", currentPosition);
            panelsTelemetry.addData("Error", error);
            panelsTelemetry.addData("error Diff", errorDiff);
            panelsTelemetry.update();
        }

        lastError = error;
        lastTime += dt;
        return lastTargetPower;
    }
}
