package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.Status;

public class TurretPDF {
    private double error;
    private double lastError;
    private double p;
    private double i;
    private double ff;
    private double vff;
    private double d;
    private double pid;
    private double dt;
    private double errorDiff;
    private double lastTime;
    private double lastTargetVelocity;
    public TelemetryManager panelsTelemetry;

    public TurretPDF() {
        this.lastTargetVelocity = 0;
        this.lastTime = 0;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /**
     * Calculates the PDF output for the servo.
     * @return The calculated PDF output.
     */
    public double calculate(double target, double currentPosition, double headingVelocity) {
        if (Math.abs(target - currentPosition) < Constants.Turret.TOLERANCE_TICKS) {
            lastTargetVelocity = 0;
//            if (Status.debugMode){ panelsTelemetry.addLine("Returned within tolerance"); panelsTelemetry.update(); }
            return 0;
        }

        double error = target - currentPosition;
        dt = System.currentTimeMillis() - lastTime;
        errorDiff = (error - lastError) / dt; // differance between the errors scaled by time

            p = Constants.Turret.KP * error; // proportional to error

            i += error * Constants.Turret.KI; // accounts for error over time

            d = Constants.Turret.KD * errorDiff; // this slows the acceleration down

            ff = Constants.Turret.KF * Math.signum(error); // flat addition to the power

            vff = (headingVelocity / Constants.Turret.GEAR_RATIO) * (Constants.Robot.MOTOR_TICKS_PER_REVOLUTION / (2 * Math.PI)); // extra velocity in ticks per second for the motor


        pid = ((p + i + d) + ff) * 100 * Constants.Robot.MOTOR_TICKS_PER_REVOLUTION + vff;

        lastTargetVelocity = pid;

        lastTargetVelocity = HelperFunctions.clamp(lastTargetVelocity, -100 * Constants.Robot.MOTOR_TICKS_PER_REVOLUTION, 100 * Constants.Robot.MOTOR_TICKS_PER_REVOLUTION);

         if (Status.debugMode){
//            panelsTelemetry.addData("Velocity", lastTargetVelocity);
//            panelsTelemetry.addData("P", p);
//            panelsTelemetry.addData("I", i);
//            panelsTelemetry.addData("D", d);
//            panelsTelemetry.addData("FF", ff);
//            panelsTelemetry.addData("PID", pid);
//            panelsTelemetry.addData("DT", dt);
//            panelsTelemetry.addData("Target Position", target);
//            panelsTelemetry.addData("Position", currentPosition);
//            panelsTelemetry.addData("Error", error);
//            panelsTelemetry.addData("error Diff", errorDiff);
//            panelsTelemetry.update();
        }

        lastError = error;
        lastTime += dt;
        return lastTargetVelocity;
    }
}
