package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.hardware.LinkedMotors;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

public class FlywheelPDF {
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
    private double dropTime;
    public TelemetryManager panelsTelemetry;

    public FlywheelPDF() {
        this.lastTargetPower = 0;
        this.lastTime = 0;
        this.dropTime = 0;
        this.i = 0;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /**
     * Calculates the PDF output for the servo.
     * @return The calculated PDF output.
     */
    public double calculate(double targetSpeed) {
        if (targetSpeed == 0) {
            lastTargetPower = 0;
            if (Status.debugMode){ panelsTelemetry.addLine("Returned within tolerance"); panelsTelemetry.update(); }
            return 0;
        }
        double error = targetSpeed - RobotContainer.HardwareDevices.flywheelMotorLeader.getVelocity();
        dt = System.currentTimeMillis() - lastTime + 1;
        errorDiff = (lastError - error) / dt; // differance between the errors scaled by time

        if (errorDiff < -4 & System.currentTimeMillis() - dropTime > 250 ){
            dropTime = System.currentTimeMillis();
        }

//            s = Math.signum(error)* Math.pow(error, 2) * Constants.Flywheel.KS; // This is tuned just like p

            p = Constants.Flywheel.KP * error; // proportional to error

            p = p < 0 ? p / Constants.Flywheel.KS : p;

            i += error * Constants.Flywheel.KI;

            d = Constants.Flywheel.KD  * -errorDiff; // this slows the acceleration down

            ff = (targetSpeed * Constants.Flywheel.KF) / Constants.Flywheel.MAX_VELOCITY; // flat addition to the power


        pid = (p + i + d) * dt + ff;

        lastTargetPower = HelperFunctions.clamp(pid, 0, 1);

        if (Status.debugMode){
            panelsTelemetry.addData("Power", lastTargetPower);
            panelsTelemetry.addData("P", p);
            panelsTelemetry.addData("I", i);
            panelsTelemetry.addData("D", d);
    //        panelsTelemetry.addData("S", s);
            panelsTelemetry.addData("FF", ff);
            panelsTelemetry.addData("PID", pid);
            panelsTelemetry.addData("DT", dt);
            panelsTelemetry.addData("Drop Time", dropTime);
            panelsTelemetry.addData("Target Velocity", targetSpeed);
            panelsTelemetry.addData("Velocity", RobotContainer.HardwareDevices.flywheelMotorLeader.getVelocity());
            panelsTelemetry.addData("Error", error);
            panelsTelemetry.addData("error Diff", errorDiff);

//        panelsTelemetry.update();
        }

        lastError = error;
        lastTime += dt;
        return lastTargetPower;
    }
}
