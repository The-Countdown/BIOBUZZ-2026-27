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
    private double velocity;
    private double lastTime;
    private double lastTargetPower;
    public TelemetryManager panelsTelemetry;

    public FlywheelPDF() {
        this.lastTargetPower = 0;
        this.lastTime = 0;
        this.velocity = 0;
        this.i = 0;
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

            /**
             * Calculates the PDF output for the Flywheel Motor.
             * @return The calculated PDF output.
             */
            public double calculate(double target) {
                velocity = RobotContainer.HardwareDevices.flywheelMotorLeader.getVelocity();

                error = target - velocity;
                dt = System.currentTimeMillis() - lastTime;
                errorDiff = (error - lastError) / dt; // differance between the errors scaled by time

                s = Math.signum(error)* Math.pow(error, 2) * Constants.Flywheel.KS; // This is tuned just like p

                p = Constants.Flywheel.KP * error; // proportional to error

                i += error * Constants.Flywheel.KI; // accounts for error over time

                d = Constants.Flywheel.KD * errorDiff; // this slows the acceleration down

                ff = (Constants.Flywheel.KF * target) / Constants.Flywheel.MAX_VELOCITY; // 1. flat addition to the power


                pid = (s + p + i + d) + ff;

                lastTargetPower = pid;

                lastTargetPower = HelperFunctions.clamp(lastTargetPower, 0, 1);

                if (Status.debugMode){
                    panelsTelemetry.addData("Power", lastTargetPower);
                    panelsTelemetry.addData("S", s);
                    panelsTelemetry.addData("P", p);
                    panelsTelemetry.addData("I", i);
                    panelsTelemetry.addData("D", d);
                    panelsTelemetry.addData("FF", ff);
                    panelsTelemetry.addData("PID", pid);
                    panelsTelemetry.addData("DT", dt);
                    panelsTelemetry.addData("Target Velocity", target);
                    panelsTelemetry.addData("Velocity", velocity);
                    panelsTelemetry.addData("Error", error);
                    panelsTelemetry.addData("error Diff", errorDiff);
                    panelsTelemetry.update();
                }

                lastError = error;
                lastTime += dt;
                return lastTargetPower;
            }
        }
