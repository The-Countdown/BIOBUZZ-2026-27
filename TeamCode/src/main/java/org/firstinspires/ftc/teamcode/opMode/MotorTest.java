package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MotorTest", group = "TeleOp")
//@Disabled
public class MotorTest extends OpMode {
    private RobotContainer robotContainer;
    private int motorIndex = -1;
    private double motorPower = 0.0;

    public void defaultMotor(){
        RobotContainer.HardwareDevices.rightFront.setPower(0);
        RobotContainer.HardwareDevices.leftFront.setPower(0);
        RobotContainer.HardwareDevices.rightBack.setPower(0);
        RobotContainer.HardwareDevices.leftBack.setPower(0);
        RobotContainer.HardwareDevices.intakeMotor.setPower(0);
        RobotContainer.HardwareDevices.turretMotor.setPower(0);
        RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(0);
        RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(0);
    }

    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();

        RobotContainer.HardwareDevices.flywheelMotorFollower.start();
        RobotContainer.HardwareDevices.flywheelMotorLeader.start();
        RobotContainer.HardwareDevices.intakeMotor.start();
        RobotContainer.HardwareDevices.turretMotor.start();


    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        robotContainer.start(this, true);
    }

    @Override
    public void loop() {
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();
        defaultMotor();
        switch(motorIndex) {
            case -1:
                telemetry.addLine("stopped");
                break;
            case 0:
                RobotContainer.HardwareDevices.rightFront.setPower(motorPower);
                telemetry.addLine("rightFront");
                break;
            case 1:
                RobotContainer.HardwareDevices.leftFront.setPower(motorPower);
                telemetry.addLine("leftFront");
                break;
            case 2:
                RobotContainer.HardwareDevices.rightBack.setPower(motorPower);
                telemetry.addLine("rightBack");
                break;
            case 3:
                RobotContainer.HardwareDevices.leftBack.setPower(motorPower);
                telemetry.addLine("leftBack");
                break;
            case 4:
                RobotContainer.HardwareDevices.intakeMotor.setPower(motorPower);
                telemetry.addLine("intakeMotor");
                break;
            case 5:
                RobotContainer.HardwareDevices.turretMotor.setPower(motorPower);
                telemetry.addLine("turretMotor");
                break;
            case 6:
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(motorPower);
                telemetry.addLine("flyWheelMotorLeader");
                break;
            case 7:
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(motorPower);
                telemetry.addLine("flyWheelMotorFollower");
                break;
            case 8:
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(motorPower);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(motorPower);
                telemetry.addLine("Flywheel motors");
                break;
            case 9:
                RobotContainer.HardwareDevices.intakeMotor.setPower(motorPower);
                RobotContainer.HardwareDevices.turretMotor.setPower(motorPower);
                telemetry.addLine("Intake motors");
                break;
            case 10:
                RobotContainer.HardwareDevices.flywheelMotorFollower.setPower(motorPower);
                RobotContainer.HardwareDevices.flywheelMotorLeader.setPower(motorPower);
                telemetry.addLine("Intake and Flywheel");
                break;
            default:
                telemetry.addData("index", motorIndex);
                break;
        }

        if (robotContainer.gamepadEx1.dpadRight.wasJustPressed()) {
            motorIndex++;
        } else if (robotContainer.gamepadEx1.dpadLeft.wasJustPressed()) {
            motorIndex--;
        }

        if (robotContainer.gamepadEx1.dpadUp.wasJustPressed()) {
            motorPower += 0.1;
        } else if (robotContainer.gamepadEx1.dpadDown.wasJustPressed()) {
            motorPower -= 0.1;
        }

        if (robotContainer.gamepadEx1.cross.wasJustPressed()) {
            motorPower = 0;
        }

        telemetry.addData("motorPower", motorPower);

        telemetry.update();
        Thread.yield();
    }

    @Override
    public void stop(){
        robotContainer.stop();
    }
}