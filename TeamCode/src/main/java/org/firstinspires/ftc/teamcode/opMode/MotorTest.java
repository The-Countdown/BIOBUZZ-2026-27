package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.TurretPDF;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MotorTest", group = "TeleOp")
//@Disabled
public class MotorTest extends OpMode {
    private RobotContainer robotContainer;
    private int motorIndex = -1;
    private double motorPower = 0.0;
    private TurretPDF turretPDF;

    public void defaultMotor(){
        RobotContainer.HardwareDevices.rightFront.setPower(0);
        RobotContainer.HardwareDevices.leftFront.setPower(0);
        RobotContainer.HardwareDevices.rightBack.setPower(0);
        RobotContainer.HardwareDevices.leftBack.setPower(0);
        RobotContainer.HardwareDevices.intakeMotor.setPower(0);
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

        turretPDF = new TurretPDF();
        RobotContainer.HardwareDevices.flywheelMotorFollower.start();
        RobotContainer.HardwareDevices.flywheelMotorLeader.start();
        RobotContainer.HardwareDevices.intakeMotor.start();
        RobotContainer.HardwareDevices.turretMotor.start();


    }

    @Override
    public void start() {
        Status.opModeIsActive = true;
        robotContainer.start(this, true);
        robotContainer.registerLoopTimer("teleop");
    }

    @Override
    public void loop() {
        robotContainer.refreshData();
        robotContainer.driverController.update();
        robotContainer.operatorController.update();
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
                if (motorPower != 0) {
                    RobotContainer.HardwareDevices.turretMotor.setPower(turretPDF.calculate(motorPower * 10 * 28, RobotContainer.HardwareDevices.turretMotor.getCurrentPosition(), 0));
                }
                telemetry.addLine("turretMotor");
                telemetry.addData("Position: ", RobotContainer.HardwareDevices.turretMotor.getCurrentPosition());
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

        if (robotContainer.driverController.dpadRight.wasJustPressed()) {
            motorIndex++;
        } else if (robotContainer.driverController.dpadLeft.wasJustPressed()) {
            motorIndex--;
        }

        if (robotContainer.driverController.dpadUp.wasJustPressed()) {
            motorPower += 0.1;
        } else if (robotContainer.driverController.dpadDown.wasJustPressed()) {
            motorPower -= 0.1;
        }

        if (robotContainer.driverController.cross.wasJustPressed()) {
            motorPower = 0;
        }

        telemetry.addData("motorPower", motorPower);
        robotContainer.updateLoopTime("teleop");
        telemetry.addData("Loop Time", robotContainer.getLoopTime("teleop"));

        telemetry.update();
        Thread.yield();
    }

    @Override
    public void stop(){
        robotContainer.stop();
    }
}