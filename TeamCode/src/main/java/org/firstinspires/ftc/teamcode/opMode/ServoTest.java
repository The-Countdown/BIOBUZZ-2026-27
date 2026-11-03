package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ServoTest", group = "TeleOp")
//@Disabled
public class ServoTest extends OpMode {
    private RobotContainer robotContainer;
    private int servoIndex = -1;
    private double servoPose = 0.0;

    @Override
    public void init() {
        try {
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robotContainer.init();
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
        switch(servoIndex) {
            case -1:
                telemetry.addLine("stopped");
                break;
            case 0:
                RobotContainer.HardwareDevices.wingsServoLeader.updateSetPosition(servoPose + RobotContainer.HardwareDevices.wingsServoLeader.getPosition());
                telemetry.addData("wingsServoLeader", RobotContainer.HardwareDevices.wingsServoLeader.getPosition());
                break;
            case 1:
                RobotContainer.HardwareDevices.wingsServoFollower.updateSetPosition(servoPose + RobotContainer.HardwareDevices.wingsServoFollower.getPosition());
                telemetry.addData("wingsServoFollower", RobotContainer.HardwareDevices.wingsServoFollower.getPosition());
                break;
            case 2:
                RobotContainer.HardwareDevices.tongueServoLeader.updateSetPosition(servoPose + RobotContainer.HardwareDevices.tongueServoLeader.getPosition());
                telemetry.addData("tongueServoLeader", RobotContainer.HardwareDevices.tongueServoLeader.getPosition());
                break;
            case 3:
                RobotContainer.HardwareDevices.tongueServoFollower.updateSetPosition(servoPose + RobotContainer.HardwareDevices.tongueServoFollower.getPosition());
                telemetry.addData("tongueServoFollower", RobotContainer.HardwareDevices.tongueServoFollower.getPosition());
                break;
            case 4:
                RobotContainer.HardwareDevices.proboscisServo.updateSetPosition(servoPose + RobotContainer.HardwareDevices.proboscisServo.getPosition());
                telemetry.addData("proboscisServo", RobotContainer.HardwareDevices.proboscisServo.getPosition());
                break;

            default:
                telemetry.addData("index", servoIndex);
                break;
        }

        if (robotContainer.gamepadEx1.dpadRight.wasJustPressed()) {
            servoIndex++;
        } else if (robotContainer.gamepadEx1.dpadLeft.wasJustPressed()) {
            servoIndex--;
        }

        if (robotContainer.gamepadEx1.dpadUp.wasJustPressed()) {
            servoPose += 0.025;
        } else if (robotContainer.gamepadEx1.dpadDown.wasJustPressed()) {
            servoPose -= 0.025;
        }

        if (robotContainer.gamepadEx1.cross.wasJustPressed()) {
            servoPose = 0;
        }

        telemetry.addData("servoPose", servoPose);

        telemetry.update();
        Thread.yield();
    }
}