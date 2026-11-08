package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.main.Constants;
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
//        RobotContainer.HardwareDevices.wingsServoLeader.updateSetPosition(Constants.Wings.MIN);
//        RobotContainer.HardwareDevices.wingsServoFollower.updateSetPosition(Constants.Wings.MIN + Constants.Wings.FOLLOWER_OFFSET);
//        RobotContainer.HardwareDevices.tongueServoLeader.updateSetPosition(Constants.Tongue.DOWN);
//        RobotContainer.HardwareDevices.tongueServoFollower.updateSetPosition(Constants.Tongue.UP);
//        RobotContainer.HardwareDevices.proboscisServo.updateSetPosition(Constants.Proboscis.UP);

    }

    @Override
    public void loop() {
        robotContainer.gamepadEx1.update();
        robotContainer.gamepadEx2.update();

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

        if (robotContainer.gamepadEx1.cross.isHeldFor(1.0)) {
            servoPose = 0.0;
        }

        if (robotContainer.gamepadEx1.cross.wasJustPressed()) {
            switch (servoIndex) {
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
            telemetry.addData("servoPose", servoPose);
            telemetry.update();
        }

//        robotContainer.wings.update(true);
        robotContainer.tongue.update(true);
//        robotContainer.proboscis.update(true);

        Thread.yield();
    }

    @Override
    public void stop(){
        robotContainer.stop();
    }
}