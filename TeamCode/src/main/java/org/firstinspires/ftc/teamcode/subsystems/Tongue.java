package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;


public class Tongue extends RobotContainer. HardwareDevices{
    public BetterServo tongueServo;

    public Constants constants;

    public final RobotContainer robotContainer;

    public Status status;

    public HelperFunctions helperFunctions;

    public Tongue(RobotContainer robotContainer, BetterServo tongueServo) {
        this.robotContainer = robotContainer;
    }

    public void update(boolean teleop){

        if (teleop) {
            if (robotContainer.gamepadEx1.triangle.wasJustPressed()) {
                robotContainer.tongue.tongueServo.updateSetPosition(Constants.Tongue.UP);
                Status.tongueDown = true;

            } else {
                robotContainer.tongue.tongueServo.updateSetPosition(Constants.Tongue.DOWN);
                Status.tongueDown = false;
            }
        }
    }

    public void setPosition(double position){
        robotContainer.tongue.tongueServo.updateSetPosition(HelperFunctions.clamp(position, Constants.Tongue.UP, Constants.Tongue.DOWN));
    }
    public void up(){robotContainer.tongue.tongueServo.updateSetPosition(Constants.Tongue.UP);}
    public void down(){robotContainer.tongue.tongueServo.updateSetPosition(Constants.Tongue.DOWN);}
}
