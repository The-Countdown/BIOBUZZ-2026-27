package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.hardware.LinkedServos;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;


public class Tongue extends RobotContainer.HardwareDevices{
    public LinkedServos tongueServos;

    public Constants constants;

    public final RobotContainer robotContainer;

    public Status status;
    public Tongue(RobotContainer robotContainer, LinkedServos tongueServos) {
        this.robotContainer = robotContainer;
    }

    public void update(boolean teleop){

        if (teleop) {
            if (robotContainer.gamepadEx1.triangle.wasJustPressed()) {
                robotContainer.tongue.tongueServos.setPosition(Constants.Tongue.UP);
                Status.tongueDown = true;

            } else {
                robotContainer.tongue.tongueServos.setPosition(Constants.Tongue.DOWN);
                Status.tongueDown = false;
            }
        }
    }

    public void setPosition(double position){
        robotContainer.tongue.tongueServos.setPosition(HelperFunctions.clamp(position, Constants.Tongue.UP, Constants.Tongue.DOWN));
    }
    public void up(){robotContainer.tongue.tongueServos.setPosition(Constants.Tongue.UP);}
    public void down(){robotContainer.tongue.tongueServos.setPosition(Constants.Tongue.DOWN);}
}
