package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.hardware.BetterServo;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

public class Proboscis extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    public Proboscis(RobotContainer robotContainer, BetterServo proboscisServo) {
        this.robotContainer = robotContainer;
    }

    public void update(boolean teleop) {
        if (teleop) {
            if (robotContainer.gamepadEx1.leftBumper.isHeld() || robotContainer.gamepadEx2.leftBumper.isHeld()) {
                down();
            } else if (Status.proboscisDown) {
                up();
            }
        }
    }

    // This is for manual control
    public void setPosition(double position) {
        proboscisServo.updateSetPosition(HelperFunctions.clamp(position, Constants.Proboscis.UP, Constants.Proboscis.DOWN));
//        Status.proboscisDown = false;
    }

    public void down(){
        setPosition(Constants.Proboscis.DOWN);
        Status.proboscisDown = true;
    }

    public void up() {
        setPosition(Constants.Proboscis.UP);
    }

}
