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
            if (robotContainer.gamepadEx1.rightBumper.isHeld() || robotContainer.gamepadEx2.rightBumper.isHeld()) {
                robotContainer.proboscis.setPosition(Constants.Proboscis.OPEN);
                Status.proboscisDown = true;
            } else {
                robotContainer.proboscis.setPosition(Constants.Proboscis.CLOSED);
                Status.proboscisDown = false;
            }
        }
    }

    // This is for manual control
    public void setPosition(double position) {
        proboscisServo.updateSetPosition(HelperFunctions.clamp(position, Constants.Proboscis.OPEN, Constants.Proboscis.CLOSED));
    }

    public void open(){
        robotContainer.proboscis.setPosition(Constants.Proboscis.OPEN);
    }

    public void close() { robotContainer.proboscis.setPosition(Constants.Proboscis.CLOSED); }

}
