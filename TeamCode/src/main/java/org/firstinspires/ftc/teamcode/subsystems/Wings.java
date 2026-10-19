package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;
import org.firstinspires.ftc.teamcode.util.HelperFunctions;
import org.firstinspires.ftc.teamcode.hardware.LinkedServos;

public class Wings extends RobotContainer.HardwareDevices {
    private final RobotContainer robotContainer;
    private final LinkedServos wingServos;
    public double targetPosition;

    public Wings(RobotContainer robotContainer, LinkedServos brakeServos) {
        this.robotContainer = robotContainer;
        this.targetPosition = Constants.Wings.MIN;
        this.wingServos = brakeServos;
    }

    public void update(boolean teleop) {
        if (teleop) {

            // Actions to happen when wingsDown boolean was just changed
            if (robotContainer.gamepadEx1.dpadDown.wasJustPressed()) {
                Status.wingsDown = !Status.wingsDown;
                if (Status.wingsDown) {
                    robotContainer.delayedActionManager.schedule(() -> targetPosition = Constants.Wings.EXTENDED, 250);
                    robotContainer.delayedActionManager.schedule(() -> robotContainer.wings.setPosition(targetPosition), 250);
                }

            // Actions to run in a loop
            } else if (Status.parking && !Status.wingsDown) {
                targetPosition = Constants.Wings.PARK;
                robotContainer.wings.setPosition(Constants.Wings.PARK);
            } else if (!Status.wingsDown) {
                targetPosition = Constants.Wings.MIN;
                robotContainer.wings.setPosition(Constants.Wings.MIN);
            }
        }
    }

    // This is for manual control
    public void setPosition(double position) {
        wingServos.setPosition(HelperFunctions.clamp(position, Constants.Wings.MIN, Constants.Wings.MAX));
    }

    public double getPosition() {
        return wingServos.getPosition();
    }

    public double getPositionDegrees() {
        return wingServos.getPositionDegrees();
    }
}
