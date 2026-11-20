package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

public class Intake {
    private RobotContainer robotContainer;
    private BetterDcMotor intakeMotor;
    public double power = 0;

    public Intake(RobotContainer robotContainer, BetterDcMotor intakeMotor) {
        this.robotContainer = robotContainer;
        this.intakeMotor = intakeMotor;

        intakeMotor.start();
    }

    public void update(boolean teleop) {
        if (teleop) {
            setPower((robotContainer.driverController.rightTriggerRaw() + robotContainer.operatorController.rightTriggerRaw()) - (robotContainer.driverController.leftTriggerRaw() + robotContainer.operatorController.leftTriggerRaw()) / 2.0);
        }
    }

    public void setPower(double inputPower) {
        double current = getCurrent();
        power = current < 5 ? inputPower : (inputPower * 5) / current;
        power = Status.tongueDown ? power : power/2;
        intakeMotor.updateSetPower(power);
    }

    public void on() {
        setPower(1);
    }

    public void off(){
        setPower(0);
    }

    public double getVelocity() {
        return intakeMotor.getVelocity();
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
}
