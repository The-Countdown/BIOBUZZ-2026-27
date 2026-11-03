package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.BetterDcMotor;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

public class Intake {
    private RobotContainer robotContainer;
    private BetterDcMotor intakeMotor;
    public double power = 0;
    public double currentAmps = 0;
    public double newPower;

    public Intake(RobotContainer robotContainer, BetterDcMotor intakeMotor) {
        this.robotContainer = robotContainer;
        this.intakeMotor = intakeMotor;
        this.newPower = 0;

        intakeMotor.start();
    }

    public void update(boolean teleop) {
        if (teleop) {
            setPower((robotContainer.gamepadEx1.rightTriggerRaw() + robotContainer.gamepadEx2.rightTriggerRaw()) - (robotContainer.gamepadEx1.leftTriggerRaw() + robotContainer.gamepadEx2.leftTriggerRaw()));
        } else {
            setPower(power);
        }
    }

    public void setPower(double power) {
        this.power = power;
        double current = getCurrent();
        if (current > 5 && !Status.tongueDown) {
            this.newPower = Math.max(power - (power * ((current - 5) * 0.1)), 0);
            intakeMotor.setPower(newPower);
        } else {
            intakeMotor.setPower(power);
        }
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

    public double getNewPower(){
        return newPower;
    }
}
