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
            setPower((robotContainer.driverController.rightTriggerRaw() + robotContainer.operatorController.rightTriggerRaw()) - (robotContainer.driverController.leftTriggerRaw() + robotContainer.operatorController.leftTriggerRaw()));
        } else {
            setPower(power);
        }
    }

    public void setPower(double power) {
//        double current = getCurrent();
//        this.power = current < 5 ? power : (power * 5) / current;
        this.power = Status.tongueDown ? power : power/2;
        intakeMotor.setPower(power);
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
