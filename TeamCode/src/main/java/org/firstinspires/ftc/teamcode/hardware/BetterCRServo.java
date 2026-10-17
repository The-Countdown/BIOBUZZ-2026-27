package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.teamcode.util.HelperFunctions;

// Warning the Axon servos in servo mode do not work.
public class BetterCRServo {
    private CRServoImplEx servo;

    private long lastTime;

    private double lastPower = 0;
    private double power = 0;

    private int minTimeBetweenUpdates = 0;
    private double minPowerDiffBetweenUpdates = 0;

    public BetterCRServo(CRServoImplEx servo, int minTimeBetweenUpdates, double minPowerDiffBetweenUpdates) {
        this.servo = servo;
        this.minTimeBetweenUpdates = HelperFunctions.getRandomWithin(minTimeBetweenUpdates, 0.5);
        this.minPowerDiffBetweenUpdates = minPowerDiffBetweenUpdates;
    }

    public void updateSetPower(double power) {
        this.power = power;
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastTime >= minTimeBetweenUpdates &&  Math.abs(power - lastPower) > minPowerDiffBetweenUpdates) {
            servo.setPower(power);
            lastTime = System.currentTimeMillis();
            this.lastPower = power;
        }
    }

    public void setDirection(CRServoImplEx.Direction direction) {
        servo.setDirection(direction);
    }
    
    public double getPower() {
        return this.power;
    }
}
