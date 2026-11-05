package org.firstinspires.ftc.teamcode.hardware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LinkedServos {
    private BetterServo masterServo;
    private List<BetterServo> slaveServo;
    private BetterCRServo masterCRServo;
    private List<BetterCRServo> slaveCRServo;

    public LinkedServos(BetterServo masterServo, BetterServo[] slaveServo) {
        this.masterServo = masterServo;
        this.slaveServo = new ArrayList<>(Arrays.asList(slaveServo));
    }

    public LinkedServos(BetterServo masterServo, BetterServo slaveServo) {
        this.masterServo = masterServo;
        this.slaveServo = new ArrayList<>();
        this.slaveServo.add(slaveServo);
    }

    public LinkedServos(BetterCRServo masterServo, BetterCRServo[] slaveServo) {
        this.masterCRServo = masterServo;
        this.slaveCRServo = new ArrayList<>(Arrays.asList(slaveServo));
    }

    public LinkedServos(BetterCRServo masterServo, BetterCRServo slaveServo) {
        this.masterCRServo = masterServo;
        this.slaveCRServo = new ArrayList<>();
        this.slaveCRServo.add(slaveServo);
    }

    public void setPosition(double position, double slaveServoOffset) {
        masterServo.updateSetPosition(position);
        for (BetterServo slaveServo : slaveServo) {
            slaveServo.updateSetPosition(position + slaveServoOffset);
        }
    }

    public void setMirroredPositions(double position1, double position2) {
        masterServo.updateSetPosition(position1);
        for (BetterServo slaveServo : slaveServo) {
            slaveServo.updateSetPosition(position2);
        }
    }

    public void setPositionDegrees(double degree) {
        masterServo.updateSetPositionDegrees(degree);
        for (BetterServo slaveServo : slaveServo) {
            slaveServo.updateSetPositionDegrees(degree);
        }
    }

    //set power 1 -1
    public void setPower(double power) {
        masterCRServo.updateSetPower(power);
        for (BetterCRServo slaveCRServo : slaveCRServo) {
            slaveCRServo.updateSetPower(power);
        }
    }
    public double getPosition() {
        return masterServo.getPosition();
    }

    public double getPositionDegrees() {
        return masterServo.getPositionDegrees();
    }
}
