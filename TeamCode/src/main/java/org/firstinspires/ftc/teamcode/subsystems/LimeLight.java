package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

public class LimeLight extends RobotContainer.HardwareDevices {

    private Limelight3A limelight;

    public Constants constants;
    private RobotContainer robotContainer;

    private LLResult result;
    public LimeLight(RobotContainer robotContainer, Limelight3A limelight) {
            this.robotContainer = robotContainer;
            this.limelight = limelight;
    }

    public void start(boolean allianceRED) {
        if (allianceRED) {
            limelight.pipelineSwitch(8);
        } else {
            limelight.pipelineSwitch(9);
        }

    }

    public boolean latestResult() {
        if (result == null) {
            return false;
        } else {
            return true;
        }
    }

    public void update(boolean Autonomous) {
        if (Autonomous){
            limelight.getLatestResult();
        }
    }

}
