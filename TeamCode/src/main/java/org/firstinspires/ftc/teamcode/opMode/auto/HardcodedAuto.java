// Autonomous Package
package org.firstinspires.ftc.teamcode.opMode.auto;

// Autonomous Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Pedro Pathing Imports
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.follower.*;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroPathingConstants;

// Panels Imports
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

// Our Imports
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@Autonomous
@Configurable // Allows you to configure in panels
public class HardcodedAuto extends OpMode {

    // Objects that need initialising
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private RobotContainer robotContainer; // Robot container sets up all necessary hardware and software so that it can be immediately used in any code across the codebase without having to set it up in every file

    // File Variables
    boolean end = false;
    ElapsedTime pathTimer = new ElapsedTime();
    int pathNumber = 1;


    @Override
    public void init() { // runs when the init is pressed on the driver station
        try { // this prevents it from crashing because of an issue with robot container
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        robotContainer.init();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry(); // creates a new instance of panels
        robotContainer.init();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Alliance", Status.alliance);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() { // Runs when the start button is pressed on the driver station
        robotContainer.start(this, false);
        pathTimer.reset();
    }

    public void strafe(boolean left, double power, double time){
        power = left ? power : -power;
        pathTimer.reset();
        while (pathTimer.milliseconds() < time) {
            RobotContainer.HardwareDevices.leftFront.setPower(-power);
            RobotContainer.HardwareDevices.leftBack.setPower(-power);
            RobotContainer.HardwareDevices.rightFront.setPower(power);
            RobotContainer.HardwareDevices.rightBack.setPower(power);
        }
    }

    public void drive(double power, double time){
        while (pathTimer.milliseconds() < time) {
            RobotContainer.HardwareDevices.leftFront.setPower(power);
            RobotContainer.HardwareDevices.leftBack.setPower(power);
            RobotContainer.HardwareDevices.rightFront.setPower(power);
            RobotContainer.HardwareDevices.rightBack.setPower(power);
        }
    }

    public void pause(double time){
        while (pathTimer.milliseconds() < time) {

        }
    }

    public void off(){
        RobotContainer.HardwareDevices.leftFront.setPower(0);
        RobotContainer.HardwareDevices.leftBack.setPower(0);
        RobotContainer.HardwareDevices.rightFront.setPower(0);
        RobotContainer.HardwareDevices.rightBack.setPower(0);
    }

    public void end() {
        end = true;
    }

    @Override
    public void loop() { // Runs constantly while the opmode is active
        if (end) return;

        robotContainer.update(false); // This will update all subsystems of the robot that need functions running in loop such as tracking the goal

            robotContainer.wings.setPosition(Constants.Wings.EXTENDED);
            Status.wingsDown = true;
            pause(0);
            strafe(true, 0.5, 350);
            drive(0.4, 1750);
            off();
            end();

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path Timer", pathTimer.milliseconds());
        panelsTelemetry.debug("Current Pose", Status.currentPose);
        panelsTelemetry.update(telemetry);

        telemetry.update();
    }
}