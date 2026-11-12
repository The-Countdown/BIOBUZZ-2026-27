// Autonomous Package
package org.firstinspires.ftc.teamcode.opMode.auto;

// Autonomous Imports
import com.pedropathing.api.Paths;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Pedro Pathing Imports
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.api.Paths.*;
import static org.firstinspires.ftc.teamcode.main.Status.alliance;
import static org.firstinspires.ftc.teamcode.main.Status.flywheelToggle;
import static java.lang.Thread.sleep;

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
import com.qualcomm.hardware.limelightvision.Limelight3A;

@Autonomous
@Configurable // Allows you to configure in panels
public class Noobular extends OpMode {

    // Objects that need initialising
    private Follower follower;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private final PoseFactory poseFactory = PoseFactory.degrees(); // Makes the creation of poses easier
    private RobotContainer robotContainer; // Robot container sets up all necessary hardware and software so that it can be immediately used in any code across the codebase without having to set it up in every file

    // File Variables
    boolean holding = false;
    ElapsedTime pathTimer = new ElapsedTime();
    int pathNumber = 1;

    public static Limelight3A limelight;

    // Poses
    private final Pose start = poseFactory.of(55.1078, 8.3569, 90);
    private final Pose point1 = poseFactory.of(8.4818, 8.5693, 179.739);
    private final Pose point2Start = poseFactory.of(8.4818, 8.5693, 180);
    private final Pose point2 = poseFactory.of(55.1751, 8.8685, 90);
    private final Pose point2Control1 = poseFactory.of(55.0152, 55.5438, 0);

    public Path path1() {
        return Paths.line(start, point1).tangent();
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }


    private final Pose start2 = poseFactory.of(55.18, 8.87, 90);
    private final Pose path3 = poseFactory.of(47.0782, 130.5712, 89.8778);
    private final Pose path3Control1 = poseFactory.of(56.2019, 66.2052, 0);
    private final Pose path3Control2 = poseFactory.of(66.6805, 58.2095, 0);
    private final Pose path3Control3 = poseFactory.of(63.4107, 112.2573, 0);
    private final Pose path3Control4 = poseFactory.of(47.0458, 115.3768, 0);
    private final Pose point4 = poseFactory.of(47.3048, 119.0366, 91.1256);
    private final Pose point5Start = poseFactory.of(47.3048, 119.0366, 90);
    private final Pose point5 = poseFactory.of(47.1546, 118.7439, 270);

    public Path path3() {
        return Paths.curve(start2, path3Control1, path3Control2, path3Control3, path3Control4, path3).tangent();
    }

    public Path path4() {
        return Paths.line(path3, point4).reverseTangent();
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    private final Pose start3 = poseFactory.of(47.15, 118.74, 270);
    private final Pose path6 = poseFactory.of(10.3921, 46.2949, 180);
    private final Pose path6Control1 = poseFactory.of(17.4632, 92.4368, 0);
    private final Pose path6Control2 = poseFactory.of(37.2515, 40.9704, 0);
    private final Pose path6Control3 = poseFactory.of(20.6661, 45.8093, 0);
    private final Pose point7Start = poseFactory.of(10.3921, 46.2949, 180);
    private final Pose point7 = poseFactory.of(47.15, 118.74, 270);
    private final Pose point7Control1 = poseFactory.of(21.187, 45.7193, 0);
    private final Pose point7Control2 = poseFactory.of(37.4038, 40.9973, 0);
    private final Pose point7Control3 = poseFactory.of(17.578, 92.022, 0);
    private final Pose point8 = poseFactory.of(11.2896, 116.7166, 3.2294);

    public Path path6() {
        return Paths.curve(start3, path6Control1, path6Control2, path6Control3, path6).linear(start3, path6);
    }

    public Path path7() {
        return Paths.curve(point7Start, point7Control1, point7Control2, point7Control3, point7).linear(point7Start, point7);
    }

    public Path path8() {
        return Paths.line(point7, point8).reverseTangent();
    }

    private final Pose start4 = poseFactory.of(47.15, 118.74, 270);
    private final Pose path9 = poseFactory.of(10.3921, 46.1164, 180);
    private final Pose path9Control1 = poseFactory.of(13.712, 101.4478, 0);
    private final Pose path9Control2 = poseFactory.of(34.3195, 45.8985, 0);
    private final Pose path9Control3 = poseFactory.of(29.8922, 45.436, 0);
    private final Pose point10 = poseFactory.of(11.5352, 95.9021, -87.0516);
    private final Pose point10Control1 = poseFactory.of(30.0039, 45.5432, 0);
    private final Pose point10Control2 = poseFactory.of(34.4614, 45.8394, 0);
    private final Pose point10Control3 = poseFactory.of(12.8881, 69.6338, 0);

    public Path path9() {
        return Paths.curve(start4, path9Control1, path9Control2, path9Control3, path9).linear(start4, path9);
    }

    public Path path10() {
        return Paths.curve(path9, point10Control1, point10Control2, point10Control3, point10).reverseTangent();
    }

    // Action Creation
    Command turretOnOff = Command.build()
            .setStart(() -> {
                // executed on start of command
                Status.flywheelToggle = true;
                robotContainer.delayedActionManager.schedule(() -> robotContainer.tongue.up(), 1000);
            })
            .setExecute(() -> {
                // executed on every loop while command is running
            })
            .setEnd(endCondition -> {
                // executed at the end of the command
                Status.flywheelToggle = false;
            });

    Command intakeOn = Command.build()
            .setStart(() -> {
                robotContainer.intake.on();
                robotContainer.tongue.down();
            })
            .setExecute(() -> {

            })
            .setEnd((endCondition) -> {

            });

    Command intakeOff = Command.build()
            .setStart(() -> {
                robotContainer.intake.off();
            })
            .setExecute(() -> {

            })
            .setEnd((endCondition) -> {

            });

    Command proboscisUp = Command.build()
            .setStart(() -> {
                robotContainer.proboscis.up();
            })
            .setExecute(() -> {

            })
            .setEnd((endCondition) -> {

            });

    Command proboscisDown = Command.build()
            .setStart(() -> {
                robotContainer.proboscis.down();
            })
            .setExecute(() -> {

            })
            .setEnd((endCondition) -> {

            });

    private Command autoRoutine1() {
        return sequential( // list of *commands* to run in a row when executed
                turretOnOff,
                intakeOn,
                follow(follower, path1()),
                intakeOff,
                follow(follower, path2())
        );
    }

    private Command autoRoutine2() {
        return sequential(
                intakeOn,
                turretOnOff,
                follow(follower,path3()),
                turretOnOff,
                follow(follower, path4()),
                intakeOff,
                follow(follower, path5())
        );
    }

    private Command autoRoutine3() {
        return sequential(
                follow(follower,path3()),
                intakeOn,
                turretOnOff,
                follow(follower, path4()),
                intakeOff,
                follow(follower, path5())
        );
    }

    private Command autoRoutine4() {
        return sequential(
                proboscisDown,
                follow(follower, path6()),
                intakeOn,
                follow(follower, path7()),
                turretOnOff,
                proboscisUp,
                intakeOff,
                follow(follower,path8())
        );
    }

    private Command autoRoutine5() {
        return sequential(
                proboscisDown,
                follow(follower, path9()),
                intakeOn,
                follow(follower, path10()),
                intakeOff,
                proboscisUp

        );
    }

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
        Scheduler.reset(); // resets the command scheduler for the ivy follower

        follower = PedroPathingConstants.create(hardwareMap);
        follower.setPose(start); // Sets the starting pose for the robot
        follower.update();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Alliance", Status.alliance);
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void start() { // Runs when the start button is pressed on the driver station
        robotContainer.start(this, false);
        limelight.start();
    }

    @Override
    public void loop() { // Runs constantly while the opmode is active
        follower.update(); // Updates that pathing follower to make the robot move in auto
        schedule(autoRoutine1());

        if (limelight.pipelineSwitch(8)) {
            schedule(autoRoutine2());
            schedule(autoRoutine4());
        } else {
            schedule(autoRoutine3());
            schedule(autoRoutine5());
        }

        if (limelight.pipelineSwitch(9)) {
            schedule(autoRoutine2());
            schedule(autoRoutine4());
        } else {
            schedule(autoRoutine3());
            schedule(autoRoutine5());
        }

        robotContainer.update(false); // This will update all subsystems of the robot that need functions running in loop such as tracking the goal

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Turret Target", robotContainer.turret.getTargetPosition());
        panelsTelemetry.debug("Turret Target Degrees", robotContainer.turret.getTargetPositionDegrees());
        panelsTelemetry.debug("Path Timer", pathTimer.milliseconds());
        panelsTelemetry.debug("Holding", holding);
        panelsTelemetry.debug("Path Number", pathNumber);
        panelsTelemetry.debug("X", follower.pose().x());
        panelsTelemetry.debug("Y", follower.pose().y());
        panelsTelemetry.debug("Heading", follower.pose().heading());
        panelsTelemetry.debug("Current Pose", Status.currentPose);
        panelsTelemetry.update(telemetry);

        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}