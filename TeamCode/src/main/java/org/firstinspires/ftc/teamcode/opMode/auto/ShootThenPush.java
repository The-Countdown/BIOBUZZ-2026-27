
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

@Autonomous
@Configurable // Allows you to configure in panels
public class ShootThenPush extends OpMode {

    // Objects that need initialising
    private Follower follower;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private final PoseFactory poseFactory = PoseFactory.degrees(); // Makes the creation of poses easier
    private RobotContainer robotContainer; // Robot container sets up all necessary hardware and software so that it can be immediately used in any code across the codebase without having to set it up in every file

    // File Variables
    boolean holding = false;
    ElapsedTime pathTimer = new ElapsedTime();
    int pathNumber = 1;

    // Poses
    private final Pose start = poseFactory.of(55.2863, 8.8922, 90);
    private final Pose path1 = poseFactory.of(10.6772, 47.2415, 180);
    private final Pose point2 = poseFactory.of(21.35, 60.5861, -3.1436);
    private final Pose point2Control1 = poseFactory.of(30.131, 47.378, 0);
    private final Pose point2Control2 = poseFactory.of(39.456, 55.1279, 0);
    private final Pose point2Control3 = poseFactory.of(25.6931, 60.3476, 0);
    private final Pose point3 = poseFactory.of(10.1281, 99.3718, -88.4246);
    private final Pose point3Control1 = poseFactory.of(6.7806, 59.9393, 0);
    private final Pose point3Control2 = poseFactory.of(11.1651, 61.666, 0);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2Control2, point2Control3, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3).reverseTangent();
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

    Command intakeOnOff = Command.build()
            .setStart(() -> {
                robotContainer.intake.on();
                robotContainer.tongue.down();
            })
            .setExecute(() -> {

            })
            .setEnd((endCondition) -> {
                robotContainer.intake.off();
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

    private Command autoRoutine() {
        return sequential( // list of *commands* to run in a row when executed
                turretOnOff,
                proboscisDown,
                follow(follower, path1()),
                intakeOnOff,
                follow(follower, path2()),
                proboscisUp,
                follow(follower, path1())
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
        schedule(autoRoutine()); // This will schedule the auto routine that you had previously created to run in sequence
    }

    @Override
    public void loop() { // Runs constantly while the opmode is active
        follower.update(); // Updates that pathing follower to make the robot move in auto
        Scheduler.execute(); // Checks which command should be running right now in the list of your commands that you configured in the start function

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