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
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.*;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PedroPathingConstants;

// Panels Imports
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.util.ElapsedTime;

// Our Imports
import org.firstinspires.ftc.teamcode.main.RobotContainer;
import org.firstinspires.ftc.teamcode.main.Status;

@Autonomous
@Configurable // Allows you to configure in panels
public class DontPayAttentionToTheManBehindTheDoor extends OpMode {

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
    private final Pose start = poseFactory.of(58.1466, 133.579, 90);
    private final Pose path1 = poseFactory.of(46.519, 135.1037, 90);
    private final Pose path1Control1 = poseFactory.of(57.8917, 104.2911, 0);
    private final Pose path1Control2 = poseFactory.of(45.2134, 109.4294, 0);
    private final Pose point2 = poseFactory.of(9.7577, 117.0937, 92.6627);
    private final Pose point2Control1 = poseFactory.of(46.8488, 110.4464, 0);
    private final Pose point2Control2 = poseFactory.of(56.481, 107.6408, 0);
    private final Pose point2Control3 = poseFactory.of(22.5037, 139.4101, 0);
    private final Pose point2Control4 = poseFactory.of(8.6227, 141.5, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1Control2, path1).constant(path1);
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2Control2, point2Control3, point2Control4, point2).reverseTangent();
    }

    // Action Creation
    Command shoot() {
        return instant(() -> {
            waitMs(4000);
            robotContainer.flywheel.shootSequence();
        });
    }

    Command intakeOn() {
        return instant(() -> {
            robotContainer.intake.on();
        });
    }

    Command intakeOff() {
        return instant(() -> {
            waitMs(1000);
            robotContainer.intake.off();
        });
    }

    Command intake() {
        return sequential(
                instant(() -> {
                    robotContainer.intake.on();
                    robotContainer.tongue.down();
                }),

                waitMs(1500),

                instant(() -> {
                    robotContainer.intake.off();
                })
        );
    }

    Command proboscisDown() {
        return instant(() -> {
            robotContainer.proboscis.down();
        });
    }

    Command proboscisUp() {
        return instant(() -> {
            robotContainer.proboscis.up();
        });
    }

    // Path methods
    private Command autoRoutine() {
        return sequential( // list of *commands* to run in a row when executed
                shoot(),
                waitMs(Constants.Flywheel.SPINUP_MS + Constants.Flywheel.SHOOT_MS + Constants.Tongue.POSITION_CHANGE_MS * 2),
                intakeOn(),
                proboscisDown(),
                follow(follower, path1()),
                intakeOff(),
                proboscisUp(),
                follow(follower, path2())
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
        follower.setPose(start);
        robotContainer.start(this, false);
        schedule(autoRoutine()); // This will schedule the auto routine that you had previously created to run in sequence
    }

    @Override
    public void loop() { // Runs constantly while the opmode is active
        follower.update(); // Updates that pathing follower to make the robot move in auto
        Scheduler.execute(); // Checks which command should be running right now in the list of your commands that you configured in the start function

        robotContainer.update(false); // This will update all subsystems of the robot that need functions running in loop such as tracking the goal
        robotContainer.positionProvider.updatePose(new Pose2D(DistanceUnit.INCH, follower.pose().x() - 1.5 - 72, follower.pose().y() - 20.5 - 72, AngleUnit.DEGREES, Math.toDegrees(follower.pose().heading()) - 180));

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

    @Override
    public void stop() {
        blackboard.put("Robot Pose", Status.currentPose);
    }
}