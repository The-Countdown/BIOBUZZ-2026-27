
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
public class ShootAfterOtherRobot extends OpMode {

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
    private final Pose start = poseFactory.of(58.855, 133.2623, 90);
    private final Pose path1 = poseFactory.of(46.8997, 130.9281, 89.3394);
    private final Pose path1Control1 = poseFactory.of(58.6828, 109.3656, 0);
    private final Pose path1Control2 = poseFactory.of(46.653, 109.529, 0);
    private final Pose point2Start = poseFactory.of(46.8997, 130.9281, 90);
    private final Pose point2 = poseFactory.of(12.6251, 94.8396, 0);
    private final Pose point2Control1 = poseFactory.of(47.6303, 105.974, 0);
    private final Pose point2Control2 = poseFactory.of(51.6107, 94.7824, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1Control2, path1).tangent();
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2).linear(point2Start, point2);
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