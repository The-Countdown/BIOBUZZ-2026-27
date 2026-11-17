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
public class Comp_2 extends OpMode {

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
    private final Pose start = poseFactory.of(61.5455, 9.0733, 180);
    private final Pose path1 = poseFactory.of(8.7737, 9.1669, 180);
    private final Pose point2 = poseFactory.of(61.5235, 9.1571, 179.9894);
    private final Pose point3 = poseFactory.of(19.0289, 46.4956, 85.6574);
    private final Pose point3Control1 = poseFactory.of(16.3645, 11.4094, 0);
    private final Pose point4 = poseFactory.of(9.8114, 46.425, -179.5616);
    private final Pose point5 = poseFactory.of(18.9583, 46.4364, -179.9285);
    private final Pose point6 = poseFactory.of(9.6909, 95.0851, -90.7818);
    private final Pose point6Control1 = poseFactory.of(15.6, 60.6195, 0);
    private final Pose point6Control2 = poseFactory.of(9.2168, 60.3423, 0);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).tangent();
    }

    public Path path5() {
        return Paths.line(point4, point5).reverseTangent();
    }

    public Path path6() {
        return Paths.curve(point5, point6Control1, point6Control2, point6).reverseTangent();
    }

    // Action Creation
    Command shoot() {
        return instant(() ->
            robotContainer.flywheel.shootSequence()
        );
    }
    Command intakeOn() {
        return instant(() -> {
            robotContainer.intake.on();
            robotContainer.tongue.down();
        });
    }

    Command intakeOff() {
        return instant(() -> {
            robotContainer.intake.off();
        });
    }

    Command intake() {
        return sequential(
                instant(() -> {
                    robotContainer.intake.on();
                    robotContainer.tongue.down();
                }),

                waitMs(1000),

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
                follow(follower, path1()),
                intake(),
                follow(follower, path2()),
                shoot(),
                follow(follower, path3()),
                proboscisDown(),
                follow(follower, path4()),
                intake(),
                follow(follower, path5()),
                proboscisUp(),
                follow(follower, path6())
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
        robotContainer.positionProvider.updatePose(new Pose2D(DistanceUnit.INCH, follower.pose().x(), follower.pose().y(), AngleUnit.DEGREES, follower.pose().heading()));

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