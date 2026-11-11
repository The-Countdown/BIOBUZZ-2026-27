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
    private final Pose path1 = poseFactory.of(9.2497, 8.1639, -179.7589);
    private final Pose point2Start = poseFactory.of(9.2497, 8.1639, 180);
    private final Pose point2 = poseFactory.of(23.332, 17.8777, 90);
    private final Pose point2Control1 = poseFactory.of(12.2949, 19.5139, 0);
    private final Pose point3 = poseFactory.of(46.3906, 124.1394, 55.0967);
    private final Pose point3Control1 = poseFactory.of(13.381, 76.827, 0);
    private final Pose point4 = poseFactory.of(46.4471, 130.5162, 89.4921);
    private final Pose point5 = poseFactory.of(46.348, 123.8845, 89.1437);
    private final Pose point6Start = poseFactory.of(46.348, 123.8845, 90);
    private final Pose point6 = poseFactory.of(17.6811, 46.2623, 180);
    private final Pose point6Control1 = poseFactory.of(26.5018, 98.6829, 0);
    private final Pose point7 = poseFactory.of(10.6126, 46.2672, 179.9602);
    private final Pose point8 = poseFactory.of(17.5015, 46.3175, -179.5818);
    private final Pose point9 = poseFactory.of(10.0877, 97.2652, -81.7205);

    public Path path1() {
        return Paths.line(start, path1).tangent();
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
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
        return Paths.curve(point6Start, point6Control1, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point6, point7).tangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).reverseTangent();
    }

    public Path path9() {
        return Paths.line(point8, point9).reverseTangent();
    }

    private final Pose path10 = poseFactory.of(10.1419, 46.7062, 180);
    private final Pose point11 = poseFactory.of(56.3129, 8.6589, 90.7611);
    private final Pose point11Control1 = poseFactory.of(22.8556, 46.8857, 0);
    private final Pose point11Control2 = poseFactory.of(55.8088, 46.6054, 0);
    private final Pose point12Start = poseFactory.of(56.3129, 8.6589, 90);
    private final Pose point12 = poseFactory.of(47.0736, 130.248, 90);
    private final Pose point12Control1 = poseFactory.of(11.8671, 41.5791, 0);
    private final Pose point12Control2 = poseFactory.of(18.651, 81.2517, 0);
    private final Pose point12Control3 = poseFactory.of(44.4672, 106.0659, 0);
    private final Pose point13 = poseFactory.of(13.8672, 119.179, -21.7888);
    private final Pose point13Control1 = poseFactory.of(46.8954, 124.2191, 0);
    private final Pose point13Control2 = poseFactory.of(51.4341, 104.1618, 0);

    public Path path10() {
        return Paths.line(start, path10).linear(start, path10);
    }

    public Path path11() {
        return Paths.curve(path10, point11Control1, point11Control2, point11).reverseTangent();
    }

    public Path path12() {
        return Paths.curve(point12Start, point12Control1, point12Control2, point12Control3, point12).linear(point12Start, point12);
    }

    public Path path13() {
        return Paths.curve(point12, point13Control1, point13Control2, point13).reverseTangent();
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

    private Command autoRoutine() {
        return sequential( // list of *commands* to run in a row when executed
                proboscisDown,
                follow(follower, path1()),
                intakeOn,
                follow(follower, path2()),
                intakeOff,
                turretOnOff,
                follow(follower, path3()),
                intakeOn,
                follow(follower, path4()),
                proboscisUp,
                follow(follower, path5()),
                intakeOff

        );
    }

    private Command autoRoutine2() {
        return sequential(
                proboscisDown,
                follow(follower, path10()),
                intakeOn,
                follow(follower, path11()),
                intakeOff,
                turretOnOff,
                follow(follower, path12()),
                intakeOn,
                follow(follower, path13()),
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
        Status.flywheelToggle = true;
        robotContainer.delayedActionManager.schedule(() -> robotContainer.tongue.up(), 1000);
        robotContainer.delayedActionManager.schedule(() -> Status.flywheelToggle = false, 2000);

            if (limelight.pipelineSwitch(8)) {
                schedule(autoRoutine2());
            }
            else {
                schedule(autoRoutine());
            }

            if (limelight.pipelineSwitch(9)) {
                schedule(autoRoutine2());
            }
            else {
                schedule(autoRoutine());
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