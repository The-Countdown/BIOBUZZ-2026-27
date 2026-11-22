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
import org.firstinspires.ftc.teamcode.util.HelperFunctions;

@Autonomous
@Configurable // Allows you to configure in panels
public class PushZaRobot extends OpMode {

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
    private  Pose start = poseFactory.of(8.7737, 59.1618, 90);
    private  Pose moveAwayFromTheWall = poseFactory.of(22.548, 59.2554, 270);
    private  Pose pushTeamMate = poseFactory.of(9.0939, 97.1311, 270);
    private  Pose pushTeamMateControl1 = poseFactory.of(7.8226, 58.0276, 0);

    public Path moveAwayFromTheWall() {
        return Paths.line(start, moveAwayFromTheWall).constant(moveAwayFromTheWall);
    }

    public Path pushTeamMate() {
        return Paths.curve(moveAwayFromTheWall, pushTeamMateControl1, pushTeamMate).constant(pushTeamMate);
    }

    // Action Creation
    Command shoot() {

        return instant(() -> {
            robotContainer.intake.on();
            robotContainer.delayedActionManager.schedule(() -> robotContainer.tongue.tick(), Constants.Flywheel.SPINUP_MS);
            robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.off(), Constants.Flywheel.SPINUP_MS + Constants.Flywheel.SHOOT_MS);
        });
    }

    Command intakeOn() {
        return instant(() -> {
            robotContainer.delayedActionManager.schedule(() -> robotContainer.intake.on(), 1000);
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
            follow(follower, moveAwayFromTheWall()),
            waitMs(500),
            follow(follower, pushTeamMate())
        );
    }

    @Override
    public void init() { // runs when the init is pressed on the driver station
        try { // this prevents it from crashing because of an issue with robot container
            robotContainer = new RobotContainer(this);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        start = HelperFunctions.scalePose(start, start);
        moveAwayFromTheWall = HelperFunctions.scalePose(moveAwayFromTheWall, start);
        pushTeamMate = HelperFunctions.scalePose(pushTeamMate, start);
        pushTeamMateControl1 = HelperFunctions.scalePose(pushTeamMateControl1, start);

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            start = HelperFunctions.mirrorPoseAboutOrigin(start);
            moveAwayFromTheWall = HelperFunctions.mirrorPoseAboutOrigin(moveAwayFromTheWall);
            pushTeamMate = HelperFunctions.mirrorPoseAboutOrigin(pushTeamMate);
            pushTeamMateControl1 = HelperFunctions.mirrorPoseAboutOrigin(pushTeamMateControl1);

        }

        robotContainer.init();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry(); // creates a new instance of panels
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
        RobotContainer.HardwareDevices.turretMotor.setPower(0);
        schedule(autoRoutine()); // This will schedule the auto routine that you had previously created to run in sequence
        Status.flywheelToggle = true;

    }

    @Override
    public void loop() { // Runs constantly while the opmode is active
        follower.update(); // Updates that pathing follower to make the robot move in auto
        robotContainer.positionProvider.updateGoalPose();
        Pose scaledPose = HelperFunctions.scalePose(follower.pose(), start);
        robotContainer.positionProvider.updateWithPose2D(new Pose2D(DistanceUnit.INCH, (scaledPose.x() - 72), (scaledPose.y() - 72), AngleUnit.DEGREES, -HelperFunctions.normalizeAngle(Math.toDegrees(follower.pose().heading()) + 90)));
        robotContainer.update(false); // This will update all subsystems of the robot that need functions running in loop such as tracking the goal
        Scheduler.execute(); // Checks which command should be running right now in the list of your commands that you configured in the start function


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
        panelsTelemetry.debug("Goal Pose", Status.goalPose);
        panelsTelemetry.debug("Goal Pose get X", Status.goalPose.getX(DistanceUnit.CM));
        panelsTelemetry.debug("Goal Pose get Y", Status.goalPose.getY(DistanceUnit.CM));
        panelsTelemetry.debug("Current Pose X", Status.currentX);
        panelsTelemetry.debug("Current Pose Y", Status.currentY);
        panelsTelemetry.debug("Current Heading", Status.currentHeading);
        panelsTelemetry.debug("angle to face the goal", robotContainer.turret.angleToFaceGoal);

        panelsTelemetry.addData("Follower Mode", follower.mode());
        panelsTelemetry.update(telemetry);


    }

    @Override
    public void stop() {
        blackboard.put("Robot Pose", new Pose2D(DistanceUnit.INCH, (follower.pose().x() - 72) / Constants.System.AUTO_X_SCALER, (follower.pose().y() - 72) / Constants.System.AUTO_Y_SCALER, AngleUnit.DEGREES, -HelperFunctions.normalizeAngle(Math.toDegrees(follower.pose().heading()) + 90)));
    }
}