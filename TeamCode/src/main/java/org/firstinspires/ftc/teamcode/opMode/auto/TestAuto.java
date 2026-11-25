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
public class TestAuto extends OpMode {

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
    private Pose start = poseFactory.of(31.1346, 8.1789, 90);
    private Pose verticalPoint = poseFactory.of(31.3135, 118.8249, 89.9074);
    private Pose start2 = poseFactory.of(8.9526, 31.4343, 0);
    private Pose horizontalPoint = poseFactory.of(130.775, 30.9912, -0.2084);


    public Path horizontalTest() {
        return Paths.line(start2, horizontalPoint).tangent();
    }

    public Path verticalTest() {
        return Paths.line(start, verticalPoint).tangent();
    }

    // Action Creation
    Command shoot() {
        return instant(() -> {
            robotContainer.delayedActionManager.schedule(() -> robotContainer.tongue.tick(), Constants.Flywheel.SPINUP_MS);
        });
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

    Command startSequence() {
        return instant(() -> {
            robotContainer.proboscis.up();
            robotContainer.wings.up();
            robotContainer.tongue.down();
            robotContainer.intake.on();
            Status.flywheelToggle = true;
        });
    }

    // Path methods
    private Command autoRoutine() {
        return sequential( // list of *commands* to run in a row when executed
                follow(follower, verticalTest())
//                follow(follower, horizontalTest())
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
        verticalPoint = HelperFunctions.scalePose(verticalPoint, start);
//        start2 = HelperFunctions.scalePose(start2, start2);
        horizontalPoint = HelperFunctions.scalePose(horizontalPoint, start2);

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            start = HelperFunctions.mirrorPoseAboutOrigin(start);
            verticalPoint = HelperFunctions.mirrorPoseAboutOrigin(verticalPoint);
//            start2 = HelperFunctions.mirrorPoseAboutOrigin(start2);
            horizontalPoint = HelperFunctions.mirrorPoseAboutOrigin(horizontalPoint);

        }

        robotContainer.init();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry(); // creates a new instance of panels
        Scheduler.reset(); // resets the command scheduler for the ivy follower

        follower = PedroPathingConstants.create(hardwareMap);
        follower.setPose(start); // Sets the starting pose for the robot
//        follower.setPose(start2);
        follower.update();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Alliance", Status.alliance);
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() { // Runs when the start button is pressed on the driver station
        follower.setPose(start);
//        follower.setPose(start2);
        robotContainer.start(this, false);
        RobotContainer.HardwareDevices.turretMotor.setPower(0);
        schedule(autoRoutine()); // This will schedule the auto routine that you had previously created to run in sequence

    }

    @Override
    public void loop() { // Runs constantly while the opmode is active
        follower.update(); // Updates that pathing follower to make the robot move in auto
        robotContainer.positionProvider.updateGoalPose();
        Pose scaledPose = HelperFunctions.scalePose(follower.pose(), start);
//        Pose scaledPose = HelperFunctions.scalePose(follower.pose(), start2);
        robotContainer.positionProvider.updateWithPose2D(HelperFunctions.pedroPathingToAudienceOrigin(new Pose2D(DistanceUnit.INCH, scaledPose.x(), scaledPose.y(), AngleUnit.DEGREES, (follower.pose().heading()))));
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
        Pose scaledPose = HelperFunctions.scalePose(follower.pose(), start);
//        Pose scaledPose = HelperFunctions.scalePose(follower.pose(), start2);
        blackboard.put("Robot Pose", HelperFunctions.pedroPathingToAudienceOrigin(new Pose2D(DistanceUnit.INCH, scaledPose.x(), scaledPose.y(), AngleUnit.RADIANS, scaledPose.heading())));
    }
}