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
public class TwoTips extends OpMode {

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
    private Pose start = poseFactory.of(63, 6.5, 180);
    private Pose startPose = poseFactory.of(59, 9, 180);
    private Pose beforeGarden = poseFactory.of(15, 7, 180);
    private Pose garden = poseFactory.of(7.5, 7, 180);
    private Pose shootAfterGarden = poseFactory.of(55, 9, 180);
    private Pose moveInFrontOfFlower = poseFactory.of(22, 50, 85);
    private Pose moveInFrontOfFlowerControl1 = poseFactory.of(16, 11, 0);
    private Pose intakeFromFlower = poseFactory.of(13, 50, -180);
    private Pose backAwayFromFlower = poseFactory.of(17, 50, -180);
    private Pose goToTheFlowerAtTheTop = poseFactory.of(47.3009, 127.2382, 90);
    private Pose goToTheFlowerAtTheTopControl1 = poseFactory.of(19.0379, 141.5, 0);
    private Pose goToTheFlowerAtTheTopControl2 = poseFactory.of(48.0717, 86.3854, 0);
    private Pose point11 = poseFactory.of(47.3476, 131.4899, -90.6298);
    private Pose point12 = poseFactory.of(47.3, 126.7, -90.5693);
    private Pose point13 = poseFactory.of(47.35, 131.6644, -90.577);
    private Pose point14 = poseFactory.of(47.3451, 113.7524, 89.9843);
    private Pose park = poseFactory.of(9.8787, 111.7182, 3.1078);



    public Path startPath() {
        return Paths.line(start, startPose).constant(start);
    }
    public Path startToBeforeGarden() {
        return Paths.line(start, beforeGarden).linear(start, beforeGarden);
    }

    public Path toGarden() {
        return Paths.line(beforeGarden, garden).linear(beforeGarden, garden);
    }

    public Path gardenToShoot() {
        return Paths.line(garden, shootAfterGarden).reverseTangent();
    }

    public Path shootToFlower() {
        return Paths.curve(shootAfterGarden, moveInFrontOfFlowerControl1, moveInFrontOfFlower).tangent();
    }

    public Path intakeFromFlower() {
        return Paths.line(moveInFrontOfFlower, intakeFromFlower).tangent();
    }

    public Path backUpFromFlower() {
        return Paths.line(intakeFromFlower, backAwayFromFlower).reverseTangent();
    }

    public Path goToTopFlower() {
        return Paths.curve(backAwayFromFlower, goToTheFlowerAtTheTopControl1, goToTheFlowerAtTheTopControl2, goToTheFlowerAtTheTop).constant(goToTheFlowerAtTheTop);
    }

    public Path moveInTheFlowerNumber2() {
        return Paths.line(goToTheFlowerAtTheTop, point11).reverseTangent();
    }

    public Path jiggleInAndOutOfTheFlower1() {
        return Paths.line(point11, point12).tangent();
    }

    public Path thePathThatPutsTheProboscisUp() {
        return Paths.line(point12, point13).reverseTangent();
    }

    public Path parkPath() {
        return Paths.line(point14, park).reverseTangent();
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
                startSequence(),
                follow(follower, startPath()),
                shoot(),
                waitMs(Constants.Flywheel.SPINUP_MS + Constants.Flywheel.SHOOT_MS + Constants.Tongue.POSITION_CHANGE_MS * 2),
                follow(follower, startToBeforeGarden()),
                waitMs(500),
                follow(follower, toGarden()),
                waitMs(500),
                follow(follower, gardenToShoot()),
                waitMs(500),
                shoot(),
                waitMs(Constants.Flywheel.SPINUP_MS + Constants.Flywheel.SHOOT_MS + Constants.Tongue.POSITION_CHANGE_MS * 2),
                waitMs(500),
                follow(follower, shootToFlower()),
                waitMs(300),
                proboscisDown(),
                follow(follower, intakeFromFlower()),
                follow(follower, backUpFromFlower()),
                follow(follower, intakeFromFlower()),
                waitMs(1000),
                follow(follower, backUpFromFlower()),
                waitMs(500),
                follow(follower, goToTopFlower()),
                shoot(),
                follow(follower, moveInTheFlowerNumber2()),
                follow(follower, jiggleInAndOutOfTheFlower1()),
                follow(follower, moveInTheFlowerNumber2()),
                waitMs(800),
                follow(follower, thePathThatPutsTheProboscisUp()),
                proboscisUp(),
                follow(follower, parkPath())




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
        startPose = HelperFunctions.scalePose(startPose,start);
        beforeGarden = HelperFunctions.scalePose(beforeGarden, start);
        garden = HelperFunctions.scalePose(garden, start);
        shootAfterGarden = HelperFunctions.scalePose(shootAfterGarden, start);
        moveInFrontOfFlower = HelperFunctions.scalePose(moveInFrontOfFlower, start);
        moveInFrontOfFlowerControl1 = HelperFunctions.scalePose(moveInFrontOfFlowerControl1, start);
        intakeFromFlower = HelperFunctions.scalePose(intakeFromFlower, start);
        backAwayFromFlower = HelperFunctions.scalePose(backAwayFromFlower, start);
        goToTheFlowerAtTheTop = HelperFunctions.scalePose(goToTheFlowerAtTheTop, start);
        goToTheFlowerAtTheTopControl1 = HelperFunctions.scalePose(goToTheFlowerAtTheTopControl1, start);
        goToTheFlowerAtTheTopControl2 = HelperFunctions.scalePose(goToTheFlowerAtTheTopControl2, start);
        point11 = HelperFunctions.scalePose(point11, start);
        point12 = HelperFunctions.scalePose(point12, start);
        point13 = HelperFunctions.scalePose(point13, start);
        point14 = HelperFunctions.scalePose(point14, start);
        park = HelperFunctions.scalePose(park, start);

        if (Status.alliance == Constants.Game.ALLIANCE.BLUE) {
            start = HelperFunctions.mirrorPoseAboutOrigin(start);
            startPose = HelperFunctions.mirrorPoseAboutOrigin(startPose);
            beforeGarden = HelperFunctions.mirrorPoseAboutOrigin(beforeGarden);
            garden = HelperFunctions.mirrorPoseAboutOrigin(garden);
            shootAfterGarden = HelperFunctions.mirrorPoseAboutOrigin(shootAfterGarden);
            moveInFrontOfFlower = HelperFunctions.mirrorPoseAboutOrigin(moveInFrontOfFlower);
            moveInFrontOfFlowerControl1 = HelperFunctions.mirrorPoseAboutOrigin(moveInFrontOfFlowerControl1);
            intakeFromFlower = HelperFunctions.mirrorPoseAboutOrigin(intakeFromFlower);
            backAwayFromFlower = HelperFunctions.mirrorPoseAboutOrigin(backAwayFromFlower);
            goToTheFlowerAtTheTop = HelperFunctions.mirrorPoseAboutOrigin(goToTheFlowerAtTheTop);
            goToTheFlowerAtTheTopControl1 = HelperFunctions.mirrorPoseAboutOrigin(goToTheFlowerAtTheTopControl1);
            goToTheFlowerAtTheTopControl2 = HelperFunctions.mirrorPoseAboutOrigin(goToTheFlowerAtTheTopControl2);
            point11 = HelperFunctions.mirrorPoseAboutOrigin(point11);
            point12 = HelperFunctions.mirrorPoseAboutOrigin(point12);
            point13 = HelperFunctions.mirrorPoseAboutOrigin(point13);
            point14 = HelperFunctions.mirrorPoseAboutOrigin(point14);
            park = HelperFunctions.mirrorPoseAboutOrigin(park);
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

    }

    @Override
    public void loop() { // Runs constantly while the opmode is active
        follower.update(); // Updates that pathing follower to make the robot move in auto
        robotContainer.positionProvider.updateGoalPose();
        Pose scaledPose = HelperFunctions.scalePose(follower.pose(), start);
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
        blackboard.put("Robot Pose", HelperFunctions.pedroPathingToAudienceOrigin(new Pose2D(DistanceUnit.INCH, scaledPose.x(), scaledPose.y(), AngleUnit.RADIANS, scaledPose.heading())));
    }
}