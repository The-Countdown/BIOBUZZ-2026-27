package org.firstinspires.ftc.teamcode.main;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.HashMap;

/**
 * The `Constants` class provides a centralized location for all the fixed values and
 * Configurableurations used throughout the robot's code.
 */
public class Constants {

    public static PinpointConfig localizerConfig;
    public static MecanumConfig drivetrainConfig;

    @Configurable
    public static class Game {
        public enum ALLIANCE {
            BLUE,
            RED
        }

        public static
            Pose2D ORIGIN = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0),
            FLOWER_RESET_POSE = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        public static double
            GRAVITY = 9.811,
            GOAL_HEIGHT = 144.78,
            CELL_POSITION_X = 35.56,
            CELL_POSITION_Y = 30.48,
            GOAL_POSITION_VELOCITY_MULTIPLIER = 1;


    }

    @Configurable
    public static class Robot {
        public static int
            MOTOR_TICKS_PER_REVOLUTION = 28,
            MOTOR_UPDATE_TIME = 50,
            SERVO_UPDATE_TIME = 100;
        public static double
            MOTOR_UPDATE_MIN_POWER_DIFF = 0.1,
            MOTOR_UPDATE_MIN_VELOCITY_DIFF = MOTOR_TICKS_PER_REVOLUTION * 10,
            SERVO_UPDATE_MIN_POWER_DIFF = 0.05,
            SERVO_UPDATE_MIN_POS_DIFF = 0.0125;

        public static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
    }

    @Configurable
    public static class System {
        public static final int
            LOOP_AVERAGE_WINDOW_SIZE = 30,
            TELEMETRY_UPDATE_INTERVAL_MS = 750,
            TELEMETRY_COMP_UPDATE_INTERVAL_MS = 750,
            PINPOINT_UPDATE_DELAY_MS = 50;
        public static final double
            PINPOINT_X_OFFSET_MM = 31,
            PINPOINT_Y_OFFSET_MM = 2.0,
            IMU_PER_ROTATION_OFFSET = 3.1;
        public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_ODOM_POD = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
        public static final GoBildaPinpointDriver.EncoderDirection
            PINPOINT_X_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD,
            PINPOINT_Y_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.REVERSED;

    }

    @Configurable
    public static class Control {
        public static double
            JOYSTICK_SCALER_EXPONENT = 3.0,
            JOYSTICK_ROTATION_SCALER_EXPONENT = 3.0,
            MAX_DRIVE_ACCELERATION = 3,
            ROTATION_SCALER = 0.75;
    }

    @Configurable
    public static class LED {
        public static final HashMap<COLOR, LED_COLOR_VALUES> COLOR_MAP = new HashMap<COLOR, LED_COLOR_VALUES>() {{
            put(COLOR.OFF, new LED_COLOR_VALUES(500, 0.0));
            put(COLOR.RED, new LED_COLOR_VALUES(1100, 0.279));
            put(COLOR.ORANGE, new LED_COLOR_VALUES(1200, 0.333));
            put(COLOR.YELLOW, new LED_COLOR_VALUES(1300, 0.388));
            put(COLOR.SAGE, new LED_COLOR_VALUES(1400, 0.444));
            put(COLOR.GREEN, new LED_COLOR_VALUES(1500, 0.500));
            put(COLOR.AZURE, new LED_COLOR_VALUES(1600, 0.555));
            put(COLOR.BLUE, new LED_COLOR_VALUES(1700, 0.611));
            put(COLOR.INDIGO, new LED_COLOR_VALUES(1800, 0.666));
            put(COLOR.VIOLET, new LED_COLOR_VALUES(1900, 0.722));
            put(COLOR.WHITE, new LED_COLOR_VALUES(2500, 1.0));
        }};

        public enum COLOR {
            OFF,
            RED,
            ORANGE,
            YELLOW,
            SAGE,
            GREEN,
            AZURE,
            BLUE,
            INDIGO,
            VIOLET,
            WHITE
        }

        public static class LED_COLOR_VALUES {
            public final int MICROSECONDS; public final double ANALOG;
            public LED_COLOR_VALUES(int MICROSECONDS, double ANALOG) {
                this.MICROSECONDS = MICROSECONDS; this.ANALOG = ANALOG;
            }
        }

        public static double
            FLYWHEEL_GRADIANT_PERIOD = 100,
            RAINBOW_SPEED = 0.003;
    }

    @Configurable
    public static class Wings {
        public static double
            MAX = 0.41,
            EXTENDED = 0.4,
            MIN = 0.325,
            PARK = 0.41,
            FOLLOWER_OFFSET = 0.625;
    }

    @Configurable
    public static class Proboscis {
        public static double
            DOWN = 0.8,
            UP = 0.25;
    }

    @Configurable
    public static class Tongue {
        public static double
            UP = 0.15,
            DOWN = 0.575,
            FOLLOWER_OFFSET = 0.075,
            POSITION_CHANGE_TIME = 250;
    }

    @Configurable
    public static class Turret {
        public static double
            GEAR_RATIO = 64.0/3.0,
            DEGREES_PER_TICK = (360.0 / (GEAR_RATIO * Robot.MOTOR_TICKS_PER_REVOLUTION)), //135 / 224
            MAX_ANGLE = 180,
            MIN_ANGLE = -180,
            SPEED_FACTOR = 0.0003,
            KP = 0.02,
            KI = 0.0,
            KD = 1.0,
            KF = 0.1,
            TOLERANCE_DEGREES = 3,
            GEAR_SLOP = 0.5,
            TOLERANCE_TICKS = (TOLERANCE_DEGREES - GEAR_SLOP) / DEGREES_PER_TICK;
        public static int
            ZERO_DEGREE_POSITION = 0,
            MAX_MOTOR = ZERO_DEGREE_POSITION + 300,
            MIN_MOTOR = ZERO_DEGREE_POSITION - 300;
        public static double[]
            HOOD_PRESETS = {0.58, 0.7, 0.95},
            HOOD_TABLE_DISTANCES = {50, 90, 130};

    }

    @Configurable
    public static class Flywheel {
        public static double GEAR_RATIO = 3.0/4.0;
        public static int MAX_VELOCITY = (int) (6000 * GEAR_RATIO * Robot.MOTOR_TICKS_PER_REVOLUTION) / 60; // 2100
        public static double KP = 0.000008;
        public static double KI = 0.0;
        public static double KD = 0.001;
        public static double KF = 0.95;
        public static double KS = 3;
        public static double[] SPEED_TABLE = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        public static double[] SPEED_TABLE_DISTANCES = {40, 56, 90, 95, 103, 180, 205};
        public static int SPINUP_MS = 1000;
        public static double HEIGHT = 0.0;
        public static double DESIRED_MAX_HEIGHT = 0.0;
        public static double POWER_VELOCITY_MULTIPLIER = 1;
    }

    @Configurable
    public static class Intake {
        public static double TOP_SPEED = 1;
        public static double BEST_INTAKE_SPEED = 1;
        public static double REVERSE_TOP_SPEED = 0.5;
    }
}
