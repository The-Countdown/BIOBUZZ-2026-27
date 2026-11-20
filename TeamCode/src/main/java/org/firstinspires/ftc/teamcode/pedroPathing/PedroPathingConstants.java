package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.main.Constants;

public class PedroPathingConstants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(Constants.System.PINPOINT_X_OFFSET_MM);
        c.yPodOffset.set(Constants.System.PINPOINT_Y_OFFSET_MM);
        c.xPodDirection.set(Constants.System.PINPOINT_X_ENCODER_DIRECTION);
        c.yPodDirection.set(Constants.System.PINPOINT_Y_ENCODER_DIRECTION);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.MM);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.375);
                Controller secondaryTranslationalForward = Controller.proportional(0.125);
                Controller primaryTranslationalLateral = Controller.proportional(0.285);
                Controller secondaryTranslationalLateral = Controller.proportional(0.075);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.0085));
                c.brake.set(Controller.proportionalFeedforward(0.0075));

                c.headingFeedback.set(Controller.proportional(5.5));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.055, 0.005));

                c.linearBrakeCoefficients.set(Matrix.diag(0.075, 0.0495));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001, 0.001));

                c.maxAchievableForwardVelocity.set(100.0);
                c.maxAchievableStrafeVelocity.set(85.0);
                c.naturalForwardDeceleration.set(50.0);
                c.naturalStrafeDeceleration.set(110.0);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}