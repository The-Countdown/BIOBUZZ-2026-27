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
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("rightFront");
                c.frontRightName.set("leftFront");
                c.backLeftName.set("leftBack");
                c.backRightName.set("rightBack");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(0.46352619261253536);
        c.yPodOffset.set(-0.5923430375226839);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3108641511385021);
                Controller secondaryTranslationalForward = Controller.proportional(0.11485599740257615);
                Controller primaryTranslationalLateral = Controller.proportional(0.6766098781028997);
                Controller secondaryTranslationalLateral = Controller.proportional(0.24998927061010637);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.008990941713010323));
                c.brake.set(Controller.proportionalFeedforward(0.007642300456058774));

                c.headingFeedback.set(Controller.proportional(8.31171820244894));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.062339166371417745, 0.007765809306430234));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10830424282121787, 0.049794735108412806));
                c.quadraticBrakeCoefficients.set(Matrix.diag(6.579671110389808E-4, 0.0016058930215237719));

                c.maxAchievableForwardVelocity.set(103.82184992550326);
                c.maxAchievableStrafeVelocity.set(80.64531151629507);
                c.naturalForwardDeceleration.set(46.89768958402567);
                c.naturalStrafeDeceleration.set(102.75601265816611);
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