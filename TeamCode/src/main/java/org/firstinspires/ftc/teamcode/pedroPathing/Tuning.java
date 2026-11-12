package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.main.Constants;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;

public class Tuning {
    // Tuners go here
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, PedroPathingConstants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, PedroPathingConstants.localizerConfig)), () -> new Foresight(PedroPathingConstants.foresightConfig));
    }

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
}
