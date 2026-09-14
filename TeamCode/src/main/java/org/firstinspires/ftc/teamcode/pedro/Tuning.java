package org.firstinspires.ftc.teamcode.pedro;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
    @Tuner
    public static Procedure tests() {
        // to test to see if everything got setup right
        // https://pedropathing.com/docs/pathing/tuning/test
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), null, null);
    }

    @Tuner
    public static Procedure pinpointTuner() {
        // to tune the gobuilda pinpoint odometry hardware
        // https://pedropathing.com/docs/pathing/tuning/localization/pinpoint#add-the-tuner
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure testLocalizer() { // I renamed this class
        // This tests the localizer
        // https://pedropathing.com/docs/pathing/tuning/test
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig)), null);
    }

    @Tuner
    public static Procedure tests() {
        // Runs the tests of everything
        // https://pedropathing.com/docs/pathing/tuning/test#add-the-tests-procedure
        return new Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig)), () -> new Foresight(Constants.foresightConfig));
    }

    @Tuner
    public static Procedure foresightTuner() {
        // The Foresight AutoTuner can automatically determine your ForesightConfig, which includes max achievable velocities, brake coefficients, and kP values.
        // https://pedropathing.com/docs/pathing/tuning/foresight#automatic-tuning
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }
}
