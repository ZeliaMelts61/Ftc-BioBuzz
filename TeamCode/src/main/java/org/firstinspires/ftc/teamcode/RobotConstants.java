package org.firstinspires.ftc.teamcode;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.AngularVelocity;

/**
 * RobotConstants Should contain all constants pertaining to the robot.
 * Constants for various systems are defined in subclasses.
 * It is recommended to only import the subclasses needed for your use cases
 */
public class RobotConstants {

    /** Constants for the intake Mechanism */
    public static class IntakeConstants{
        public static final String intakeRollerName = "intakeRoller";
        public static final String intakeLeftServoName = "leftIntake";
        public static final String intakeRightServoName = "rightIntake";

        public static final NextMotor.Direction intakeRollerDirection = NextMotor.Direction.FORWARD;
        public static final NextMotor.Direction intakeLeftServoDirection = NextMotor.Direction.FORWARD;
        public static final NextMotor.Direction intakeRightServoDirection = NextMotor.Direction.REVERSE;

        public static final NextMotor.ZeroPowerBehavior IntakeRollerZeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE;
    }

    /** Constants for the flywheel Mechanism */
    public static class FlywheelConstants{
        public static final String flywheelName = "flywheel";

        public static final NextMotor.Direction flywheelDirection = NextMotor.Direction.FORWARD;

        // THIS SHOULD NEVER BE ON BRAKE THAT'S HOW YOU DESTROY STUFF
        public static final NextMotor.ZeroPowerBehavior flywheelZeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT;

        public static final double VELOCITY_TOLERANCE_RPM = 30;
        public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerMinute.of(VELOCITY_TOLERANCE_RPM); // This is in Ticks per second not RPM


        // Pidf constants for the flywheel
        // The 'I' term and the 'A' term should probably not be used
        public static final double
                kP = 0.003,
                kI = 0,
                kD = 0.00002,
                kV = 0.000375,
                kA = 0,
                kS = 0.08; //TODO: TUNE VALUES

        public static final double encoderCountsPerRevolution = 28;
        public static final double gearReduction = 1;
        public static final double ticksPerRevolution = encoderCountsPerRevolution * gearReduction;
        public static final double degreesPerCount = 360.0 / ticksPerRevolution;
        public static final Angle angleUnitPerEncoderCount = Degrees.of(degreesPerCount);

        public static final double FLYWHEEL_MOTOR_CACHING_TOLERANCE = 0.05;

        // Define tuned (Distance in inches, RPM) data points here.
        // MUST BE SORTED by distance from lowest to highest. WITH NO DUPLICATES
        // It will do BAD things if it is not sorted correctly.
        // TODO: Put in real values here cause i doubt the robot can launch a ball at 5900 rpm 💀
        public static final double[][] RPM_LOOKUP_TABLE = {
                {94.0,  4500.0},  // {Distance, RPM}
                {110.0, 4900.0},
                {120.0, 5200.0},
                {130.0, 5550.0},
                {140.0, 5900.0}   // Add as many points as you need
        };


    }

    /** Constants for the windmill Mechanism */
    public static class WindmillConstants{
        public static final String windmillServoName = "windmillServo";
        public static final NextMotor.Direction windmillServoDirection = NextMotor.Direction.FORWARD;
    }

    /** Constants pertaining to robot telemetry */
    public static class TelemetryConstants{
        public static final boolean debugMode = false;
        public static final double TELEMETRY_UPDATE_MS = 125.0;
    }

    /** Constants pertaining to driver control.
     *  Example: joystick deadbands
     */
    public static class DriverConstants{

    }

    /** Constants pertaining to the match.
     * Example: The Poses of the hives
     */
    public static class MatchConstants{
        public enum ALLIANCE_COLOR{
            RED,
            BLUE,
            NONE
        }
        public static ALLIANCE_COLOR allianceColor = ALLIANCE_COLOR.NONE;
        public static void setAllianceColor(ALLIANCE_COLOR allianceColor) {
            MatchConstants.allianceColor = allianceColor;
        }

        private static final PoseFactory poseFactory = PoseFactory.degrees();

        // You may add as many shoot poses as you want as long as you update the shoot poses arrays below

        // TODO: put real shoot poses
        public static final Pose shootPoseRed1 = poseFactory.of(15,58,0);
        public static final Pose shootPoseRed2 = poseFactory.of(127,58, 180);
        public static final Pose shootPoseBlue1 = poseFactory.of(15,84, 0);
        public static final Pose shootPoseBlue2 = poseFactory.of(127,84,180);

        public static final Pose[] redShootPoses = {
                shootPoseRed1,
                shootPoseRed2};

        public static final Pose[] blueShootPoses = {
                shootPoseBlue1,
                shootPoseBlue2};

        public static final Pose[] allShootPoses = {
                shootPoseRed1,shootPoseRed2,
                shootPoseBlue1,shootPoseBlue2};


        public static final Pose hivePoseRed1 = new Pose(58,58);
        public static final Pose hivePoseRed2 = new Pose(84,58);
        public static final Pose hivePoseBlue1 = new Pose(58,84);
        public static final Pose hivePoseBlue2 = new Pose(84,84);

        public static final Pose[] redHivePoses = {
                hivePoseRed1,
                hivePoseRed2};

        public static final Pose[] blueHivePoses = {
                hivePoseBlue1,
                hivePoseBlue2};

        public static final Pose[] allHivePoses = {
                hivePoseRed1,hivePoseRed2,
                hivePoseBlue1,hivePoseBlue2};

    }
}
