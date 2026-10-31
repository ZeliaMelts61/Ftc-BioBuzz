package org.firstinspires.ftc.teamcode;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
        public static final NextMotor.Direction intakeLeftServoDirection = NextMotor.Direction.REVERSE;
        public static final NextMotor.Direction intakeRightServoDirection = NextMotor.Direction.FORWARD;

        public static final NextMotor.ZeroPowerBehavior IntakeRollerZeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE;
    }
    public static class HoodConstants{
        public static final String hoodServoName = "hoodServo";
        public static final NextMotor.Direction hoodServoDirection = NextMotor.Direction.FORWARD;
    }

    public static double map(double val, double low1, double high1, double low2, double high2) {
        return low2 + ((val - low1) * (high2 - low2) / (high1 - low1));
    }
    /** Constants for the flywheel Mechanism */
    public static class FlywheelConstants{
        public static final String flywheelName = "flywheel";

        public static final NextMotor.Direction flywheelDirection = NextMotor.Direction.FORWARD;

        // THIS SHOULD NEVER BE ON BRAKE THAT'S HOW YOU DESTROY STUFF
        public static final NextMotor.ZeroPowerBehavior flywheelZeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT;

        public static final double VELOCITY_TOLERANCE_RPM = 45;
        public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerMinute.of(VELOCITY_TOLERANCE_RPM); // This is in Ticks per second not RPM

        // we can try these for pids
        // motorOne.getPositionConstants().withV(0.0075);
        //        motorOne.getPositionConstants().withP(0.1);
        // public static double KP  = 0.0005;
        //    public static double KF  = 0.003;

        // Pidf constants for the flywheel
        // The 'I' term and the 'A' term should probably not be used
//        PIDFCoefficients q = new PIDFCoefficients(0,0,0,0);
        public static final double
                kP = 0.0005, // pulled from another team GitHub
                kI = 0,
                kD = 0.0,
                kV = 0.0075, //  pulled from another team GitHub
                kA = 0, // Need to tune
                kS = 0.008; // made up just cause
        public static final double encoderCountsPerRevolution = 28;
        public static final double gearReduction = 1;
        public static final double ticksPerRevolution = encoderCountsPerRevolution * gearReduction;
        public static final double degreesPerCount = 360.0 / ticksPerRevolution;
        public static final Angle angleUnitPerEncoderCount = Degrees.of(degreesPerCount);

        public static final double FLYWHEEL_MOTOR_CACHING_TOLERANCE = 0.05;



        // Define tuned (Distance in inches, RPM) data points here.
        // MUST BE SORTED by distance from lowest to highest. WITH NO DUPLICATES
        // It will do BAD things if it is not sorted correctly.
        // TODO: Put in real values here
        public static final double[][] RPM_LOOKUP_TABLE = {
                {10, 1300},  // {Distance, RPM}
                {20, 1400},
                {30, 1500},
                {40, 1600},
                {43, 2678}, // this value is right the rest are not
                {50, 2750}   // Add as many points as you need
        };


    }

    public static class BackupFlywheelConstants{
        public static final DcMotorEx.Direction flywheelDirection = DcMotorEx.Direction.FORWARD;

        // THIS SHOULD NEVER BE ON BRAKE THAT'S HOW YOU DESTROY STUFF
        public static final DcMotorEx.ZeroPowerBehavior flywheelZeroPowerBehavior = DcMotorEx.ZeroPowerBehavior.FLOAT;


        public static final double
                kP = 40, // pulled from sample code
                kI = 0,  // pulled from sample code
                kD = 0, // pulled from sample code
                kF = 12.5; // pulled from sample code



    }

    /** Constants for the windmill Mechanism */
    public static class WindmillConstants{
        public static final String windmillServoName = "windmillServo";
        public static final NextMotor.Direction windmillServoDirection = NextMotor.Direction.FORWARD;
    }

    /** Constants pertaining to robot telemetry */
    public static class TelemetryConstants{
        public static final boolean debugMode = true;
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
        private static final PoseFactory poseFactory = PoseFactory.degrees();

        // You may add as many shoot poses as you want as long as you update the shoot poses arrays below

        // TODO: put real shoot poses
        public static final Pose shootPoseRed1 = poseFactory.of(57.5,8.5,90);
        public static final Pose shootPoseRed2 = poseFactory.of(57.5,132, 180);
        public static final Pose shootPoseBlue1 = poseFactory.of(84,8.5, 0);
        public static final Pose shootPoseBlue2 = poseFactory.of(84,133,180);


        public static final Pose[] redShootPoses = {
                shootPoseRed1,
                shootPoseRed2};

        public static final Pose[] blueShootPoses = {
                shootPoseBlue1,
                shootPoseBlue2};

        public static final Pose[] allShootPoses = {
                shootPoseRed1,shootPoseRed2,
                shootPoseBlue1,shootPoseBlue2};


        public static final Pose hivePoseRed1 = new Pose(57.5,55);
        public static final Pose hivePoseRed2 = new Pose(57.5,87);
        public static final Pose hivePoseBlue1 = new Pose(84,55);
        public static final Pose hivePoseBlue2 = new Pose(84,87);

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
