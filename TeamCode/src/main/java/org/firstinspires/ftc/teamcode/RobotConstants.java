package org.firstinspires.ftc.teamcode;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.reflect.Field;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.AngularVelocity;
import dev.nextftc.units.unittypes.AngleUnit;

public class RobotConstants {

    public static class IntakeConstants{
        public static final String intakeRollerName = "intakeRoller";
        public static final String intakeLeftServoName = "leftIntake";
        public static final String intakeRightServoName = "rightIntake";

        public static final NextMotor.Direction intakeRollerDirection = NextMotor.Direction.FORWARD;
        public static final NextMotor.Direction intakeLeftServoDirection = NextMotor.Direction.FORWARD;
        public static final NextMotor.Direction intakeRightServoDirection = NextMotor.Direction.REVERSE;

        public static final NextMotor.ZeroPowerBehavior IntakeRollerZeroPowerBehavior = NextMotor.ZeroPowerBehavior.BRAKE;
    }

    public static class FlywheelConstants{
        public static final String flywheelName = "flywheel";

        public static final NextMotor.Direction flywheelDirection = NextMotor.Direction.FORWARD;

        // THIS SHOULD NEVER BE ON BRAKE THAT'S HOW YOU DESTROY STUFF
        public static final NextMotor.ZeroPowerBehavior flywheelZeroPowerBehavior = NextMotor.ZeroPowerBehavior.FLOAT;

        public static final double VELOCITY_TOLERANCE_RPM = 30;
        public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerMinute.of(VELOCITY_TOLERANCE_RPM); // This is in Ticks per second not RPM


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

    public static class WindmillConstants{
        public static final String windmillServoName = "windmillServo";
        public static final NextMotor.Direction windmillServoDirection = NextMotor.Direction.FORWARD;

    }

    public static class TelemetryConstants{
        public static final boolean debugMode = false;
        public static final double TELEMETRY_UPDATE_MS = 125.0;
    }

    public static class DriverConstants{

    }

    public static class  Hardware {
        // thresholds for caching wrappers
        public static final double MOTOR_CACHING_TOLERANCE = 0.05;
        public static final double CRSERVO_CACHING_TOLERANCE = 0.05;
        public static final double SERVO_CACHING_TOLERANCE = 0.01;

        public static final double PROXIMITY_POLL_MS = 100.0;
        public static final double PROXIMITY_POLL_MS_FULL = 300.0;
        public static final double TELEMETRY_UPDATE_MS = 125.0;
    }

    public static class MatchConstants{
        public enum ALLIANCE_COLOR{
            Red,
            BLUE,
            NONE
        }
        public static ALLIANCE_COLOR allianceColor = ALLIANCE_COLOR.NONE;
        public static void setAllianceColor(ALLIANCE_COLOR allianceColor) {
            MatchConstants.allianceColor = allianceColor;
        }

        private static final PoseFactory poseFactory = PoseFactory.degrees();
//         TODO: put real shoot poses
        public static final Pose shootPoseRed1 = poseFactory.of(15,58,0);
        public static final Pose shootPoseRed2 = poseFactory.of(127,58, 180);
        public static final Pose shootPoseBlue1 = poseFactory.of(15,84, 0);
        public static final Pose shootPoseBlue2 = poseFactory.of(127,84,180);

        public static final Pose hivePoseRed1 = new Pose(58,58);
        public static final Pose hivePoseRed2 = new Pose(84,58);
        public static final Pose hivePoseBlue1 = new Pose(58,84);
        public static final Pose hivePoseBlue2 = new Pose(84,84);
    }
}
