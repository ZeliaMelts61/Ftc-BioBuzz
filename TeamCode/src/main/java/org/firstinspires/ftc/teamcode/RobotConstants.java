package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotConstants {

    public static class IntakeConstants{
        public static final String intakeRollerName = "intakeRoller";
        public static final String intakeLeftServoName = "leftIntake";
        public static final String intakeRightServoName = "rightIntake";

        public static final DcMotor.Direction intakeRollerDirection = DcMotorSimple.Direction.FORWARD;
        public static final DcMotor.Direction intakeLeftServoDirection = DcMotorSimple.Direction.FORWARD;
        public static final DcMotor.Direction intakeRightServoDirection = DcMotorSimple.Direction.REVERSE;

        public static final DcMotor.ZeroPowerBehavior IntakeRollerZeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    }

    public static class FlywheelConstants{
        public static final String flywheelName = "flywheel";

        public static final DcMotor.Direction flywheelDirection = DcMotorSimple.Direction.FORWARD;

        // THIS SHOULD NEVER BE ON BRAKE THAT'S HOW YOU DESTROY STUFF
        public static final DcMotor.ZeroPowerBehavior flywheelZeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

        public static final int VELOCITY_TOLERANCE = 40; // This is in Ticks per second not RPM

        public static final double kS = 0.08, kV = 0.000375, kP = 0.003; //TODO: TUNE VALUES


    }

    public static class WindmillConstants{
        public static final String windmillServoName = "windmillServo";
        public static final DcMotor.Direction windmillServoDirection = DcMotorSimple.Direction.FORWARD;

    }

    public static class TelemetryConstants{
        public static final boolean debugMode = false;
        public static final double TELEMETRY_UPDATE_MS = 125.0;
    }

    public static class DriverConstants{

    }

    public static class  Hardware {
        // thresholds for caching wrappers
        public static double MOTOR_CACHING_TOLERANCE = 0.05;
        public static double CRSERVO_CACHING_TOLERANCE = 0.05;
        public static double SERVO_CACHING_TOLERANCE = 0.01;

        public static double PROXIMITY_POLL_MS = 100.0;
        public static double PROXIMITY_POLL_MS_FULL = 300.0;
        public static double TELEMETRY_UPDATE_MS = 125.0;
    }
}
