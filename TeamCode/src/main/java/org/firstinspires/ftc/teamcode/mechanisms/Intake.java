package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.core.math.MathUtils;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.RobotConstants.IntakeConstants;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    NextMotor intakeRoller;
    NextCRServo leftServo;
    NextCRServo rightServo;
    public Intake(){
        intakeRoller = new NextMotor(IntakeConstants.intakeRollerName);
        intakeRoller.setDirection(IntakeConstants.intakeRollerDirection);
        intakeRoller.setZeroPowerBehavior(IntakeConstants.IntakeRollerZeroPowerBehavior);

        leftServo = new NextCRServo(IntakeConstants.intakeLeftServoName);
        leftServo.setDirection(IntakeConstants.intakeLeftServoDirection);

        rightServo = new NextCRServo(IntakeConstants.intakeRightServoName);
        rightServo.setDirection(IntakeConstants.intakeRightServoDirection);

//        intakeRoller.setThrottle(0);
//        leftServo.setPower(0);
//        rightServo.setPower(0);
    }

    public void setRollerThrottle(double power){
        intakeRoller.setThrottle(MathUtils.clamp(power,-1.0,1.0));
    }

    public void setLeftServoThrottle(double power){
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        leftServo.setPower(clamped);
    }

    public void setRightServoThrottle(double power){
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        rightServo.setPower(clamped);
    }

    public void setServoThrottle(double power){
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        leftServo.setPower(clamped);
        rightServo.setPower(clamped);
    }

    public void setAllThrottle(double power){
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        intakeRoller.setThrottle(clamped);
        leftServo.setPower(clamped);
        rightServo.setPower(clamped);
    }

    public void stopRoller(){
        intakeRoller.setThrottle(0);
    }

    public void stopServo(){
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    public void stop(){
        stopRoller();
        stopServo();
    }

    public void intake(){
        setAllThrottle(1);
    }

    public void outtake(){
        setAllThrottle(-1);
    }

    public double getRollerThrottle(){
        return intakeRoller.getThrottle();
    }

    public double getLeftServoThrottle(){
        return leftServo.getPower();
    }

    public double getRightServoThrottle(){
        return rightServo.getPower();
    }

    public Command setIntakeThrottleCommand(double power) {
        return instant(() -> this.setAllThrottle(power));
    }

    public Command intakeCommand(){
        return setIntakeThrottleCommand(1);
    }

    public Command outtakeCommand(){
        return setIntakeThrottleCommand(-1);
    }

    public Command stopIntakeCommand(){
        return instant(this::stop);
    }

    @Override
    public void periodic() {}

//    public void updateTelemetry(Telemetry telemetry){
//        telemetry.addData("Intake Roller Throttle", getRollerThrottle());
//        telemetry.addData("Intake Left Servo Throttle", getLeftServoThrottle());
//        telemetry.addData("Intake Right Servo Throttle", getRightServoThrottle());
//    }

}
