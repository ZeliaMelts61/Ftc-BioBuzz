package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants.IntakeConstants;
import org.firstinspires.ftc.teamcode.util.hardware.CRServoEx;
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;
import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

public class Intake {
    MotorEx intakeRoller;
    CRServoEx leftServo;
    CRServoEx rightServo;
    public Intake(HardwareMap hardwareMap){
        intakeRoller = new MotorEx(hardwareMap, IntakeConstants.intakeRollerName);
        intakeRoller.setDirection(IntakeConstants.intakeRollerDirection);
        intakeRoller.setZeroPowerBehavior(IntakeConstants.IntakeRollerZeroPowerBehavior);

        leftServo = new CRServoEx(hardwareMap, IntakeConstants.intakeLeftServoName);
        leftServo.setDirection(IntakeConstants.intakeLeftServoDirection);

        rightServo = new CRServoEx(hardwareMap, IntakeConstants.intakeRightServoName);
        rightServo.setDirection(IntakeConstants.intakeRightServoDirection);
    }

    public void setRollerPower(double power){
        intakeRoller.setPower(MathUtils.clamp(power,-1.0,1.0));
    }

    public void setLeftServoPower(double power){
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        leftServo.setPower(clamped);
    }

    public void setRightServoPower(double power){
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        rightServo.setPower(clamped);
    }

    public void setServoPower(double power){
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        leftServo.setPower(clamped);
        rightServo.setPower(clamped);
    }

    public void setAllPower(double power){
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        intakeRoller.setPower(clamped);
        leftServo.setPower(clamped);
        rightServo.setPower(clamped);
    }

    public void stopRoller(){
        intakeRoller.setPower(0);
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
        setAllPower(1);
    }

    public void outtake(){
        setAllPower(-1);
    }

    public double getRollerPower(){
        return intakeRoller.getPower();
    }

    public double getLeftServoPower(){
        return leftServo.getPower();
    }

    public double getRightServoPower(){
        return rightServo.getPower();
    }

//    public void updateTelemetry(Telemetry telemetry){
//        telemetry.addData("Intake Roller Power", getRollerPower());
//        telemetry.addData("Intake Left Servo Power", getLeftServoPower());
//        telemetry.addData("Intake Right Servo Power", getRightServoPower());
//    }

}
