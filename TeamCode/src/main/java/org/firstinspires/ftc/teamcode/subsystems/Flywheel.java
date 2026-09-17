package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants.kP;
import static org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants.kS;
import static org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants.kV;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.util.hardware.MotorEx;

public class Flywheel {

    private double target = 0;
    private boolean activated = false;

    MotorEx flywheel;
    VoltageSensor voltageSensor;

    public Flywheel(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        this.voltageSensor = voltageSensor;
        flywheel = new MotorEx(hardwareMap, FlywheelConstants.flywheelName);
        flywheel.setDirection(FlywheelConstants.flywheelDirection);
        flywheel.setZeroPowerBehavior(FlywheelConstants.flywheelZeroPowerBehavior);
    }
    
    /**
     *
     * @param ticksPerSecond motor speed in ticks per second
     * @return flywheel speed in radians per second
     */
    public static double motorTicksToFlywheelRPM(double ticksPerSecond) {
        // 28 -- ticks per rotation
        // 60 -- seconds in a minute
        return (ticksPerSecond / 28) * 60;
    }

    /**
     *
     * @param rotationsPerMinute flywheel speed in rotations per minute
     * @return motor speed in ticks per second
     */
    public static double flywheelRPMToMotorTicks(double rotationsPerMinute) {
        // 28 -- ticks per rotation
        // 60 -- seconds in a minute
        return (rotationsPerMinute / 60) * 28;
    }


    public double getCurrentAngularVel() {
        return flywheel.getVelocity();
    }

    public double getCurrentRPM(){
        return motorTicksToFlywheelRPM(getCurrentAngularVel());
    }

    public double getTargetRPM(){
        return motorTicksToFlywheelRPM(getTargetAngularVelocity());
    }

    public void setTargetAngularVelocity(double target) {
        this.target = target;
    }

    public void setTargetRPM(double target) {
        this.target = flywheelRPMToMotorTicks(target);
    }


    public double getTargetAngularVelocity() {
        return target;
    }

    public double getPower(){
        return flywheel.getPower();
    }

    public void setPower(double power) {
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        flywheel.setPower(clamped);
    }

    public void deactivate() {
        activated = false;
        setPower(0);
    }

    public void activate() {
        activated = true;
    }

    public boolean isReady() {
        return Math.abs(getTargetAngularVelocity() - getCurrentAngularVel()) <= FlywheelConstants.VELOCITY_TOLERANCE;
    }

    public boolean getActivated() {
        return activated;
    }

    public double getCurrent() {
        return flywheel.getCurrent();
    }

    public void toggle() {
        activated = !activated;
        if (!activated) {
            setPower(0);
        }
    }


    // TODO: Change to be a real pid controller
    public void update() {
        if (activated) {
            double power =  (kV * getTargetAngularVelocity()) + (kP * (getTargetAngularVelocity() - getCurrentAngularVel())) + kS;
            power *= 12 / voltageSensor.getVoltage();
            setPower(power);
        }
    }
}
