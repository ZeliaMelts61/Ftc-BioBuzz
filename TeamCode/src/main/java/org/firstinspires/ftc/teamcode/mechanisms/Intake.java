package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.core.math.MathUtils;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.RobotConstants.IntakeConstants;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

/**
 * This is the Intake Mechanism.
 * It controls the intake roller, the left intake servo and the right intake servo.
 */
public class Intake implements Mechanism {
    NextMotor intakeRoller;
    NextCRServo leftServo;
    NextCRServo rightServo;

    /**
     * Creates an Intake
     */
    public Intake(){
        intakeRoller = new NextMotor(IntakeConstants.intakeRollerName);
        intakeRoller.setDirection(IntakeConstants.intakeRollerDirection);
        intakeRoller.setZeroPowerBehavior(IntakeConstants.IntakeRollerZeroPowerBehavior);

        leftServo = new NextCRServo(IntakeConstants.intakeLeftServoName);
        leftServo.setDirection(IntakeConstants.intakeLeftServoDirection);

        rightServo = new NextCRServo(IntakeConstants.intakeRightServoName);
        rightServo.setDirection(IntakeConstants.intakeRightServoDirection);
    }

    /**
     * Sets the throttle of the intake roller.
     * @param throttle A double -1 to 1 that the throttle of the intake roller will be set to.
     */
    public void setRollerThrottle(double throttle){
        intakeRoller.setThrottle(MathUtils.clamp(throttle,-1.0,1.0));
    }

    /**
     * Sets the throttle of the left intake servo.
     * @param throttle A double -1 to 1 that the throttle of the left intake servo will be set to.
     */
    public void setLeftServoThrottle(double throttle){
        double clamped = MathUtils.clamp(throttle,-1.0,1.0);
        leftServo.setPower(clamped);
    }

    /**
     * Sets the throttle of the right intake servo.
     * @param throttle A double -1 to 1 that the throttle of the right intake servo will be set to.
     */
    public void setRightServoThrottle(double throttle){
        double clamped = MathUtils.clamp(throttle,-1.0,1.0);
        rightServo.setPower(clamped);
    }

    /**
     * Sets the throttle of both the left and right intake servos.
     * @param throttle A double -1 to 1 that the throttle of both the left and right intake servos will be set to.
     */
    public void setServosThrottle(double throttle){
        double clamped = MathUtils.clamp(throttle,-1.0,1.0);
        leftServo.setPower(clamped);
        rightServo.setPower(clamped);
    }

    /**
     * Sets the throttle of all actuators in the intake.
     * This includes the left and right intake servos, and the intake roller.
     * @param throttle A double -1 to 1 that the throttle of all intake actuators will be set to.
     */
    public void setAllThrottle(double throttle){
        double clamped = MathUtils.clamp(throttle,-1.0,1.0);
        intakeRoller.setThrottle(clamped);
        leftServo.setPower(clamped);
        rightServo.setPower(clamped);
    }

    /**
     * Stops the intake roller by setting its throttle to 0.
     */
    public void stopRoller(){
        intakeRoller.setThrottle(0);
    }

    /**
     * Stops the intake Servos by setting their throttles to 0.
     */
    public void stopServos(){
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    /**
     * Stops all actuators in the intake by setting their throttles to 0.
     * This includes the left and right intake servos, and the intake roller.
     */
    public void stop(){
        stopRoller();
        stopServos();
    }

    /**
     *  Sets the throttle of all actuators on the intake to 1 (max).
     *  This includes the left and right intake servos, and the intake roller.
     */
    public void intake(){
        setAllThrottle(1);
    }

    /**
     *  Sets the throttle of all actuators on the intake to -1 (max backward).
     *  This includes the left and right intake servos, and the intake roller.
     */
    public void outtake(){
        setAllThrottle(-1);
    }

    /**
     * Gets the throttle of the intake roller.
     * @return A double representing the throttle of the intake roller.
     */
    public double getRollerThrottle(){
        return intakeRoller.getThrottle();
    }

    /**
     * Gets the throttle of the left intake servo.
     * @return A double representing the throttle of the left intake servo.
     */
    public double getLeftServoThrottle(){
        return leftServo.getPower();
    }

    /**
     * Gets the throttle of the right intake servo.
     * @return A double representing the throttle of the right intake servo.
     */
    public double getRightServoThrottle(){
        return rightServo.getPower();
    }

    /**
     * Returns an instant command that sets the throttle of the intake roller.
     * @param throttle A double -1 to 1 that the throttle of the intake roller will be set to.
     * @return An instant command that sets the throttle.
     */
    public Command setRollerThrottleCommand(double throttle){
        return instant(() -> this.setRollerThrottle(throttle));
    }

    /**
     * Returns an instant command that sets the throttle of the left intake servo.
     * @param throttle A double -1 to 1 that the throttle of the left intake servo will be set to.
     * @return An instant command that sets the throttle.
     */
    public Command setLeftServoThrottleCommand(double throttle){
        return instant(() -> this.setLeftServoThrottle(throttle));
    }
    /**
     * Returns an instant command that sets the throttle of the right intake servo.
     * @param throttle A double -1 to 1 that the throttle of the right intake servo will be set to.
     * @return An instant command that sets the throttle.
     */
    public Command setRightServoThrottleCommand(double throttle){
        return instant(() -> this.setRightServoThrottle(throttle));
    }

    /**
     * Returns an instant command that sets the throttle both intake servos.
     * @param throttle A double -1 to 1 that the throttle of both intake servos will be set to.
     * @return An instant command that sets the throttle.
     */
    public Command setServosThrottleCommand(double throttle){
        return instant(() -> this.setServosThrottle(throttle));
    }

    /**
     * Returns an instant command that sets the throttle of all intake actuators.
     * This includes the left and right intake servos, and the intake roller.
     * @param throttle A double -1 to 1 that the throttle of all intake actuators will be set to.
     * @return An instant command that sets the throttle.
     */
    public Command setIntakeThrottleCommand(double throttle) {
        return instant(() -> this.setAllThrottle(throttle));
    }

    /**
     * Returns an instant command that sets the throttle of all intake actuators to 1 (max).
     * This includes the left and right intake servos, and the intake roller.
     * @return An instant command that sets the throttle.
     */
    public Command intakeCommand(){
        return setIntakeThrottleCommand(1);
    }

    /**
     * Returns an instant command that sets the throttle of all intake actuators to -1 (max backward).
     * This includes the left and right intake servos, and the intake roller.
     * @return An instant command that sets the throttle.
     */
    public Command outtakeCommand(){
        return setIntakeThrottleCommand(-1);
    }

    /**
     * Returns an instant command that stops all intake actuators.
     * This includes the left and right intake servos, and the intake roller.
     * @return An instant command that sets the throttle.
     */
    public Command stopIntakeCommand(){
        return instant(this::stop);
    }

    @Override
    public void periodic() {}

}
