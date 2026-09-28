package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.utils.Utils.lerp;
import static org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants.RPM_LOOKUP_TABLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants.kA;
import static org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants.kD;
import static org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants.kI;
import static org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants.kP;
import static org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants.kS;
import static org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants.kV;

import static dev.nextftc.units.Units.Inches;
import static dev.nextftc.units.Units.RotationsPerMinute;

import androidx.core.math.MathUtils;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.FlywheelConstants;

import java.util.function.Supplier;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.AngularVelocity;
import dev.nextftc.units.measuretypes.Current;
import dev.nextftc.units.measuretypes.Distance;

/**
 * This is the Flywheel Mechanism.
 * It controls the flywheel motor.
 */
public class Flywheel implements Mechanism {

    private AngularVelocity target = RotationsPerMinute.of(0);
    private boolean activated = false;

    NextMotor flywheel;

    /**
     * A Class used for shooter regression.
     * (E.g. Calculating the rpm of the flywheel based on distance to the hive)
     */
    private static class ShooterRegression {
        public static double rpmFromDistanceInches(double distance) {
            int length = RPM_LOOKUP_TABLE.length;

            // Edge Case distance is closer than the closest data point
            if (distance <= RPM_LOOKUP_TABLE[0][0]) {
                return RPM_LOOKUP_TABLE[0][1];
            }

            // Edge Case distance is further than the furthest data point
            else if (distance >= RPM_LOOKUP_TABLE[length - 1][0]) {
                return RPM_LOOKUP_TABLE[length - 1][1];
            }

            // 2. Binary Search: Find the exact interval at O(log N) runtime speed
            int low = 0;
            int high = length - 1;
            int index = 0;

            while (low <= high) {
                int mid = (low + high) / 2;
                if (RPM_LOOKUP_TABLE[mid][0] <= distance) {
                    index = mid; // This is the lower bound of our interval
                    low = mid + 1;
                } else {
                    high = mid - 1;
                }
            }

            // 3. Extract the two points surrounding our current distance
            double x0 = RPM_LOOKUP_TABLE[index][0];
            double y0 = RPM_LOOKUP_TABLE[index][1];
            double x1 = RPM_LOOKUP_TABLE[index + 1][0];
            double y1 = RPM_LOOKUP_TABLE[index + 1][1];

            // Calculate 't' (the percentage of the distance between x0 and x1)
            double t = (distance - x0) / (x1 - x0);

            // Call your existing project-wide lerp function
            return lerp(y0, y1, t);
        }
    }

    /**
     * Constructs A flywheel object using values defined in RobotConstants.FlywheelConstants
     */
    public Flywheel() {
        flywheel = new NextMotor(
                FlywheelConstants.flywheelName,
                FlywheelConstants.angleUnitPerEncoderCount,
                FlywheelConstants.FLYWHEEL_MOTOR_CACHING_TOLERANCE);
        flywheel.setDirection(FlywheelConstants.flywheelDirection);
        flywheel.setZeroPowerBehavior(FlywheelConstants.flywheelZeroPowerBehavior);
        flywheel.getVelocityConstants()
                .withP(kP)
                .withI(kI)
                .withD(kD)
                .withV(kV)
                .withA(kA)
                .withS(kS);
    }


    /**
     * Gets the current angular velocity of the flywheel.
     * @return An AngularVelocity object representing the current angular velocity of the flywheel.
     */
    public AngularVelocity getCurrentAngularVel() {
        return flywheel.getEncoderVelocity();
    }

    /**
     * Gets the current Rotations Per Minute of the flywheel.
     * @return A double representing the current RPM of the flywheel.
     */
    public double getCurrentRPM(){
        return getCurrentAngularVel().into(RotationsPerMinute);
    }

    /**
     * Sets the target angular velocity of the flywheel.
     * @param target The target angular velocity.
     */
    public void setTargetAngularVelocity(AngularVelocity target) {
        this.target = target;
    }

    /**
     * Sets the target Rotations Per Minute of the flywheel.
     * @param target The target RPM.
     */
    public void setTargetRPM(double target) {
        this.target = RotationsPerMinute.of(target);
    }

    /**
     * Gets the target angular velocity of the flywheel.
     * @return An AngularVelocity object representing the target angular velocity of the flywheel.
     */
    public AngularVelocity getTargetAngularVelocity() {
        return target;
    }

    /**
     * Gets the target Rotations Per Minute of the flywheel.
     * @return A double representing the target RPM of the flywheel.
     */
    public double getTargetRPM(){
        return target.into(RotationsPerMinute);
    }


    /**
     * Gets the current throttle of the flywheel.
     * @return A double representing the current throttle of the flywheel.
     */
    public double getThrottle(){
        return flywheel.getThrottle();
    }

    /**
     * Sets the throttle of the flywheel.
     * IMPORTANT: THIS WILL STILL MOVE THE FLYWHEEL EVEN WHEN IT IS DEACTIVATED.
     * @param throttle A value -1 to 1 that flywheel throttle will be set to.
     */
    public void setThrottle(double throttle) {
        double clamped = MathUtils.clamp(throttle,-1.0,1.0);
        flywheel.setThrottle(clamped);
    }

    /**
     * Deactivates the flywheel. This turns off PIDF control and lets the flywheel coast.
     * IMPORTANT: THIS WILL NOT STOP {@link #setThrottle(double)} FROM CONTROLLING THE FLYWHEEL.
     */
    public void deactivate() {
        activated = false;
        setThrottle(0);
    }

    /**
     * Activates the flywheel. This turns on PIDF control.
     */
    public void activate() {
        activated = true;
    }

    /**
     * Returns if the flywheel is activated
     * @return A boolean, true if the flywheel is activated, false otherwise.
     */
    public boolean getActivated() {
        return activated;
    }

    /**
     * Toggles the activated/deactivated state of the flywheel.
     * E.g. if the is activated and toggle is called the flywheel will be deactivated.
     */
    public void toggle() {
        activated = !activated;
        if (!activated) {
            setThrottle(0);
        }
    }

    /**
     * Checks if the flywheel is up to target speed.
     * 1. It subtracts the current velocity from the target velocity
     * 2. Absolute values the result of step 1
     * 3. Checks to see if the result of step 2 is less than the velocity tolerance
     * @return A boolean, true if the flywheel is up to speed, and false otherwise
     */
    public boolean isReady() {
        return getCurrentAngularVel().minus(getTargetAngularVelocity()).getAbsoluteValue().isNear(RotationsPerMinute.zero(),FlywheelConstants.VELOCITY_TOLERANCE);
    }

    /**
     * Gets the current draw of the flywheel motor.
     * @return A Current object representing the current draw of the flywheel motor.
     */
    public Current getCurrent() {
        return flywheel.getCurrent();
    }


//

    /**
     * Sets the flywheel velocity based on the distance from the robot to the hive
     * @param distance A Distance object representing the distance between the center of the robot and the hive.
     */
    public void setVelocityFromDistance(Distance distance){
        setTargetAngularVelocity(calcVelocityFromDist(distance));
    }

    /**
     * Calculates teh Flywheel velocity based on the distance to the hive.
     * @param dist The distance between the center of the robot and the hive.
     * @return An Angular velocity that the flywheel need to be at to make it in to the hive from {@code dist}
     */
    private AngularVelocity calcVelocityFromDist(Distance dist){
        return RotationsPerMinute.of(ShooterRegression.rpmFromDistanceInches(dist.into(Inches)));
    }

    /**
     * If the flywheel is activated it sets the velocity target to the target velocity and updates the flywheel.
     * If the flywheel is deactivated it sets the throttle to 0 and coasts the flywheel down.
     */
    @Override
    public void periodic(){
        if(activated){
            flywheel.setVelocitySetpoint(target);
            flywheel.update();
        } else {
            flywheel.setThrottle(0);
        }
    }

    /**
     * Returns an instant command that sets the target angular velocity of the flywheel.
     * @param target The target angular velocity that the flywheel should be set to.
     * @return An instant command that sets the target angular velocity of the flywheel.
     */
    public Command setTargetAngularVelocityCommand(AngularVelocity target) {
        return instant(()->setTargetAngularVelocity(target));
    }

    /**
     * Returns an instant command that sets the target Rotations Per Minute of the flywheel.
     * @param rpm The target RPM that the flywheel should be set to.
     * @return An instant command that sets the target RPM of the flywheel.
     */
    public Command setTargetRPMCommand(double rpm){
        return instant(() -> this.setTargetRPM(rpm));
    }

    /**
     * Returns an instant command that sets the throttle of the flywheel.
     * IMPORTANT: THIS WILL STILL MOVE THE FLYWHEEL EVEN WHEN IT IS DEACTIVATED.
     * @param throttle A value -1 to 1 that flywheel throttle will be set to.
     * @return An instant command that sets the throttle of the flywheel.
     */
    public Command setThrottleCommand(double throttle){
        return instant(()->setThrottle(throttle));
    }

    /**
     * Returns an instant command that will activate the flywheel.
     * @return An instant command that activates the flywheel.
     */
    public Command activateCommand(){
        return instant(this::activate);
    }

    /**
     * Returns an instant command that will deactivate the flywheel.
     * IMPORTANT: THIS WILL NOT STOP {@link #setThrottleCommand(double)} FROM CONTROLLING THE FLYWHEEL.
     * @return An instant command that deactivates the flywheel.
     */
    public Command deactivateCommand(){
        return instant(this::deactivate);
    }

    /**
     * Returns an instant command that toggles the activated/deactivated state of the flywheel.
     * E.g. if the flywheel is activated and the toggleCommand is called the flywheel will be deactivated.
     * @return An instant command that toggles the activated/deactivated state of the flywheel.
     */
    public Command toggleCommand(){
        return instant(this::toggle);
    }

    /**
     * Returns an instant command that sets the flywheel velocity based on the distance from the robot to the hive
     * @param distance A Distance object representing the distance between the center of the robot and the hive.
     * @return An instant command that sets the target flywheel velocity.
     */
    public Command setVelocityFromDistanceCommand(Distance distance) {
        return instant(()->setVelocityFromDistance(distance));
    }

    /**
     * Returns an infinite command that sets the flywheel velocity based on the distance from the robot to the hive
     * @param distance A Distance supplier object representing the distance between the center of the robot and the hive.
     * @return An infinite command that sets the target flywheel velocity.
     */
    public Command setVelocityFromDistanceContinuousCommand(Supplier<Distance> distance){
        return infinite(()->setVelocityFromDistance(distance.get()));
    }



}
