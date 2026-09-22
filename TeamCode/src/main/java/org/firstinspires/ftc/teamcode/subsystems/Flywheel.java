package org.firstinspires.ftc.teamcode.subsystems;

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

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.AngularVelocity;
import dev.nextftc.units.measuretypes.Current;
import dev.nextftc.units.measuretypes.Distance;

public class Flywheel implements Mechanism {

    private AngularVelocity target = RotationsPerMinute.of(0);
    private boolean activated = false;

    NextMotor flywheel;

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

    public Flywheel() {



        flywheel = new NextMotor(
                FlywheelConstants.flywheelName,
                FlywheelConstants.angleUnitPerEncoderCount,
                RobotConstants.Hardware.MOTOR_CACHING_TOLERANCE);
        flywheel.setDirection(FlywheelConstants.flywheelDirection);
        flywheel.setZeroPowerBehavior(FlywheelConstants.flywheelZeroPowerBehavior);
        flywheel.setThrottle(0);

        flywheel.getVelocityConstants()
                .withP(kP)
                .withI(kI)
                .withD(kD)
                .withV(kV)
                .withA(kA)
                .withS(kS);
    }

    public AngularVelocity getCurrentAngularVel() {
        return flywheel.getEncoderVelocity();
    }

    public double getCurrentRPM(){
        return getCurrentAngularVel().into(RotationsPerMinute);
    }

    public double getTargetRPM(){
        return target.into(RotationsPerMinute);
    }

    public void setTargetAngularVelocity(AngularVelocity target) {
        this.target = target;
    }

    public void setTargetRPM(double target) {
        this.target = RotationsPerMinute.of(target);
    }


    public AngularVelocity getTargetAngularVelocity() {
        return target;
    }

    public double getThrottle(){
        return flywheel.getThrottle();
    }

    public void setThrottle(double throttle) {
        double clamped = MathUtils.clamp(throttle,-1.0,1.0);
        flywheel.setThrottle(clamped);
    }

    public void deactivate() {
        activated = false;
        setThrottle(0);
    }

    public void activate() {
        activated = true;
    }

    // this is a mess
    // It should:
    // 1. Subtract the current vel from the target vel
    // 2. Absolute value it
    // 3. Check to see if that value is less than the tolerance
    public boolean isReady() {
        return getCurrentAngularVel().minus(getTargetAngularVelocity()).getAbsoluteValue().isNear(RotationsPerMinute.zero(),FlywheelConstants.VELOCITY_TOLERANCE);
    }

    public boolean getActivated() {
        return activated;
    }

    public Current getCurrent() {
        return flywheel.getCurrent();
    }

    public void toggle() {
        activated = !activated;
        if (!activated) {
            setThrottle(0);
        }
    }

    /** Set the flywheel velocity based on the distance from the robot to the hive */
    public void setVelocityFromDistance(Distance distance){
        setTargetAngularVelocity(calcVelocityFromDist(distance));
    }

    private AngularVelocity calcVelocityFromDist(Distance dist){
        return RotationsPerMinute.of(ShooterRegression.rpmFromDistanceInches(dist.into(Inches)));
    }

    // TODO: Change to be a real pid controller
    @Override
    public void periodic(){
        if(activated){
            flywheel.setVelocitySetpoint(target);
            flywheel.update();
        } else {
            flywheel.setThrottle(0);
        }
    }

    public Command setTargetRPMCommand(double rpm){
        return instant(() -> this.setTargetRPM(rpm));
    }

    public Command activateCommand(){
        return instant(this::activate);
    }


    public Command deactivateCommand(){
        return instant(this::deactivate);
    }

    public Command toggleCommand(){
        return instant(this::toggle);
    }

    public Command setThrottleCommand(double throttle){
        return instant(()->setThrottle(throttle));
    }

    public Command setTargetAngularVelocityCommand(AngularVelocity target) {
        return instant(()->setTargetAngularVelocity(target));
    }

    public Command setVelocityFromDistanceCommand(Distance distance) {
        return instant(()->setTargetAngularVelocity(target));
    }



}
