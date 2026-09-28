package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.core.math.MathUtils;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.RobotConstants.WindmillConstants;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.robot.Mechanism;

/**
 * This is the Windmill Mechanism.
 * It controls the windmill servo.
 */
public class Windmill implements Mechanism {
    NextCRServo windmillServo;

    /** Creates a windmill */
    public Windmill(){
        windmillServo = new NextCRServo(WindmillConstants.windmillServoName);
        windmillServo.setDirection(WindmillConstants.windmillServoDirection);
    }

    /**
     * Sets the throttle of the windmill servo
     * @param throttle A double -1 to 1 that the throttle of windmill servo will be set to.
     */
    public void setThrottle(double throttle){
        double clamped = MathUtils.clamp(throttle,-1.0,1.0);
        windmillServo.setPower(clamped);
    }

    /**
     * Gets the current throttle of the windmill servo.
     * @return A double representing the throttle of the windmill servo.
     */
    public double getThrottle(){
        return windmillServo.getPower();
    }

    /**
     * Sets the throttle of the windmill servo to 1 (max).
     */
    public void forward(){
        setThrottle(1);
    }

    /**
     * Sets the throttle of the windmill servo to -1 (backward max).
     */
    public void reverse(){
        setThrottle(-1);
    }

    /**
     * Stops the windmill servo.
     */
    public void stop(){
        setThrottle(0);
    }

    /**
     * Returns an instant command that sets the throttle of the windmill servo.
     * @param throttle A double -1 to 1 that the throttle of the windmill servo will be set to.
     * @return An instant command that sets the throttle.
     */
    public Command setThrottleCommand(double throttle){
        return instant(()->setThrottle(throttle));
    }

    /**
     * Returns an instant command that sets the throttle of the windmill servo to 1 (max).
     * @return An instant command that sets the throttle.
     */
    public Command forwardCommand(){
        return instant(this::forward);
    }

    /**
     * Returns an instant command that sets the throttle of the windmill servo to -1 (max backward).
     * @return An instant command that sets the throttle.
     */
    public Command reverseCommand(){
        return instant(this::reverse);
    }

    /**
     * Returns an instant command that stops the windmill servo.
     * @return An instant command that stops the windmill servo.
     */
    public Command stopCommand(){
        return instant(this::stop);
    }




    @Override
    public void periodic(){}

}
