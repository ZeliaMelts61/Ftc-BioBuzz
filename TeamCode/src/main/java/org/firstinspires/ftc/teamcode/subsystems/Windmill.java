package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import androidx.core.math.MathUtils;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants.WindmillConstants;
import org.firstinspires.ftc.teamcode.util.hardware.CRServoEx;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.robot.Mechanism;

public class Windmill implements Mechanism {
    NextCRServo windmillServo;

    public Windmill(HardwareMap hardwareMap){
        windmillServo = new NextCRServo(WindmillConstants.windmillServoName);
        windmillServo.setDirection(WindmillConstants.windmillServoDirection);
    }

    public void setThrottle(double power){
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        windmillServo.setPower(clamped);
    }

    public double getThrottle(){
        return windmillServo.getPower();
    }

    public void forward(){
        setThrottle(1);
    }

    public void reverse(){
        setThrottle(-1);
    }

    public void stop(){
        setThrottle(0);
    }

    public Command setThrottleCommand(double throttle){
        return instant(()->setThrottle(throttle));
    }

    public Command forwardCommand(){
        return instant(this::forward);
    }

    public Command reverseCommand(){
        return instant(this::reverse);
    }

    public Command stopCommand(){
        return instant(this::stop);
    }




    @Override
    public void periodic(){}

}
