package org.firstinspires.ftc.teamcode.subsystems;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants.WindmillConstants;
import org.firstinspires.ftc.teamcode.util.hardware.CRServoEx;

public class Windmill {
    CRServoEx windmillServo;

    public Windmill(HardwareMap hardwareMap){
        windmillServo = new CRServoEx(hardwareMap, WindmillConstants.windmillServoName);
        windmillServo.setDirection(WindmillConstants.windmillServoDirection);
    }

    public void setPower(double power){
        double clamped = MathUtils.clamp(power,-1.0,1.0);
        windmillServo.setPower(clamped);
    }

    public double getPower(){
        return windmillServo.getPower();
    }

    public void forward(){
        setPower(1);
    }

    public void reverse(){
        setPower(-1);
    }

    public void stop(){
        setPower(0);
    }

}
