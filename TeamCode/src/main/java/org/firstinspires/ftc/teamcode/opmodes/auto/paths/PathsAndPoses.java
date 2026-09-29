package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.shootPoseRed1;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.shootPoseRed2;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.data.Alliance;


public class PathsAndPoses {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    public PoseFactory getPoseFactory(){
        return poseFactory;
    }

    // WE ARE ALWAYS MAKING RED SIDE POSES
    public void mirrorPose(Alliance alliance){
        if (alliance == Alliance.BLUE) {
            poseFactory.mirrorX(72.0);
            poseFactory.mirrorY(72.0);
        }
    }

    // MAKE POSES HERE
    public final Pose startRightOfFlower = poseFactory.of(105.5, 8, 270);

    public final Pose gardenLineup = poseFactory.of(120, 8.5, 180);
    public final Pose gardenPickup = poseFactory.of(133, 8.5, 180);
    public final Pose gardenTo2ndShootControlPoint = poseFactory.of(13.5,27.5,0);

    public final Pose startLeftOfFlower = poseFactory.of(83.5,8,270);



//    public final Pose leavePos = poseFactory.of(137.37, 30, 180);
    // MAKE PATHS HERE

//    public Path startFLowerToLeavePos() {
//        return through(startNextToFlower, leavePos).constant(startNextToFlower);
//    }

    public Path startFlowerToShootPoseFirst(){
        return line(startRightOfFlower, shootPoseRed2).linear(startRightOfFlower,shootPoseRed2);
    }
    public Path shoot1ToGardenLineup(){
        return line(shootPoseRed2, gardenLineup).linear(shootPoseRed2,gardenLineup);
    }
    public Path gardenLineUpToGarden(){
        return Paths.line(gardenLineup,gardenPickup).reverseTangent();
    }
    public Path gardenToShootPoseSecond(){
        Interpolator interpolation = Interpolator.piecewise()
                        .until(0.1, Interpolator.constant(180))
                        .until(1, Interpolator.linear(gardenPickup,shootPoseRed1));
        return curve(gardenPickup,gardenTo2ndShootControlPoint,shootPoseRed1).heading(interpolation);
    }
}
