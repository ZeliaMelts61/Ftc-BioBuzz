package org.firstinspires.ftc.teamcode.opmodes.auto.paths;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.data.Alliance;


public class PathsAndPoses {
    private static final PoseFactory poseFactory = PoseFactory.degrees();
    public PoseFactory getPoseFactory(){
        return poseFactory;
    }

    public PathsAndPoses(Alliance alliance){
        mirrorPose(alliance);
    }

    // WE ARE ALWAYS MAKING RED SIDE POSES
    private void mirrorPose(Alliance alliance){
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

    // This is the starting position for exactly in line with the upward starting hive.
    public static final Pose againstWallByUpwardsFacingStartingHive = poseFactory.of(135.5,59,0);

    public final Pose shootPose1 = poseFactory.of(127,58,180);
    public final Pose shootPose2 = poseFactory.of(15,58,0);

    public Pose parkPose = poseFactory.of(46,10,0);
    public Pose controlPointForStartShootPoseToPark = poseFactory.of(107,18,0);

    public Path startShootToPark(){
        return curve(againstWallByUpwardsFacingStartingHive, controlPointForStartShootPoseToPark, parkPose).constant(againstWallByUpwardsFacingStartingHive);
    }
    public Path startFlowerToShootPoseFirst(){
        return line(startRightOfFlower, shootPose1).linear(startRightOfFlower,shootPose1);
    }
    public Path shoot1ToGardenLineup(){
        return line(shootPose1, gardenLineup).linear(shootPose1,gardenLineup);
    }
    public Path gardenLineUpToGarden(){
        return Paths.line(gardenLineup,gardenPickup).reverseTangent();
    }
    public Path gardenToShootPoseSecond(){
        Interpolator interpolation = Interpolator.piecewise()
                .until(0.1, Interpolator.constant(180))
                .until(1, Interpolator.linear(gardenPickup,shootPose2));
        return curve(gardenPickup,gardenTo2ndShootControlPoint,shootPose2).heading(interpolation);
    }
}