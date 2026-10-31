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
            poseFactory.mirrorX((141.5/2));
            poseFactory.mirrorY((141.5/2));
        }
    }

    // start close shoot to park
    public static final Pose startFacingUpHive = poseFactory.of(59, 8.5, 270);
    private final Pose parkClose = poseFactory.of(14, 95.5, 270);
    private final Pose startShootToParkControl1 = poseFactory.of(23, 15, 0);

    public Path startShootToPark() {
        return Paths.curve(startFacingUpHive, startShootToParkControl1, parkClose).constant(parkClose);
    }


    // from wait shoot to park
    public static final Pose startFacingDownHive = poseFactory.of(57.5, 134, 90);
    private final Pose waitShootPark = poseFactory.of(9, 118, 90);
    private final Pose waitShootParkControl1 = poseFactory.of(58, 115, 0);

    public Path waitShootPark() {
        return Paths.curve(startFacingDownHive, waitShootParkControl1, waitShootPark).linear(startFacingDownHive, waitShootPark);
    }


    // 2 cycle
    private final Pose gardenLineup = poseFactory.of(8.5, 14.5, 270);
    private final Pose gardenPickup = poseFactory.of(8.5, 6, -90);
    private final Pose shoot2 = poseFactory.of(61.5, 132.5, -270);
    private final Pose gardenToShoot2Control1 = poseFactory.of(35, 30, 0);
    private final Pose gardenToShoot2Control2 = poseFactory.of(0, 107, 0);
    private final Pose gardenToShoot2Control3 = poseFactory.of(58, 108, 0);
    private final Pose gardenToShoot2Segment1Heading = poseFactory.of(60, 133, 270);
    private final Pose gardenToShoot2Segment2Start = poseFactory.of(60, 133, -90);
    private final Pose gardenToShoot2Segment2End = poseFactory.of(60, 133, 90);
    private final Pose parkFar = poseFactory.of(10, 118, 90);
    private final Pose shoot2ToParkControl1 = poseFactory.of(63, 115, 0);

    public Path startToGardenLineup() {
        return Paths.line(startFacingUpHive, gardenLineup).linear(startFacingUpHive, gardenLineup);
    }

    public Path gardenPickup() {
        return Paths.line(gardenLineup, gardenPickup).tangent();
    }

    public Path gardenToShoot2() {
        return Paths.curve(gardenPickup, gardenToShoot2Control1, gardenToShoot2Control2, gardenToShoot2Control3, shoot2).heading(Interpolator.piecewise().until(0.1, Interpolator.constant(gardenToShoot2Segment1Heading)).until(1, Interpolator.linear(gardenToShoot2Segment2Start, gardenToShoot2Segment2End)));
    }

    public Path shoot2ToPark() {
        return Paths.curve(shoot2, shoot2ToParkControl1, parkFar).constant(parkFar);
    }




    private final Pose gardenToParkControl1 = poseFactory.of(45, 47, 0);

    public Path gardenToPark() {
        return Paths.curve(gardenPickup, gardenToParkControl1, parkClose).constant(gardenPickup);
    }

    private final Pose farShootToPickupControl1 = poseFactory.of(69, 107.5, 0);
    private final Pose farShootToPickupControl2 = poseFactory.of(0, 130, 0);
    private final Pose farShootToPickupControl3 = poseFactory.of(42, 22.5, 0);

    public Path farShootToPickup() {
        return Paths.curve(startFacingDownHive, farShootToPickupControl1, farShootToPickupControl2, farShootToPickupControl3, gardenLineup).linear(startFacingDownHive, gardenLineup);
    }


}