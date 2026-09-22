package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.api.Paths.line;

import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.allianceColor;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.hivePoseBlue1;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.hivePoseBlue2;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.hivePoseRed1;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.hivePoseRed2;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.shootPoseBlue1;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.shootPoseBlue2;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.shootPoseRed1;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.shootPoseRed2;

import static dev.nextftc.units.Units.Inches;

import androidx.annotation.NonNull;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.Path;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.DoubleSupplier;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.Distance;

public class Drivetrain implements Mechanism {
    Follower follower;
    PoseFactory poseFactory = PoseFactory.degrees();

    private final Controller headingController = Constants.foresightConfig.headingFeedback.get();

    public Drivetrain(Follower follower){
        this.follower=follower;
    }

    public void manualArcade(double forward, double heading){
        follower.manual(forward,0,heading);
    }

    public void manualRobotOriented(double forward, double lateral, double heading){
        follower.manual(forward, lateral, heading);
    }

    public void manualFieldOriented(double forward, double lateral, double heading, double angle){
        DrivePowers powers = getFieldCentricPowers(forward,lateral,heading,angle);
        follower.manual(powers);
    }

    public void manualFieldOriented(double forward, double lateral, double heading){
        manualFieldOriented(forward,lateral,heading,follower.pose().heading());
    }

    private DrivePowers getFieldCentricPowers(double forward, double lateral, double heading, double currentAngle){
        return ManualDrive.fieldCentric(
                -forward,
                lateral,
                heading,
                currentAngle);
    }

    private DrivePowers getRobotCentricPowers(double forward, double lateral, double heading){
        return ManualDrive.fieldCentric(
                -forward,
                lateral,
                heading,
                0.0);
    }


    public void manualHoldAngleFieldOriented(double forward, double lateral, double heading, double currentAngle, double holdAngle){
        DrivePowers powers =
            ManualDrive.headingLock(
                    follower,
                    headingController,
                    getFieldCentricPowers(forward,lateral,heading,currentAngle),
                    holdAngle);
        follower.manual(powers);
    }

    public void manualHoldAngleFieldOriented(double forward, double lateral, double heading, double holdAngle) {
        manualHoldAngleFieldOriented(forward, lateral, heading, follower.pose().heading(), holdAngle);
    }

    /** not recommended */
    public void manualHoldAngleRobotOriented(double forward, double lateral, double heading, double holdAngle){
        DrivePowers powers =
                ManualDrive.headingLock(
                        follower,
                        headingController,
                        getRobotCentricPowers(forward,lateral,heading),
                        holdAngle);
        follower.manual(powers);
    }

    public void followPath(Path path){
        follower.follow(path);
    }

    private Path getPathToShootPose(){
        Pose[] shootPoses;
        Pose[] hivePoses;
        switch (allianceColor){
            case Red:
                shootPoses = new Pose[]{shootPoseRed1, shootPoseRed2};
                hivePoses = new Pose[]{hivePoseRed1, hivePoseRed2};
            case BLUE:
                shootPoses = new Pose[]{shootPoseBlue1, shootPoseBlue2};
                hivePoses = new Pose[]{hivePoseBlue1, hivePoseBlue2};
            default:
                shootPoses = new Pose[]{
                        shootPoseRed1,shootPoseRed2,shootPoseBlue1,shootPoseBlue2};
                hivePoses = new Pose[]{
                        hivePoseRed1, hivePoseRed2, hivePoseBlue1, hivePoseBlue2};
        }
        Pose shootPose = closestPose(follower.pose(), shootPoses);
        Pose hivePose = closestPose(shootPose, hivePoses);

        return line(follower.pose(),shootPose).facingPoint(hivePose);
    }

    public void pathToShootPose() {
        follower.follow(getPathToShootPose());
    }

    private Pose closestPose(Pose pose, @NonNull Pose[] poses){
        double[] distances = new double[poses.length];
        for (int i = 0; i < poses.length; i++) {
            distances[i] = pose.distance(poses[i]);
        }
        int minInd=0;
        double minDist = distances[0];
        for(int i = 0; i < distances.length; i++){
            if(distances[i] < minDist){
                minInd=i;
                minDist=distances[i];
            }
        }
        return poses[minInd];
    }

    public void stop(){
        manualArcade(0,0);
    }

    public Command manualArcadeCommand(DoubleSupplier forward, DoubleSupplier heading){
        return infinite(()->{
            manualArcade(forward.getAsDouble(),heading.getAsDouble());
        }).setEnd((endCondition)-> {
            stop();
        }).requiring(this);
    }

    public Command manualRobotOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading){
        return infinite(()->{
            manualRobotOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    public Command manualFieldOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading, DoubleSupplier angle){
        return infinite(()->{
            manualFieldOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble(),angle.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    public Command manualFieldOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading){
        return infinite(()->{
            manualFieldOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    public Command manualHoldAngleFieldOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading, DoubleSupplier currentAngle, DoubleSupplier holdAngle){
        return infinite(()-> {
            manualHoldAngleFieldOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble(),currentAngle.getAsDouble(),holdAngle.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    public Command manualHoldAngleFieldOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading, DoubleSupplier holdAngle){
        return infinite(()-> {
            manualHoldAngleFieldOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble(),holdAngle.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    /** not recommended */
    public Command manualHoldAngleRobotOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading, DoubleSupplier holdAngle){
        return infinite(()-> {
            manualHoldAngleRobotOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble(),holdAngle.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    public Command followPathCommand(Path path){
        return new CommandBuilder()
                .setStart(()->follower.follow(path))
                .setDone(follower::atParametricEnd)
                .setEnd((endCondition) -> follower.stop())
                .requiring(this);
    }

    public Command pathToShootPoseCommand(){
        return followPathCommand(getPathToShootPose());
    }

    @Override
    public void periodic(){
        follower.update();
    }

    private Pose closestHivePose(Pose pose){
        Pose[] hivePoses;
        switch (allianceColor){
            case Red:
                hivePoses = new Pose[]{hivePoseRed1, hivePoseRed2};
            case BLUE:
                hivePoses = new Pose[]{hivePoseBlue1, hivePoseBlue2};
            default:
                hivePoses = new Pose[]{
                        hivePoseRed1, hivePoseRed2, hivePoseBlue1, hivePoseBlue2};
        }
        return closestPose(pose,hivePoses);
    }


    public Distance distanceToHive(){
        return Inches.of(closestHivePose(follower.pose()).distance(follower.pose()));
    }




}
