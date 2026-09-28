package org.firstinspires.ftc.teamcode.mechanisms;

import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.allHivePoses;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.allShootPoses;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.allianceColor;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.blueHivePoses;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.blueShootPoses;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.hivePoseBlue1;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.hivePoseBlue2;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.hivePoseRed1;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.hivePoseRed2;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.redHivePoses;
import static org.firstinspires.ftc.teamcode.RobotConstants.MatchConstants.redShootPoses;
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
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.function.DoubleSupplier;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.Distance;

/**
 * This is the Drivetrain Mechanism.
 * It is important that the Follower is passed into this class
 * That can be done either by using the constructor that includes follower,
 * or by calling setFollower.
 * If the drivetrain is not provided a follower it will most likely
 * raise a NULL_POINTER_EXCEPTION
 */
public class Drivetrain implements Mechanism {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Command defaultCommand = infinite(()->{});

    private final Controller headingController = Constants.foresightConfig.headingFeedback.get();

    /**
     * Constructs a new Drivetrain.
     * IMPORTANT: If using this constructor it is necessary to call
     * the setFollower method before attempting to use any other methods.
     */
    public Drivetrain(){
    }

    /**
     * Constructs a new Drivetrain.
     * It is not necessary to call setFollower with this constructor
     */
    public Drivetrain(Follower follower){
        this.follower=follower;
    }

    /**
     * Sets the Pedropather follower that this Drivetrain will use.
     * It is not necessary to call this method if the drivetrain was
     * constructed using the constructor with a parameter of Follower.
     * @param follower The Follower this drivetrain will use.
     */
    public void setFollower(Follower follower){
        this.follower=follower;
    }

    /**
     * Drives the robot using Robot oriented arcade drive.
     * Must be called every loop.
     * @param forward The -1 to 1 value that controls the robot relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param heading The -1 to 1 value that controls the counterclockwise/clockwise rotation effort.
     *                Positive is counterclockwise movement.
     */
    public void manualArcade(double forward, double heading){
        follower.manual(forward,0,heading);
    }

    /**
     * Drives the robot using Robot oriented Mecanum drive.
     * Must be called every loop.
     * @param forward The -1 to 1 value that controls the robot relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral The -1 to 1 value that controls the robot relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading The -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     */
    public void manualRobotOriented(double forward, double lateral, double heading){
        follower.manual(forward, lateral, heading);
    }

    /**
     * Drives the robot using Field oriented Mecanum drive.
     * Must be called every loop.
     * @param forward The -1 to 1 value that controls the field relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral The -1 to 1 value that controls the field relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading The -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @param angle The current angle of the robot in Radians
     */
    public void manualFieldOriented(double forward, double lateral, double heading, double angle){
        DrivePowers powers = getFieldCentricPowers(forward,lateral,heading,angle);
        follower.manual(powers);
    }

    /**
     * Drives the robot using Field oriented Mecanum drive, using the current heading of the follower.
     * Must be called every loop.
     * @param forward The -1 to 1 value that controls the field relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral The -1 to 1 value that controls the field relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading The -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     */
    public void manualFieldOriented(double forward, double lateral, double heading){
        manualFieldOriented(forward,lateral,heading,follower.pose().heading());
    }

    /**
     * Creates a Field relative DrivePowers object from the input parameters.
     * @param forward The -1 to 1 value that controls the field relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral The -1 to 1 value that controls the field relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading The -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @param currentAngle The current angle of the robot in Radians.
     * @return A DrivePowers object representing the Field relative movement of the parameters provided.
     */
    private DrivePowers getFieldCentricPowers(double forward, double lateral, double heading, double currentAngle){
        return ManualDrive.fieldCentric(
                -forward,
                lateral,
                heading,
                currentAngle);
    }

    /**
     * Creates a Robot relative DrivePowers object from input parameters.
     * @param forward The -1 to 1 value that controls the robot relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral The -1 to 1 value that controls the robot relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading The -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @return A DrivePowers object representing the Robot relative movement of the parameters provided.
     */
    private DrivePowers getRobotCentricPowers(double forward, double lateral, double heading){
        return ManualDrive.fieldCentric(
                -forward,
                lateral,
                heading,
                0.0);
    }

    /**
     * Drives the robot using Field relative hold angle Mecanum drive.
     * @param forward The -1 to 1 value that controls the field relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral The -1 to 1 value that controls the field relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading The -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @param currentAngle The current angle of the robot in Radians.
     * @param holdAngle The Field Relative angle in radians that the robot should hold.
     */
    public void manualHoldAngleFieldOriented(double forward, double lateral, double heading, double currentAngle, double holdAngle){
        DrivePowers powers =
            ManualDrive.headingLock(
                    follower,
                    headingController,
                    getFieldCentricPowers(forward,lateral,heading,currentAngle),
                    holdAngle);
        follower.manual(powers);
    }

    /**
     * Drives the robot using Field relative hold angle Mecanum drive, using the current angle of the follower.
     * @param forward The -1 to 1 value that controls the field relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral The -1 to 1 value that controls the field relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading The -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @param holdAngle The Field Relative angle in radians that the robot should hold.
     */
    public void manualHoldAngleFieldOriented(double forward, double lateral, double heading, double holdAngle) {
        manualHoldAngleFieldOriented(forward, lateral, heading, follower.pose().heading(), holdAngle);
    }

    /**
     * NOT RECOMMENDED: Tends to be very disorienting.
     * Drives the robot using Robot relative hold angle Mecanum drive.
     * @param forward The -1 to 1 value that controls the robot relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral The -1 to 1 value that controls the robot relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading The -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @param holdAngle The FIELD RELATIVE angle in radians that the robot should hold.
     */
    public void manualHoldAngleRobotOriented(double forward, double lateral, double heading, double holdAngle){
        DrivePowers powers =
                ManualDrive.headingLock(
                        follower,
                        headingController,
                        getRobotCentricPowers(forward,lateral,heading),
                        holdAngle);
        follower.manual(powers);
    }

    /**
     * Tells the follower to begin following the path provided.
     * @param path The path that the Drivetrain should follow.
     */
    public void followPath(Path path){
        follower.follow(path);
    }

    /**
     * Creates a path based on the closest allowed shoot pose and hive Pose.
     * The path is created using line so it will not be curved.
     * The path also called facingPoint so that it will constantly face the hive throughout the path.
     * @return A Line Path to the closest shoot pose with a facing point of the Hive.
     */
    public Path getPathToShootPose(){
        Pose[] shootPoses = getShootPoses();
        Pose[] hivePoses = getHivePoses();
        Pose shootPose = closestPose(follower.pose(), shootPoses);
        Pose hivePose = closestPose(shootPose, hivePoses);
        return line(follower.pose(),shootPose).facingPoint(hivePose);
    }

    /**
     * Gets the allowed Hive poses based on the current alliance.
     * If no Alliance color is present a list of all hive poses is returned
     * @return A Pose[] of the hive poses corresponding to the Alliance color.
     */
    private Pose[] getHivePoses(){
        switch (allianceColor){
            case RED:
                return redHivePoses;
            case BLUE:
                return blueHivePoses;
            default:
                return allHivePoses;
        }
    }

    /**
     * Gets the allowed Shoot poses based on the current alliance.
     * If no Alliance color is present a list of all shoot poses is returned
     * @return A Pose[] of the shoot poses corresponding to the Alliance color.
     */
    private Pose[] getShootPoses(){
        switch (allianceColor){
            case RED:
                return redShootPoses;
            case BLUE:
                return blueShootPoses;
            default:
                return allShootPoses;
        }
    }

    /**
     * A wrapper method for {@link #followPath(Path)} if Path is {@link #getPathToShootPose()}
     */
    public void pathToShootPose() {
        follower.follow(getPathToShootPose());
    }

    /**
     * Returns the pose with an array that is closest to the provided pose
     * @param pose The pose that will be checked against.
     * @param poses The Pose[] that contains all the poses that should be checked.
     * @return The pose within {@code poses} that is closest to {@code pose}
     */
    private Pose closestPose(Pose pose, @NonNull Pose[] poses){
        double[] distances = new double[poses.length];
        for (int i = 0; i < poses.length; i++) {
            distances[i] = pose.distance(poses[i]);
        }
        int minInd=0;
        double minDist = distances[0];
        for(int i = 1; i < distances.length; i++){
            if(distances[i] < minDist){
                minInd=i;
                minDist=distances[i];
            }
        }
        return poses[minInd];
    }

    /**
     * Stops the drivetrain by calling stop() on the current follower.
     */
    public void stop(){
        follower.stop();
    }

    /**
     * Returns a command that drives the robot using Robot oriented arcade drive.
     * The Command will never finish on its own.
     * @param forward A DoubleSupplier that provides a -1 to 1 value that controls the robot relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param heading A DoubleSupplier that provides a -1 to 1 value that controls the counterclockwise/clockwise rotation effort.
     *                Positive is counterclockwise movement.
     * @return A Command that continuously calls {@link #manualArcade(double, double)} with the provided parameters.
     * This command will not finish automatically.
     */
    public Command manualArcadeCommand(DoubleSupplier forward, DoubleSupplier heading){
        return infinite(()->{
            manualArcade(forward.getAsDouble(),heading.getAsDouble());
        }).setEnd((endCondition)-> {
            stop();
        }).requiring(this);
    }

    /**
     * Returns a command that drives the robot using Robot oriented Mecanum drive.
     * @param forward A DoubleSupplier that provides a -1 to 1 value that controls the robot relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral A DoubleSupplier that provides a -1 to 1 value that controls the robot relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading A DoubleSupplier that provides a -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @return A Command that continuously calls {@link #manualRobotOriented(double, double, double)} with the provided parameters.
     * This command will not finish automatically.
     */
    public Command manualRobotOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading){
        return infinite(()->{
            manualRobotOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    /**
     * Returns a command that drives the robot using Field oriented Mecanum drive.
     * @param forward A DoubleSupplier that provides a -1 to 1 value that controls the field relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral A DoubleSupplier that provides a -1 to 1 value that controls the field relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading A DoubleSupplier that provides a -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @param angle A DoubleSupplier that provides the current angle of the robot in Radians
     * @return A Command that continuously calls {@link #manualFieldOriented(double, double, double, double)} with the provided parameters.
     * This command will not finish automatically.
     */
    public Command manualFieldOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading, DoubleSupplier angle){
        return infinite(()->{
            manualFieldOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble(),angle.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    /**
     * Returns a command that drives the robot using Field oriented Mecanum drive, using the current heading of the follower.
     * @param forward A DoubleSupplier that provides a -1 to 1 value that controls the field relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral A DoubleSupplier that provides a -1 to 1 value that controls the field relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading A DoubleSupplier that provides a -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @return A Command that continuously calls {@link #manualFieldOriented(double, double, double)} with the provided parameters.
     * This command will not finish automatically.
     */
    public Command manualFieldOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading){
        return infinite(()->{
            manualFieldOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    /**
     * Returns a command that drives the robot using Field relative hold angle Mecanum drive.
     * @param forward A DoubleSupplier that provides a -1 to 1 value that controls the field relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral A DoubleSupplier that provides a -1 to 1 value that controls the field relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading A DoubleSupplier that provides a -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @param currentAngle A DoubleSupplier that provides the current angle of the robot in Radians.
     * @param holdAngle A DoubleSupplier that provides the Field Relative angle in radians that the robot should hold.
     * @return A Command that continuously calls {@link #manualHoldAngleFieldOriented(double, double, double, double, double)} with the provided parameters.
     * This command will not finish automatically.
     */
    public Command manualHoldAngleFieldOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading, DoubleSupplier currentAngle, DoubleSupplier holdAngle){
        return infinite(()-> {
            manualHoldAngleFieldOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble(),currentAngle.getAsDouble(),holdAngle.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    /**
     * Returns a command that drives the robot using Field relative hold angle Mecanum drive.
     * @param forward A DoubleSupplier that provides a -1 to 1 value that controls the field relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral A DoubleSupplier that provides a -1 to 1 value that controls the field relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading A DoubleSupplier that provides a -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @param holdAngle A DoubleSupplier that provides the Field Relative angle in radians that the robot should hold.
     * @return A Command that continuously calls {@link #manualHoldAngleFieldOriented(double, double, double, double)} with the provided parameters.
     * This command will not finish automatically.
     */
    public Command manualHoldAngleFieldOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading, DoubleSupplier holdAngle){
        return infinite(()-> {
            manualHoldAngleFieldOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble(),holdAngle.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    /**
     * NOT RECOMMENDED: Tends to be very disorienting.
     * Returns a command that drives the robot using Robot relative hold angle Mecanum drive.
     * @param forward A DoubleSupplier that provides a -1 to 1 value that controls the robot relative longitudinal forward/backward effort.
     *                Positive is forward movement.
     * @param lateral A DoubleSupplier that provides a -1 to 1 value that controls the robot relative lateral left/right effort.
     *                Positive is leftward movement.
     * @param heading A DoubleSupplier that provides a -1 to 1 value that controls the rotation counterclockwise/clockwise effort.
     *                Positive is counterclockwise movement.
     * @param holdAngle A DoubleSupplier that provides the FIELD RELATIVE angle in radians that the robot should hold.
     * @return A Command that continuously calls {@link #manualHoldAngleRobotOriented(double, double, double, double)} with the provided parameters.
     * This command will not finish automatically.
     */
    public Command manualHoldAngleRobotOrientedCommand(DoubleSupplier forward, DoubleSupplier lateral, DoubleSupplier heading, DoubleSupplier holdAngle){
        return infinite(()-> {
            manualHoldAngleRobotOriented(forward.getAsDouble(),lateral.getAsDouble(),heading.getAsDouble(),holdAngle.getAsDouble());
        }).setEnd((endCondition)->{
            stop();
        }).requiring(this);
    }

    /**
     * Returns a command of the robot following a path.
     * When this command is scheduled it will begin the path.
     * This command will finish when the path is at its parametric end
     * @param path The path that the command will be constructed off of.
     * @return A Command that will begin a path when scheduled and end when that path is at its parametric end.
     */
    public Command followPathCommand(Path path){
        return new CommandBuilder()
                .setStart(()->follower.follow(path))
                .setDone(follower::atParametricEnd)
                .setEnd((endCondition) -> follower.stop())
                .requiring(this);
    }

    /**
     * This is a Wrapper command for {@link #followPathCommand(Path)} where Path is {@link #pathToShootPose()}
     * @return A command that paths to the shoot pose
     */
    public Command pathToShootPoseCommand(){
        return followPathCommand(getPathToShootPose());
    }

    /**
     * Sets the default command for this mechanism.
     * @param command The command that should be set as the default command of this subsystem.
     * @return this command, so calls can be chained.
     */
    public Command setDefaultCommand(Command command){
        this.defaultCommand = command;
        return command;
    }

    @NonNull
    @Override
    public Command getDefaultCommand() {
        return defaultCommand;
    }

    @Override
    public void periodic(){
        follower.update();
    }

    /**
     * Returns the closest hive pose allowed to the provided pose.
     * @param pose The pose that should be checked against
     * @return The closest hive pose to {@code pose}
     */
    public Pose closestHivePose(Pose pose){
        Pose[] hivePoses= getHivePoses();
        return closestPose(pose,hivePoses);
    }

    /**
     * Returns the distance to the closest allowed hive, using the robots current position
     * @return A Distance unit of the distance between the followers current pose and the closest allowed hive.
     */
    public Distance distanceToHive(){
        return Inches.of(closestHivePose(follower.pose()).distance(follower.pose()));
    }

    /**
     * Returns the distance to the closest allowed hive, using the provided pose.
     * @return A Distance unit of the distance between {@code pose} and the closest allowed hive.
     */
    public Distance distanceToHive(Pose pose){
        return Inches.of(closestHivePose(pose).distance(pose));
    }
}
