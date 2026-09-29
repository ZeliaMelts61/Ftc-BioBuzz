package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.data.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.auto.paths.PathsAndPoses;
import org.firstinspires.ftc.teamcode.robot.ZeliaRobot;

public class Routines {
    private ZeliaRobot robot;
    private final Follower follower;
    private final PathsAndPoses paths;
    private final AutoCommands commands;

    public Routines(ZeliaRobot robot, PathsAndPoses paths, AutoCommands commands){
        this.robot = robot;
        follower = robot.getFollower();
        this.paths = paths;
        paths.mirrorPose(robot.getAlliance());
        this.commands = commands;
    }

//    public Command leaveAuto(Alliance alliance) {
//        paths.mirrorPose(alliance);
//        return sequential(
//                commands.runPath(paths.startPos_to_leavePos())
//        );
//    }

    public Command preloadShootNoPickupAuto(Alliance alliance) {
        paths.mirrorPose(alliance);
        return sequential(
                robot.setupShoot(),
                robot.getDrivetrain().followPathCommand(paths.startFlowerToShootPoseFirst()),
                robot.shoot()
                        .raceWith(waitMs(10000))
        );
    }

    public Command full2CycleAuto(Alliance alliance) {
        paths.mirrorPose(alliance);
        return sequential(
                robot.setupShoot(),
                robot.getDrivetrain().followPathCommand(paths.startFlowerToShootPoseFirst()),
                robot.shoot()
                        .raceWith(waitMs(5000)),
                robot.getDrivetrain().followPathCommand(paths.shoot1ToGardenLineup()),
                robot.getDrivetrain().followPathCommand(paths.gardenLineUpToGarden())
                        .with(robot.getIntake().intakeCommand()),
                robot.getDrivetrain().followPathCommand(paths.gardenLineUpToGarden())
                        .with(robot.continuousFlywheelSpeedControl()),
                robot.runShoot()
                        .raceWith(waitMs(10000))
        );
    }

    public Command full1CycleAndPickupAuto(Alliance alliance){
        paths.mirrorPose(alliance);
        return sequential(
                robot.setupShoot(),
                robot.getDrivetrain().followPathCommand(paths.startFlowerToShootPoseFirst()),
                robot.shoot()
                        .raceWith(waitMs(5000)),
                robot.getDrivetrain().followPathCommand(paths.shoot1ToGardenLineup()),
                robot.getDrivetrain().followPathCommand(paths.gardenLineUpToGarden())
                        .with(robot.getIntake().intakeCommand())
        );

    }

}
