package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.opmodes.auto.paths.PathsAndPoses;
import org.firstinspires.ftc.teamcode.robot.ZeliaRobot;

public class Routines {
    private ZeliaRobot robot;
    private final PathsAndPoses paths;
    private final AutoCommands commands;

    public Routines(ZeliaRobot robot){
        this.robot = robot;
        this.paths = new PathsAndPoses(robot.getAlliance());
        this.commands = new AutoCommands(robot, paths);
    }

    public Command preloadShootNoPickupAuto() {
        robot.getFollower().setPose(paths.startRightOfFlower);
        return sequential(
                robot.setupShoot(),
                robot.getDrivetrain().followPathCommand(paths.startFlowerToShootPoseFirst()),
                commands.timedShoot(10000)
        );
    }

    public Command full2CycleAuto() {
        robot.getFollower().setPose(paths.startRightOfFlower);
        return sequential(
                robot.setupShoot(),
                robot.getDrivetrain().followPathCommand(paths.startFlowerToShootPoseFirst()),
                commands.timedShoot(5000),
                robot.getDrivetrain().followPathCommand(paths.shoot1ToGardenLineup()),
                robot.getDrivetrain().followPathCommand(paths.gardenLineUpToGarden())
                        .with(robot.getIntake().intakeCommand()),
                robot.getDrivetrain().followPathCommand(paths.gardenToShootPoseSecond())
                        .with(robot.continuousFlywheelSpeedControl()),
                commands.timedShoot(10000)
        );
    }

    public Command full1CycleAndPickupAuto(){
        robot.getFollower().setPose(paths.startRightOfFlower);
        return sequential(
                robot.setupShoot(),
                robot.getDrivetrain().followPathCommand(paths.startFlowerToShootPoseFirst()),
                commands.timedShoot(5000),
                robot.getDrivetrain().followPathCommand(paths.shoot1ToGardenLineup()),
                robot.getDrivetrain().followPathCommand(paths.gardenLineUpToGarden())
                        .with(robot.getIntake().intakeCommand())
        );

    }

    public Command simpleShootDrive() {
        // sets the starting pose
        robot.getFollower().setPose(paths.againstWallByUpwardsFacingStartingHive);
        return sequential(
                // shoots for 10 seconds
                robot.shootAtRPM(2678)
                        // raceWith cancels both commands when one finishes
                        // In this case it will cancel the shoot command after 10 seconds
                        .raceWith(Commands.waitMs(10000)),

                // Drive backward for .5 seconds
                // You may need to swap this to forward if flywheel side is front
                robot.getDrivetrain().manualArcadeCommand(() -> -0.5, () -> 0)
                        // raceWith cancels both commands when one finishes
                        // In this case it will cancel the drive command after 0.5 seconds
                        .raceWith(Commands.waitMs(500)));
    }
    public Command autoDrivePark() {
        return sequential(
                // sets the starting pose
                instant(()-> robot.getFollower().setPose(paths.againstWallByUpwardsFacingStartingHive)),
                 // shoots for 10 seconds
                robot.shootAtRPM(2678)
                        // raceWith cancels both commands when one finishes
                        // In this case it will cancel the shoot command after 10 seconds
                        .raceWith(Commands.waitMs(10000)),
                robot.getDrivetrain().followPathCommand(paths.startShootToPark())
        );
    }

}
