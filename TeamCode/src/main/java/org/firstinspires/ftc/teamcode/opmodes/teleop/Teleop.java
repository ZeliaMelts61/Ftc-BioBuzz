package org.firstinspires.ftc.teamcode.opmodes.teleop;


import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.robot.ZeliaRobot;


import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import dev.nextftc.robot.triggers.Trigger;

@NextTeleop(name = "Teleop")
public class Teleop extends NextOpMode {
    private final ZeliaRobot robot;

    public Teleop(ZeliaRobot robot) {
        super(robot);
        this.robot = robot;
        Scheduler.reset();
    }

    @Override
    public void start() {
        Trigger.Companion.getDefaultEventLoop().clear();

        CommandGamepad gp1 = new CommandGamepad(gamepad1);
        CommandGamepad gp2 = new CommandGamepad(gamepad2);


        robot.init(); // MUST CALL BEFORE RUNNING STUFF
        robot.startDrive(gamepad1);


        // Intake while the Left Bumper is being held
        gp1.leftBumper()
                .onTrue(robot.getIntake().intakeCommand())
                .onFalse(robot.getIntake().stopIntakeCommand());

        // Shoot with just a rpm with the right bumper held down
        gp1.rightBumper()
                .whileTrue(robot.shootAtRPM(1250));

        // Turn to target (whilst still allowing movement) and shoot while the Right Trigger is being held
        gp1.rightTrigger().isOver(0.2).whileTrue(robot.turnToTargetAndShoot(gamepad1));

        // Shoot normally (without moving the drivetrain) while the left trigger is being held
        gp1.leftTrigger().isOver(0.2).whileTrue(robot.shoot());

        // AutoDrive™ to the shoot pose and shoot. (only while A is being held)
        gp1.a().whileTrue(robot.driveToShootSpotThenShoot());

        // Reset Heading
        gp1.start().onTrue(instant(()->robot.getFollower().localizer.setHeading(0)));

        // The UH-OH something got stuck.™ Rotates everything backward while the back button is being held
        gp1.back().whileTrue(robot.runEverythingReverse());

        // Runs the intake backward while D-pad down is held
        gp1.dpadDown()
                .onTrue(robot.getIntake().outtakeCommand())
                .onFalse(robot.getIntake().stopIntakeCommand());

        // Runs the windmill backward while D-pad right is held
        gp1.dpadRight()
                .onTrue(robot.getWindmill().reverseCommand())
                .onFalse(robot.getWindmill().stopCommand());

        // Runs the windmill forward while D-pad left is held
        gp1.dpadLeft()
                .onTrue(robot.getWindmill().forwardCommand())
                .onFalse(robot.getWindmill().stopCommand());


        gp1.start().onTrue(instant(() -> robot.getDrivetrain().setDriverHeadingOffset(robot.getFollower().pose().heading())));


    }

    @Override
    public void periodic() {

    }

    @Override
    public void end() {
        robot.getIntake().stop();
        robot.getFlywheel().deactivate();
        robot.getWindmill().stop();
        robot.getDrivetrain().stop();
    }
}
