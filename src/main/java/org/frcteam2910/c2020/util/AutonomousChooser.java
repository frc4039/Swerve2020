package org.frcteam2910.c2020.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.frcteam2910.c2020.RobotContainer;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.common.control.Trajectory;
import org.frcteam2910.c2020.common.math.RigidTransform2;
import org.frcteam2910.c2020.common.math.Rotation2;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private static SendableChooser<AutonomousMode> autonomousModeChooser;

    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.setDefaultOption("5ft Forward", AutonomousMode.FORWARD);
        autonomousModeChooser.addOption("5ft Left, slow", AutonomousMode.LEFT_SLOW);
        autonomousModeChooser.addOption("5ft Left Diagonal", AutonomousMode.DIAGONAL);
        autonomousModeChooser.addOption("5ft Left Diagonal, slow", AutonomousMode.DIAGONAL_ROTATE);
        autonomousModeChooser.addOption("10ft Arc", AutonomousMode.ARC);
        autonomousModeChooser.addOption("Diagonal, arc back to start, go forward", AutonomousMode.COMPLEX);
        autoTab.add("Mode", autonomousModeChooser)
        .withSize(3, 1);
    }

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;
    }

    private SequentialCommandGroup getFiveFootForward(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFiveFootForward());
        follow(command,container, trajectories.getFiveFootForward());
        
        return command;
    }

    private Command getFiveFootLeftSlow(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFiveFootLeftSlow());
        follow(command, container, trajectories.getFiveFootLeftSlow());

        return command;
    }

    private Command getFiveFootDiagonal(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFiveFootDiagonal());
        follow(command, container, trajectories.getFiveFootDiagonal());

        return command;
    }

    public Command getFiveFootDiagonalRotate(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFiveFootDiagonalRotate());
        follow(command, container, trajectories.getFiveFootDiagonalRotate());

        return command;
    }

    public Command getTenFootArc(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFootArc());
        follow(command, container, trajectories.getTenFootArc());

        return command;
    }

    public Command getComplexTrajectory(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getComplexTrajectory());
        follow(command, container, trajectories.getComplexTrajectory());

        return command;
    }

    
    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case FORWARD:
                return getFiveFootForward(container);
            case LEFT_SLOW:
                return getFiveFootLeftSlow(container);
            case DIAGONAL:
                return getFiveFootDiagonal(container);
            case DIAGONAL_ROTATE:
                return getFiveFootDiagonalRotate(container);
            case ARC:
                return getTenFootArc(container);
            case COMPLEX:
                return getComplexTrajectory(container);
        }

        return getFiveFootForward(container);
    }
    

    /*
    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container) {
        shootAtTarget(command, container, 2.5);
    }

    private void shootAtTarget(SequentialCommandGroup command, RobotContainer container, double timeToWait) {
        command.addCommands(
                new TargetWithShooterCommand(container.getShooterSubsystem(), container.getVisionSubsystem(), container.getPrimaryController())
                        .alongWith(new VisionRotateToTargetCommand(container.getDrivetrainSubsystem(), container.getVisionSubsystem(), () -> 0.0, () -> 0.0))
                        .alongWith(
                                new WaitCommand(0.1).andThen(new AutonomousFeedCommand(container.getShooterSubsystem(), container.getFeederSubsystem(), container.getVisionSubsystem())))
                        .withTimeout(timeToWait));
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                .deadlineWith(new TargetWithShooterCommand(container.getShooterSubsystem(), container.getVisionSubsystem(), container.getPrimaryController()))
                .alongWith(new PrepareBallsToShootCommand(container.getFeederSubsystem(), 1.0)));
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getIntakeSubsystem().setExtended(true)));
        command.addCommands(
                new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                        .deadlineWith(
                                new IntakeCommand(container.getIntakeSubsystem(), container.getFeederSubsystem(), -1.0).withTimeout(0.25)
                                        .andThen(
                                                new IntakeCommand(container.getIntakeSubsystem(), container.getFeederSubsystem(), 1.0)
                                                        .alongWith(
                                                                new FeederIntakeWhenNotFullCommand(container.getFeederSubsystem(), 1.0)
                                                        ))));
        command.addCommands(new InstantCommand(() -> container.getIntakeSubsystem().setExtended(false)));
    }
*/

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), Rotation2.ZERO))));
    }

    private enum AutonomousMode {
        FORWARD,
        LEFT_SLOW,
        DIAGONAL,
        DIAGONAL_ROTATE,
        ARC,
        COMPLEX
    }
}
