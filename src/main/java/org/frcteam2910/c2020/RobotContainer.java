package org.frcteam2910.c2020;

import edu.wpi.first.wpilibj2.command.*;
import org.frcteam2910.c2020.commands.DriveCommand;
import org.frcteam2910.c2020.commands.*;
import org.frcteam2910.c2020.subsystems.*;
import org.frcteam2910.c2020.util.AutonomousChooser;
import org.frcteam2910.c2020.util.AutonomousTrajectories;
import org.frcteam2910.c2020.common.math.Rotation2;
import org.frcteam2910.c2020.common.math.Vector2;
import org.frcteam2910.c2020.common.robot.input.Axis;
import org.frcteam2910.c2020.common.robot.input.XboxController;

import java.io.IOException;

public class RobotContainer {

    private final XboxController primaryController = new XboxController(Constants.PRIMARY_CONTROLLER_PORT);
    private final XboxController secondaryController = new XboxController(Constants.SECONDARY_CONTROLLER_PORT);

    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem(drivetrainSubsystem);

    private AutonomousTrajectories autonomousTrajectories;
    private final AutonomousChooser autonomousChooser;

    public RobotContainer() {
        try {
            autonomousTrajectories  = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
        } catch (IOException e) {
            e.printStackTrace();
        }
        autonomousChooser = new AutonomousChooser(autonomousTrajectories);

        primaryController.getLeftXAxis().setInverted(true);
        primaryController.getRightXAxis().setInverted(true);

        CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
        CommandScheduler.getInstance().registerSubsystem(visionSubsystem);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        primaryController.getBackButton().whenPressed(
                () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
        );
        

        primaryController.getAButton().whenPressed(
                new BasicDriveCommand(drivetrainSubsystem, new Vector2(-0.5, 0.0), 0.0, false).withTimeout(0.12)
        );


//        secondaryController.getAButton().whenPressed(new PlayTheWheelOfFortuneCommand(wheelOfFortuneSubsystem));
    }

    public Command getAutonomousCommand() {
        return autonomousChooser.getCommand(this);
    }

    private Axis getDriveForwardAxis() {
        return primaryController.getLeftYAxis();
    }

    private Axis getDriveStrafeAxis() {
        return primaryController.getLeftXAxis();
    }

    private Axis getDriveRotationAxis() {
        return primaryController.getRightXAxis();
    }

    public DrivetrainSubsystem getDrivetrainSubsystem() {
        return drivetrainSubsystem;
    }


    public VisionSubsystem getVisionSubsystem() { return  visionSubsystem; }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    public XboxController getSecondaryController() {
        return secondaryController;
    }
}
