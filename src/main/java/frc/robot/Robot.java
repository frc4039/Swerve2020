/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final XboxController stick = new XboxController(1);
  private final SwerveDriveKinematics dop_swerve = new SwerveDriveKinematics(
      new Translation2d(-8 * 25.4 / 1000, -8 * 25.4 / 1000), new Translation2d(-8 * 25.4 / 1000, 8 * 25.4 / 1000),
      new Translation2d(8 * 25.4 / 1000, -8 * 25.4 / 1000), new Translation2d(8 * 25.4 / 1000, 8 * 25.4 / 1000));

  private final SwerveModule swerve_front_left = new SwerveModule(new CANSparkMax(1, MotorType.kBrushless),
      new CANSparkMax(2, MotorType.kBrushless), new AnalogInput(0), -0.93);

  private final SwerveModule swerve_front_right = new SwerveModule(new CANSparkMax(3, MotorType.kBrushless),
      new CANSparkMax(4, MotorType.kBrushless), new AnalogInput(1), -5.31);

  private final SwerveModule swerve_rear_left = new SwerveModule(new CANSparkMax(5, MotorType.kBrushless),
      new CANSparkMax(6, MotorType.kBrushless), new AnalogInput(2), -4.97);

  private final SwerveModule swerve_rear_right = new SwerveModule(new CANSparkMax(7, MotorType.kBrushless),
      new CANSparkMax(8, MotorType.kBrushless), new AnalogInput(3), -5.03);

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
  }

  /**
   * This function is run once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("front left encoder", swerve_front_left.get_angle_from_encoder());
    SmartDashboard.putNumber("front right encoder", swerve_front_right.get_angle_from_encoder());
    SmartDashboard.putNumber("rear left encoder", swerve_rear_left.get_angle_from_encoder());
    SmartDashboard.putNumber("rear right encoder", swerve_rear_right.get_angle_from_encoder());
  }

  /**
   * This function is called periodically during teleoperated mode.
   */
  @Override
  public void teleopPeriodic() {
    // to use X and B buttons to steer
    double steer_count = 0;
    if (stick.getXButton()) {
      steer_count = steer_count - 2;
    }
    if (stick.getBButton()) {
      steer_count = steer_count + 2;
    }
    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    double joystick_Y_cubed = Math.pow(stick.getY(Hand.kLeft), 3);
    double joystick_X_cubed = Math.pow(stick.getX(Hand.kLeft), 3);
    if (Math.abs(joystick_Y_cubed) < 0.05 && Math.abs(joystick_X_cubed) < 0.05 && steer_count == 0) {
      swerve_front_left.stop_motors();
      swerve_front_right.stop_motors();
      swerve_rear_left.stop_motors();
      swerve_rear_right.stop_motors();
    } else {
      final ChassisSpeeds speeds = new ChassisSpeeds(joystick_Y_cubed, joystick_X_cubed, steer_count);

      // Convert to module states
      final SwerveModuleState[] moduleStates = dop_swerve.toSwerveModuleStates(speeds);

      // Front left module state
      final SwerveModuleState frontLeft = moduleStates[0];
      swerve_front_left.swerve_instruction_to_controller(frontLeft);

      // Front right module state
      final SwerveModuleState frontRight = moduleStates[1];
      swerve_front_right.swerve_instruction_to_controller(frontRight);

      // Back left module state
      final SwerveModuleState rearLeft = moduleStates[2];
      swerve_rear_left.swerve_instruction_to_controller(rearLeft);

      // Back right module state
      final SwerveModuleState rearRight = moduleStates[3];
      swerve_rear_right.swerve_instruction_to_controller(rearRight);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
