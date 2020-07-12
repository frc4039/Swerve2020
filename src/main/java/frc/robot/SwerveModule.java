/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * Add your docs here.
 */
public class SwerveModule {
    private CANSparkMax swerve_fwd_rvs;
    private CANSparkMax swerve_turn;
    private AnalogInput swerve_angle;
    private double offset_angle;
    // PID defaults from SwerveDriveSpecialties
    private PIDController swerve_PID = new PIDController(0.5, 0, 0.0001);

    public SwerveModule(CANSparkMax swerve_fwd_rvs, CANSparkMax swerve_turn, AnalogInput swerve_angle,
            double offset_angle) {
        this.swerve_fwd_rvs = swerve_fwd_rvs;
        this.swerve_turn = swerve_turn;
        this.swerve_angle = swerve_angle;
        this.offset_angle = offset_angle;
        // allows continuous rotation
        this.swerve_PID.enableContinuousInput(0, 2.0 * Math.PI);
    }

    private double angleBetween(double angle1, double angle2) {
        double target = ((angle1 - angle2) + (2 * Math.PI)) % (2 * Math.PI);
    
        return Math.min(target, (2 * Math.PI) - target);
    }

    public void swerve_instruction_to_controller(SwerveModuleState goal) {
        // swerve_fwd_rvs.set(0);
        // swerve_turn.set(0);
        // swerve_turn.set(swerve_PID.calculate(get_angle_from_encoder(), 0));
        double target_speed = goal.speedMetersPerSecond * 0.1;
        double target_angle = goal.angle.getRadians();
        if (target_angle < 0.0) {
            target_angle = target_angle + Math.PI * 2;
        }
        double actual_angle = get_angle_from_encoder();

        // Check if we can do a shorter angle change by reversing the motor.
        double angle_diff = angleBetween(target_angle, actual_angle);
        if(angle_diff > Math.PI / 2) {
            target_angle = (target_angle + Math.PI) % (Math.PI * 2);
            target_speed = -target_speed;
        }
            
        swerve_fwd_rvs.set(target_speed);

        double swerve_angle_output = swerve_PID.calculate(actual_angle, target_angle);
        swerve_angle_output = Math.min(Math.max(swerve_angle_output, -0.25), 0.25);
        swerve_turn.set(swerve_angle_output);
    }

    public double get_angle_from_encoder() {
        double angle = (1.0 - swerve_angle.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;
        angle += offset_angle;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    public void stop_motors() {
        swerve_fwd_rvs.set(0);
        swerve_turn.set(0);
    }
}
