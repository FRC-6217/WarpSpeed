// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new SwerveDrivetrain. */

  public SwerveModule frontLeftModule = new SwerveModule(Constants.RobotConstants.frontLeft);
  public SwerveModule frontRightModule = new SwerveModule(Constants.RobotConstants.frontRight);
  public SwerveModule backLeftModule = new SwerveModule(Constants.RobotConstants.backLeft);
  public SwerveModule backRightModule = new SwerveModule(Constants.RobotConstants.backRight);

  public SwerveDrivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
