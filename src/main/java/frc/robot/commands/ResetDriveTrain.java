// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class ResetDriveTrain extends CommandBase {
  /** Creates a new ResetDriveTrain. */
  SwerveDrivetrain sDrivetrain;
  public ResetDriveTrain(SwerveDrivetrain sDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sDrivetrain = sDrivetrain;
    addRequirements(sDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sDrivetrain.reset();;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

// Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
