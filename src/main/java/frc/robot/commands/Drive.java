// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  SwerveDrivetrain swerveDrivetrain;
  DoubleSupplier strafeSupplier;
  DoubleSupplier rotationSupplier;
  DoubleSupplier translationSupplier;//todo make better name
  Double governor;
  CommandJoystick cJoystick;

  public Drive(SwerveDrivetrain swerveDrivetrain, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, DoubleSupplier translationSupplier, CommandJoystick commandJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrivetrain);
    this.translationSupplier = translationSupplier;
    this.rotationSupplier = rotationSupplier;
    this.strafeSupplier = strafeSupplier;
    this.swerveDrivetrain = swerveDrivetrain;
    this.cJoystick = commandJoystick;
    governor = (double) 1;
    SmartDashboard.putData("Teleop Drive", this);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO add Deadband, add Govenor
    
    governor = (-(cJoystick.getThrottle()-1)/2);
    
    swerveDrivetrain.drive(new Translation2d(getStrafe(), getTranslation()).times(governor*Constants.RobotConstants.driveMaxVelo), getRotation()*(governor*Constants.RobotConstants.rotationMaxAngleVelo));
    SmartDashboard.putNumber("Drive Governor", governor);
    SmartDashboard.putNumber("Pigeon Angle", swerveDrivetrain.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DriveCommand");
    builder.addDoubleProperty("Strafe: ", this::getStrafe, null);
    builder.addDoubleProperty("Translation: ", this::getTranslation, null);
    builder.addDoubleProperty("Rotation: ", this::getRotationRaw, null);
  }

  private double getStrafe() {
    //return 0;
    return MathUtil.applyDeadband(strafeSupplier.getAsDouble(), 0.15);
   // return Math.abs(strafeSupplier.getAsDouble()) < 0.15 ? 0 : (strafeSupplier.getAsDouble()*governor);
  }

  private double getTranslation() {
    //return 0;
    return MathUtil.applyDeadband(translationSupplier.getAsDouble(), 0.15);
    //return Math.abs(translationSupplier.getAsDouble()) < 0.15 ? 0 : translationSupplier.getAsDouble()*governor;
  }

  private double getRotation() {
    //return new Rotation2d(0);
    return MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.15);
    //return(Math.abs((rotationSupplier.getAsDouble()) < 0.15 ? 0 : rotationSupplier.getAsDouble()*governor));
  }

  private double getRotationRaw() {
    return getRotation();
  }
}
