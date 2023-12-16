// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  SwerveDrivetrain swerveDrivetrain;
  DoubleSupplier strafeSupplier;
  DoubleSupplier rotationSupplier;
  DoubleSupplier translationSupplier;//todo make better name

  public Drive(SwerveDrivetrain swerveDrivetrain, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, DoubleSupplier translationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrivetrain);
    this.translationSupplier = translationSupplier;
    this.rotationSupplier = rotationSupplier;
    this.strafeSupplier = strafeSupplier;
    this.swerveDrivetrain = swerveDrivetrain;
    SmartDashboard.putData("Teleop Drive", this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO add Deadband, add Govenor
    swerveDrivetrain.drive(new Translation2d(getStrafe(), getTranslation()), getRotation());
    
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
    return Math.abs(strafeSupplier.getAsDouble()) < 0.15 ? 0 : strafeSupplier.getAsDouble()*0.5;
  }

  private double getTranslation() {
    //return 0;
    return Math.abs(translationSupplier.getAsDouble()) < 0.15 ? 0 : translationSupplier.getAsDouble()*0.5;
  }

  private Rotation2d getRotation() {
    //return new Rotation2d(0);
    return new Rotation2d(Math.abs(rotationSupplier.getAsDouble()) < 0.15 ? 0 : rotationSupplier.getAsDouble()*.5);
  }

  private double getRotationRaw() {
    return getRotation().getDegrees();
  }
}
