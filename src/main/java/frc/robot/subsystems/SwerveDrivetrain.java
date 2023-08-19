// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new SwerveDrivetrain. */

  public SwerveModule frontLeftModule = new SwerveModule(Constants.RobotConstants.frontLeft);
  public SwerveModule frontRightModule = new SwerveModule(Constants.RobotConstants.frontRight);
  public SwerveModule backLeftModule = new SwerveModule(Constants.RobotConstants.backLeft);
  public SwerveModule backRightModule = new SwerveModule(Constants.RobotConstants.backRight);
  public Pigeon2 pigeon2 = new Pigeon2(50);
  public SwerveDriveOdometry sOdometry;
  public SwerveDriveKinematics sKinematics;
  public SwerveModule[] modules;
  public ChassisSpeeds cSpeeds;

  public SwerveDrivetrain() {
    cSpeeds = new ChassisSpeeds(0, 0, 0);
    sKinematics = new SwerveDriveKinematics(new Translation2d(-Constants.RobotConstants.trackWidth/2, Constants.RobotConstants.trackLength/2),
                                            new Translation2d(Constants.RobotConstants.trackWidth/2, Constants.RobotConstants.trackLength/2),
                                            new Translation2d(-Constants.RobotConstants.trackWidth/2, -Constants.RobotConstants.trackLength/2),
                                            new Translation2d(Constants.RobotConstants.trackWidth/2, -Constants.RobotConstants.trackLength/2));
    modules = new SwerveModule[]{frontLeftModule, frontRightModule, backLeftModule, backRightModule};
    sOdometry = new SwerveDriveOdometry(sKinematics, getGyroRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));
  }

  private Rotation2d getGyroRotation2d(){
    return Rotation2d.fromDegrees(pigeon2.getYaw()); 
  }

  private SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] sPosition = new SwerveModulePosition[4];
    for(SwerveModule module : modules){
      sPosition[module.operationOrderID] = module.getModulePosition();
    }
    return sPosition;
  }

  public void drive(Translation2d t2D, Rotation2d r2D2){
    cSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(t2D.getX(), t2D.getY(), r2D2.getDegrees(), getGyroRotation2d());
    SwerveModuleState[] states = sKinematics.toSwerveModuleStates(cSpeeds);
    for(SwerveModule module : modules){
      module.setState(states[module.operationOrderID]);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sOdometry.update(getGyroRotation2d(), getModulePositions());
  }


  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DriveTraingo");
    builder.addDoubleProperty("PigeonYaw", this::getPigeonYaw, null);
    builder.addDoubleArrayProperty("Odometry: ", this::getOdometry, null);
    builder.addDoubleArrayProperty("Chassis Speed: ", this::getChassisSpeed, null);
  }

  private double[] getChassisSpeed() {
    return new double[] {cSpeeds.vxMetersPerSecond, cSpeeds.vyMetersPerSecond, cSpeeds.omegaRadiansPerSecond};
  }

  private double[] getOdometry() {
    return new double[] {sOdometry.getPoseMeters().getX(), sOdometry.getPoseMeters().getY()};
  }

  private double getPigeonYaw(){
    return pigeon2.getYaw();
  }
}
