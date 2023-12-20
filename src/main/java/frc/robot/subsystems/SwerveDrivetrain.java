// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new SwerveDrivetrain. */

  public final SwerveModule frontRightModule;// = new SwerveModule(Constants.RobotConstants.frontRight);
  public final SwerveModule backLeftModule;// = new SwerveModule(Constants.RobotConstants.backLeft);
  public final  SwerveModule backRightModule;// = new SwerveModule(Constants.RobotConstants.backRight);
  public final SwerveModule frontLeftModule;// = new SwerveModule(Constants.RobotConstants.frontLeft);

  public final WPI_Pigeon2 pigeon2 = new WPI_Pigeon2(50);
  public SwerveDriveOdometry sOdometry;
  public SwerveDriveKinematics sKinematics;
  public SwerveModule[] modules;
  public ChassisSpeeds cSpeeds;

  public SwerveDrivetrain() {

    frontRightModule = new SwerveModule(Constants.RobotConstants.frontRight);
    backLeftModule = new SwerveModule(Constants.RobotConstants.backLeft);
    backRightModule = new SwerveModule(Constants.RobotConstants.backRight);
    frontLeftModule = new SwerveModule(Constants.RobotConstants.frontLeft);

    pigeon2.reset();
    
    cSpeeds = new ChassisSpeeds(0, 0, 0);
    sKinematics = new SwerveDriveKinematics(new Translation2d(Constants.RobotConstants.trackWidth/2, Constants.RobotConstants.trackLength/2),
                                            new Translation2d(Constants.RobotConstants.trackWidth/2, -Constants.RobotConstants.trackLength/2),
                                            new Translation2d(-Constants.RobotConstants.trackWidth/2, Constants.RobotConstants.trackLength/2),
                                            new Translation2d(-Constants.RobotConstants.trackWidth/2, -Constants.RobotConstants.trackLength/2));
    modules = new SwerveModule[]{backLeftModule, backRightModule, frontLeftModule, frontRightModule};
    sOdometry = new SwerveDriveOdometry(sKinematics, getGyroRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));
    pigeon2.getAngle();
  }

  private Rotation2d getGyroRotation2d(){
    return Rotation2d.fromDegrees(pigeon2.getYaw()); 
  }

  private SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] sPosition = new SwerveModulePosition[4];
    for(SwerveModule module : modules){
      //todo remove when module is back
      //if (module.operationOrderID != Constants.RobotConstants.backLeft.position)
      sPosition[module.operationOrderID] = module.getModulePosition();
    }
    return sPosition;
  }


  public void drive(Translation2d desiredTranslation, Rotation2d desiredRotation){
    
    //cSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(t2D.getX(), t2D.getY(), desiredRotation.getDegrees(), getGyroRotation2d());
    //SwerveModuleState[] states = sKinematics.toSwerveModuleStates(cSpeeds);
  
   // cSpeeds = new ChassisSpeeds(t2D.getX(), t2D.getY() , desiredRotation.getRadians());
    cSpeeds.vxMetersPerSecond = desiredTranslation.getX();
    cSpeeds.vyMetersPerSecond = desiredTranslation.getY();
    cSpeeds.omegaRadiansPerSecond = desiredRotation.getRadians();
    SwerveModuleState[] states = sKinematics.toSwerveModuleStates(cSpeeds);
    SmartDashboard.putNumber("Desired X", desiredTranslation.getX());
    SmartDashboard.putNumber("Desired Y", desiredTranslation.getY());
    SmartDashboard.putNumber("Desired Rotation: ", desiredRotation.getDegrees());
    SmartDashboard.putNumber("cSpeeds Y", cSpeeds.vyMetersPerSecond);
    
    for(SwerveModule module : modules){
      module.setState(states[module.operationOrderID]);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sOdometry.update(getGyroRotation2d(), getModulePositions());
    SmartDashboard.putNumber("Gyro Angle", getGyroRotation2d().getDegrees());
    SmartDashboard.putNumber("Odometry pose X: ", sOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry pose Y: ", sOdometry.getPoseMeters().getY());

  }

/*
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DriveTraingo");
    builder.addDoubleProperty("PigeonYaw", this::getPigeonYaw, null);
    builder.addDoubleArrayProperty("Odometry: ", this::getOdometry, null);
    builder.addDoubleArrayProperty("Chassis Speed: ", this::getChassisSpeed, null);
  }

  */

  private double[] getChassisSpeed() {
    return new double[] {cSpeeds.vxMetersPerSecond, cSpeeds.vyMetersPerSecond, cSpeeds.omegaRadiansPerSecond};
  }

  private double[] getOdometry() {
    return new double[] {sOdometry.getPoseMeters().getX(), sOdometry.getPoseMeters().getY()};
  }

  public CommandBase getResetCommand(){
    return this.runOnce(() -> reset());

  }

  public double getAbsoluteEncoder(){
    return backRightModule.getAbsoluteEncoder();
  }

  public void enableBrakes(){
    for(SwerveModule module: modules){
      if(module != null)
      //Brakes not working look here
      module.toggleBrakes(true);
    }
  }

  public void unenableBrakes(){
    for(SwerveModule module: modules){
      if(module != null)
      module.toggleBrakes(false);
      }
    }

  public void reset() {
    pigeon2.reset();
    sOdometry.resetPosition(pigeon2.getRotation2d(), getModulePositions(), new Pose2d());
    frontLeftModule.steerEncoder.setPosition(0);
    frontRightModule.steerEncoder.setPosition(0);
    backLeftModule.steerEncoder.setPosition(0);
    backRightModule.steerEncoder.setPosition(0);
    cSpeeds.omegaRadiansPerSecond = 0;
    cSpeeds.vxMetersPerSecond = 0;
    cSpeeds.vyMetersPerSecond = 0;
  }

  private double getPigeonYaw(){
    return pigeon2.getYaw();
  }

  public void setupSmartDashboard() {
    SmartDashboard.putData("DriveTrain", this);
    for(SwerveModule module : modules){
  //    module.setupSmartDashboard();
    }
  }

public double getAngle() {
    return pigeon2.getYaw();
}
}
