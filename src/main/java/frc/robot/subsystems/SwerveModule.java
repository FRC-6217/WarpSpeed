// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IEncoder;
import frc.robot.subsystems.SwerveModule.Constants.encoderType;

public class SwerveModule extends SubsystemBase{
  /** Creates a new SwerveModule. */

  CANSparkMax driveMotor;
  CANSparkMax steerMotor;
  RelativeEncoder driveEncoder;
  RelativeEncoder steerEncoder;
  SparkMaxPIDController drivePID;
  double driveSetpoint = 0;
  SparkMaxPIDController steerPID;
  double steerSetpoint = 0;
  IEncoder absEncoder;
  String name;
  int operationOrderID;

  public SwerveModule(Constants constants){
    driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    steerMotor = new CANSparkMax(constants.steerMotorID, MotorType.kBrushless);
    if(constants.type == encoderType.CAN){
      absEncoder = new BBCANEncoder(constants.absEncoderID);
    }else if(constants.type == encoderType.Spark){
      absEncoder = new BBAbsoluteEncoder(steerMotor);
    }
    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getEncoder();
    drivePID = driveMotor.getPIDController();
    steerPID = steerMotor.getPIDController();

    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    drivePID.setP(0);
    drivePID.setD(0);
    drivePID.setI(0);
    drivePID.setFF(0);

    steerPID.setP(0);
    steerPID.setD(0);
    steerPID.setI(0);
    steerPID.setFF(0);
    name = new String(constants.name);
    operationOrderID = constants.position;
    SmartDashboard.putData(name, this);
  }
  
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module");
    builder.addDoubleProperty("DriveSpeed", this::getDriveSpeed, null);
    builder.addDoubleProperty("DrivePosition", this::getDrivePosition, null);
    builder.addDoubleProperty("SteerSpeed", this::getSteerSpeed, null);
    builder.addDoubleProperty("SteerPosition", this::getSteerPosition, null);
    builder.addDoubleProperty("DriveP", this::getDriveP, this::setDriveP);
    builder.addDoubleProperty("DriveI", this::getDriveI, this::setDriveI);
    builder.addDoubleProperty("DriveD", this::getDriveD, this::setDriveD);
    builder.addDoubleProperty("SteerP", this::getSteerP, this::setSteerP);
    builder.addDoubleProperty("SteerI", this::getSteerI, this::setSteerI);
    builder.addDoubleProperty("SteerD", this::getSteerD, this::setSteerD);
    builder.addDoubleProperty(" AngleState", this::getSetPointAngle, null);
    builder.addDoubleProperty(" SpeedState", this::getSetPointSpeed, null);
  }

  private double getSetPointAngle() {
    return steerSetpoint;
  }
  
  private double getSetPointSpeed() {
    return driveSetpoint;
  }

  private double getDriveSpeed(){
    return driveEncoder.getVelocity();
  }

  private double getDrivePosition(){
    return driveEncoder.getPosition();
  }

  private double getSteerSpeed(){
    return steerEncoder.getPosition();
  }
  private double getSteerPosition(){
    return steerEncoder.getPosition();
  }

  private double getDriveP(){
    return drivePID.getP();
  }

  private double getDriveI(){
    return drivePID.getI();
  }

  private double getDriveD(){
    return drivePID.getD();
  }

  private void setDriveP(double p){
   drivePID.setP(p);
  }

  private void setDriveI(double i){
   drivePID.setI(i);
  }

  private void setDriveD(double d){
   drivePID.setD(d);
  }
  
  private double getSteerP(){
    return steerPID.getP();
  }

  private double getSteerI(){
    return steerPID.getI();
  }

  private double getSteerD(){
    return steerPID.getD();
  }

  private void setSteerP(double p){
    steerPID.setP(p);
  }

  private void setSteerI(double i){
   steerPID.setI(i);
  }

  private void setSteerD(double d){
   steerPID.setD(d);
  }

  public SwerveModulePosition getModulePosition(){
    return new SwerveModulePosition(getDistance(), absEncoder.getAngle());
  }

  private double getDistance() {
    return driveEncoder.getPosition();
  }
  
  private void setSpeed(SwerveModuleState state){
    driveSetpoint = state.speedMetersPerSecond;
    driveMotor.set(0);
  }

  private void setAngle(SwerveModuleState state){
    steerSetpoint = state.angle.getDegrees();
    steerPID.setReference(0, ControlType.kPosition);
  }
  public void setState(SwerveModuleState state){
    setAngle(state);
    setSpeed(state);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static class Constants{
    public int position;
    public int driveMotorID;
    public int steerMotorID;
    public int absEncoderID;
    public enum encoderType {
      CAN,
      Spark
    }
    public encoderType type;
    public String name;
    public Constants(int position, int driveMotorID, int steerMotorID, int absEncoderID, String name, encoderType type) {
      this.position = position;
      this.driveMotorID = driveMotorID;
      this.steerMotorID = steerMotorID;
      this.absEncoderID = absEncoderID;
      this.name = name;
      this.type = type;
    }
    
  }
}
