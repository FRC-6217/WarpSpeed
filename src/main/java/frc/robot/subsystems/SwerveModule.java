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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.interfaces.IEncoder;
import frc.robot.subsystems.SwerveModule.Constants.encoderType;

public class SwerveModule extends SubsystemBase{
  /** Creates a new SwerveModule. */

  final CANSparkMax driveMotor;
  final CANSparkMax steerMotor;
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
    name = new String(constants.name);
    double startTime = Timer.getFPGATimestamp();
    
    driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    System.out.println(name + " delta1: "+ (Timer.getFPGATimestamp() - startTime));

    steerMotor = new CANSparkMax(constants.steerMotorID, MotorType.kBrushless);
    System.out.println(name + " delta2: "+ (Timer.getFPGATimestamp() - startTime));

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

    steerPID.setP(0.05);
    steerPID.setD(0);
    steerPID.setI(0);
    steerPID.setFF(0);

    SmartDashboard.putNumber(name + "DrivePosition", getDrivePosition());
    
    SmartDashboard.putNumber(name + "DriveSpeed", getDriveSpeed());
    
    
    SmartDashboard.putNumber(name + "SteerSpeed", getSteerSpeed());
    SmartDashboard.putNumber(name + "SteerPosition", getSteerPosition());
    SmartDashboard.putNumber(name + "DriveP", getDriveP());
    SmartDashboard.putNumber(name + "DriveI", getDriveI());

    SmartDashboard.putNumber("DriveD", getDriveD());
    SmartDashboard.putNumber("SteerP", getSteerP());
    SmartDashboard.putNumber("SteerI", getSteerI());
    SmartDashboard.putNumber("SteerD", getSteerD());
    SmartDashboard.putNumber(" AngleState", getSetPointAngle());
    SmartDashboard.putNumber(" SpeedState", getSetPointSpeed());
    operationOrderID = constants.position;
    System.out.println(name + " delta: "+ (Timer.getFPGATimestamp() - startTime));

  }

  @Override
  public void initSendable(SendableBuilder builder) {
   // builder.setSmartDashboardType(name + "Swerve Module");
    builder.addDoubleProperty(name + "DrivePosition", this::getDrivePosition, null);
    
    builder.addDoubleProperty(name + "DriveSpeed", this::getDriveSpeed, null);
    
    
    builder.addDoubleProperty(name + "SteerSpeed", this::getSteerSpeed, null);
    builder.addDoubleProperty(name + "SteerPosition", this::getSteerPosition, null);
    builder.addDoubleProperty(name + "DriveP", this::getDriveP, this::setDriveP);
    builder.addDoubleProperty(name + "DriveI", this::getDriveI, this::setDriveI);

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
  
  //Check SparkMax IDs Why Move when 0???????
  private void setSpeed(SwerveModuleState state){
    driveSetpoint = state.speedMetersPerSecond;
    driveMotor.set(driveSetpoint);
   // driveMotor.set(0);
  }

  private void setAngle(SwerveModuleState state){
    steerSetpoint = state.angle.getDegrees();
    steerPID.setReference(steerSetpoint, ControlType.kPosition);
  }

  public Rotation2d getCurrentAngle(){
    return new Rotation2d(Math.toRadians(getSteerPosition()));
  }
  public void setState(SwerveModuleState state){
    state = optimize(state, getCurrentAngle());

    setAngle(state);
    setSpeed(state);
  }

  public void setupSmartDashboard() {
    SmartDashboard.putData(name, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing in
   * appropriate scope for CTRE and REV onboard control as both controllers as of writing don't have
   * support for continuous input.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
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
