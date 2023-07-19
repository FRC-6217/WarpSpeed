// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */

  CANSparkMax driveMotor;
  CANSparkMax steerMotor;
  RelativeEncoder driveEncoder;
  RelativeEncoder steerEncoder;
  SparkMaxPIDController drivePID;
  SparkMaxPIDController steerPID;
  CANCoder absEncoder;
  String name;

  public SwerveModule(Constants constants) {
    driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    steerMotor = new CANSparkMax(constants.steerMotorID, MotorType.kBrushless);
    absEncoder = new CANCoder(constants.absEncoderID);
    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getEncoder();
    drivePID = driveMotor.getPIDController();
    steerPID = steerMotor.getPIDController();
    name = new String(constants.name);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static class Constants{
    public int driveMotorID;
    public int steerMotorID;
    public int absEncoderID;
    public String name;
    public Constants(int driveMotorID, int steerMotorID, int absEncoderID, String name) {
      this.driveMotorID = driveMotorID;
      this.steerMotorID = steerMotorID;
      this.absEncoderID = absEncoderID;
      this.name = name;
    }
    
  }
}
