// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveModule.Constants.encoderType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
  }

  
  public static class RobotConstants{
    public static final double trackWidth = Units.inchesToMeters(24.5);
    public static final double trackLength = Units.inchesToMeters(24.5);
    public static final double driveGearRatio = 6.67;
    public static final double steerGearRatio = 1.2;
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveMaxVelo = (5676/60/steerGearRatio)*(wheelDiameter)*Math.PI;
    public static final double rotationMaxAngleVelo = 2*Math.PI;
    //Random numberdecided make actually number that the angle velo max actually is. 
/*
    // Practice Robot
    public static final SwerveModule.Constants frontLeft = new SwerveModule.Constants(0, 10, 20, 0, "Front Left", encoderType.Spark);
    public static final SwerveModule.Constants frontRight = new SwerveModule.Constants(1, 12, 22, 0, "Front Right", encoderType.Spark);
    public static final SwerveModule.Constants backLeft = new SwerveModule.Constants(2, 11, 21, 0, "Back Left", encoderType.Spark);
    public static final SwerveModule.Constants backRight = new SwerveModule.Constants(3, 13, 23, 0, "Back Right", encoderType.Spark);
 */
    // Competition Robot
    public static final SwerveModule.Constants frontLeft = new SwerveModule.Constants(2, 17, 16, 60,22.239, "Front Left", encoderType.CAN);
    public static final SwerveModule.Constants frontRight = new SwerveModule.Constants(3, 31, 2, 59,225.115, "Front Right", encoderType.CAN);
    public static final SwerveModule.Constants backLeft = new SwerveModule.Constants(0, 18, 19, 61,56.433, "Back Left", encoderType.CAN);
    public static final SwerveModule.Constants backRight = new SwerveModule.Constants(1, 3, 30, 62,205.952, "Back Right", encoderType.CAN);
  }


}
