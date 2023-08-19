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
    public static final int kDriverControllerPort = 0;
  }

  
  public static class RobotConstants{
    public static final double trackWidth = Units.inchesToMeters(19.25);
    public static final double trackLength = Units.inchesToMeters(20.25);
    public static final double driveGearRatio = 6.67;
    public static final double steerGearRatio = 1.2;
    public static final double wheelDiameter = Units.inchesToMeters(4); 
    public static final SwerveModule.Constants frontLeft = new SwerveModule.Constants(0, 13, 23, 0, "Front Left", encoderType.Spark);
    public static final SwerveModule.Constants frontRight = new SwerveModule.Constants(1, 11, 21, 0, "Front Right", encoderType.Spark);
    public static final SwerveModule.Constants backLeft = new SwerveModule.Constants(2, 12, 22, 0, "Back Left", encoderType.Spark);
    public static final SwerveModule.Constants backRight = new SwerveModule.Constants(3, 10, 20, 0, "Back Right", encoderType.Spark);
  }


}
