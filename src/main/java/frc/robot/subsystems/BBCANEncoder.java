// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.interfaces.IEncoder;
import frc.robot.subsystems.SwerveModule.Constants;

/** Add your docs here. */
public class BBCANEncoder implements IEncoder{
    CANCoder encoder;
    BBCANEncoder(Constants constants){

        encoder = new CANCoder(constants.absEncoderID);
        encoder.configSensorDirection(true);
        //encoder.configMagnetOffset(0);
        encoder.configMagnetOffset(constants.absEncoderOffset);
        //Offset Is ALways In Degrees
        
        encoder.configFeedbackCoefficient(0.00153416806, "radians", SensorTimeBase.PerSecond);
    }


    @Override
    public int getTest(double tomatoes) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRawValue() {
        // TODO Auto-generated method stub
        return encoder.getAbsolutePosition();
    }

    @Override
    public Rotation2d getAngle() {
        // TODO Auto-generated method stub
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }
    
}
