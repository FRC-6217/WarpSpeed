// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.interfaces.IEncoder;

/** Add your docs here. */
public class BBAbsoluteEncoder implements IEncoder{
    SparkMaxAbsoluteEncoder encoder;
    BBAbsoluteEncoder(CANSparkMax motor){
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    }

    @Override
    public int getTest(double tomatoes) {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public double getRawValue() {
        // TODO Auto-generated method stub
        return encoder.getPosition();
    }

    @Override
    public Rotation2d getAngle() {
        // TODO Auto-generated method stub
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

}
