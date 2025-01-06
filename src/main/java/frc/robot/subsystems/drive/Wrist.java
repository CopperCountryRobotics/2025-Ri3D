// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;


public class Wrist extends SubsystemBase {
    private final CANSparkMax wristMotor = new CANSparkMax(0, MotorType.kBrushless);



  public Wrist() {
    wristMotor.configFactoryDefault();
    wristMotor.getEncoder().setPosition(0);
     
   //wristMotor.setPosition(180);

  }

  public void setSpeed(double speed) {
    wristMotor.set(speed);
  }

  public void stop() {
    wristMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}