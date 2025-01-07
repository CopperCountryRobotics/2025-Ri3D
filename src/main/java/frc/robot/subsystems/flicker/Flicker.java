// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flicker;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flicker extends SubsystemBase {

  private PWMTalonSRX wheel;
  private CANSparkMax arm; 
  private SparkPIDController pidControllerArm;
  private RelativeEncoder encoderArm;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kPosition;

    public Flicker() {
      wheel = new PWMTalonSRX(FlickerConstants.wheelPWMid);
      arm = new CANSparkMax(FlickerConstants.pivotCANid, MotorType.kBrushless);

      this.arm.setSmartCurrentLimit(30); // recalculate (placeholder numbers)
      this.arm.setInverted(false);
      this.arm.setIdleMode(IdleMode.kBrake);
      this.arm.setOpenLoopRampRate(.35);


      encoderArm = arm.getEncoder();
      wheel.setInverted(true);

      pidControllerArm = arm.getPIDController();

        kP = 0.55; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = .5; 
        kMinOutput = -.5;
    


        pidControllerArm.setP(0.55); // recalculate (placeholder numbers)
        pidControllerArm.setI(0.0);
        pidControllerArm.setD(0.0);
        ////pidControllerArm.setIZone(0);
        //pidControllerArm.setIMaxAccum(0.0, 0);
        pidControllerArm.setFF(0.0);
        pidControllerArm.setOutputRange(-.3, .3);
        SmartDashboard.putNumber("Flicker P Gain", kP);
        SmartDashboard.putNumber("Flicker I Gain", kI);
        SmartDashboard.putNumber("Flicker D Gain", kD);
        SmartDashboard.putNumber("Flicker Max Output", kMaxOutput);
        SmartDashboard.putNumber("Flicker Min Output", kMinOutput);
        SmartDashboard.putNumber("Flicker Set Rotations", 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flicker Encoder", encoderArm.getPosition());
        double p = SmartDashboard.getNumber("Flicker P Gain", 0);
        double i = SmartDashboard.getNumber("Flicker I Gain", 0);
        double d = SmartDashboard.getNumber("Flicker D Gain", 0);
        double max = SmartDashboard.getNumber("Flicker Max Output", 0);
        double min = SmartDashboard.getNumber("Flicker Min Output", 0);
        double encoderValue = SmartDashboard.getNumber("Flicker Set Rotations", 0);

        if((p != kP)) { pidControllerArm.setP(p); kP = p; }
        if((i != kI)) { pidControllerArm.setI(i); kI = i; }
        if((d != kD)) { pidControllerArm.setD(d); kD = d; }

        if((max != kMaxOutput) || (min != kMinOutput)) { 
            pidControllerArm.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }
        if((encoderValue != kPosition)){pidControllerArm.setReference(encoderValue, ControlType.kPosition); kPosition = encoderValue;}
    }
    

    public void moveArm(double speed){
      arm.set(speed);
    }
  
    public void spinWheel(double speed) {
      wheel.set(speed);
    }

    public void positionArm(double pos) {
      pidControllerArm.setReference(pos, ControlType.kPosition, 0); // recalculate (placeholder numbers)
  }

  public void resetEncoders(){
    encoderArm.setPosition(0);
  }
}
