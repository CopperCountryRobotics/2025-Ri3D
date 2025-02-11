package frc.robot.subsystems.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    
    private final CANSparkMax wrist;

    private SparkPIDController wristPID;
    private RelativeEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double setpoint;

    public Wrist() {
        wrist = new CANSparkMax(WristConstants.wristCANId, MotorType.kBrushless);
        encoder = wrist.getEncoder();
        wristPID = wrist.getPIDController();

        kP = 0.4; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        wristPID.setP(kP);
        wristPID.setI(kI);
        wristPID.setD(kD);
        wristPID.setIZone(kIz);
        wristPID.setFF(kFF);
        wristPID.setOutputRange(kMinOutput, kMaxOutput);
        
        wrist.setSmartCurrentLimit(20);
        wrist.setIdleMode(IdleMode.kBrake);
        wrist.setOpenLoopRampRate(.35);

        resetEncoders();
        wristPID.setReference(0, ControlType.kPosition);

        SmartDashboard.putNumber("Wrist P Gain", kP);
        SmartDashboard.putNumber("Wrist I Gain", kI);
        SmartDashboard.putNumber("Wrist D Gain", kD);
        SmartDashboard.putNumber("Wrist Max Output", kMaxOutput);
        SmartDashboard.putNumber("Wrist Min Output", kMinOutput);
        SmartDashboard.putNumber("Wrist Set- Rotations", 0);
    }


    /**
     * Rotate the wrist. Positive is away from the robot. 
     * @param speed
     */
    public void rotate(double speed) {
        wrist.set(speed);
    }

    public void setPosition(double setpoint){
        wristPID.setReference(setpoint, ControlType.kPosition);
        this.setpoint = setpoint;
    }

    public boolean atPosition(){
        return Math.abs(setpoint - encoder.getPosition()) < .1;
    }

    public double getPosition(){
        return encoder.getPosition();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Encoder", encoder.getPosition());
        double p = SmartDashboard.getNumber("Wrist P Gain", 0);
        double i = SmartDashboard.getNumber("Wrist I Gain", 0);
        double d = SmartDashboard.getNumber("Wrist D Gain", 0);
        double max = SmartDashboard.getNumber("Wrist Max Output", 0);
        double min = SmartDashboard.getNumber("Wrist Min Output", 0);
        double encoderValue = SmartDashboard.getNumber("Wrist Set- Rotations", 0);

        if((p != kP)) { wristPID.setP(p); kP = p; }
        if((i != kI)) { wristPID.setI(i); kI = i; }
        if((d != kD)) { wristPID.setD(d); kD = d; }

        if((max != kMaxOutput) || (min != kMinOutput)) { 
            wristPID.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }
        //if((encoderValue != setpoint)){wristPID.setReference(encoderValue, ControlType.kPosition); setpoint = encoderValue;}
    }

    public void resetEncoders(){
        encoder.setPosition(0);
    }
}
