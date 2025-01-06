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

        kP = 0.2; 
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
        return Math.abs(setpoint - encoder.getPosition()) < 1;
    }

    public double getPosition(){
        return encoder.getPosition();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Wrist Pos", encoder.getPosition());
    }

    public void resetEncoders(){
        encoder.setPosition(0);
    }
}
