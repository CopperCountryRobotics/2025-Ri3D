package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final CANSparkMax heightMotor;
    private final CANSparkMax heightMotor2;

    private SparkPIDController heightPID;
    private RelativeEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double setpoint;


    public Elevator(){
        heightMotor = new CANSparkMax(ElevatorConstants.heightMotorCANID, MotorType.kBrushless);
        heightMotor2 = new CANSparkMax(ElevatorConstants.heightMotor2CANID, MotorType.kBrushless);
        heightPID = heightMotor.getPIDController();
        encoder = heightMotor.getEncoder();

        kP = 0.1; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = .3; 
        kMinOutput = -.3;

        heightPID.setP(kP);
        heightPID.setI(kI);
        heightPID.setD(kD);
        heightPID.setIZone(kIz);
        heightPID.setFF(kFF);
        heightPID.setOutputRange(kMinOutput, kMaxOutput);
        heightMotor.setSmartCurrentLimit(50);
        heightMotor2.setSmartCurrentLimit(50);
        heightMotor.setIdleMode(IdleMode.kBrake);
        heightMotor2.setIdleMode(IdleMode.kBrake);
        heightMotor.setInverted(true);
        heightMotor2.follow(heightMotor, true);
        encoder.setPositionConversionFactor(ElevatorConstants.positionConversionFactor);

        heightMotor.getForwardLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);
        heightMotor.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(true);

        heightMotor.burnFlash();
        heightMotor2.burnFlash();
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
        SmartDashboard.putBoolean("Elevator at limit", hasHitHardStop());

        SmartDashboard.putBoolean("Elevator top limit", heightMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed());
        SmartDashboard.putBoolean("Elevator bottom limit", heightMotor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed());
    }

    public boolean hasHitHardStop() {
        return 
            heightMotor.getOutputCurrent() > ElevatorConstants.autoStopCurrentThreshold
            && 
            heightMotor.getEncoder().getVelocity() < ElevatorConstants.autoStopVelocityThreshold;
    }
    /**
     * Sets the elevators setpoint.
     * @param height Position to move towards
     */
    public void setPosition(double height){
        heightPID.setReference(height, ControlType.kPosition);
        setpoint = height;
    }

    public boolean atPosition(){
        return Math.abs(setpoint - encoder.getPosition()) < 1;
    }

    public void setSpeed(double speed){
        heightMotor.set(speed);
    }

    public void resetEncoders(){
        encoder.setPosition(0);
    }
}
