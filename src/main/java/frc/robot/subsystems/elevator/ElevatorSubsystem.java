package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;

import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.SparkUtil;

public class ElevatorSubsystem extends SubsystemBase{
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    public ElevatorSubsystem(){
        motor = new SparkMax(DriveConstants.leftOneCANID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        SparkMaxConfig globalConfig = new SparkMaxConfig();
        globalConfig.smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(false);
        SparkUtil.tryUntilOk(5, 
        () -> motor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    /**
     * Set the motion of the elevator. Positive is up.
     * @param speed
     */
    public void run(double speed){
        motor.set(speed);
    }

    public boolean atMaxHeight(){
        if(motor.get() > 0.01 
        && Math.abs(encoder.getVelocity()) < 0.1 
        && motor.getOutputCurrent() > 15){
            return true;
        }
        return false;
    }

    public boolean atMinHeight(){
        if(motor.get() < 0.01 
        && Math.abs(encoder.getVelocity()) < 0.1 
        && motor.getOutputCurrent() > 15){
            return true;
        }
        return false;
    }

}
