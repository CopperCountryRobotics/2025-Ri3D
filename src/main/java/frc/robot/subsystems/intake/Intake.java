package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private PWMTalonSRX intake;
  
    public Intake(int intake) {
      this.intake = new PWMTalonSRX(intake);
      this.intake.setInverted(true);
    }
  
    public void intake(double intakeSpeed, double indexSpeed) {
      this.intake.set(intakeSpeed);
    }
  
    public void stop() {
      this.intake.set(0);
    }
  }