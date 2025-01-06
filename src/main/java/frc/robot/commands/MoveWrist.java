package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

public class MoveWrist extends Command {
    
    private final Wrist wrist;
    private final boolean direction;

    public MoveWrist(Wrist wrist, boolean moveOut) {
        this.wrist = wrist;
        direction = moveOut;
    }

    @Override
    public void execute() {
        wrist.rotate(WristConstants.wristSpeed * (direction ? 1 : -1));
    }

    @Override
    public boolean isFinished() {
        return wrist.hasHitHardStop();
    }

}
