package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class OpenClaw extends CommandBase {
    private Arm s_Arm;

    public OpenClaw(Arm input) {
        s_Arm = input;
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        s_Arm.openClaw();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
