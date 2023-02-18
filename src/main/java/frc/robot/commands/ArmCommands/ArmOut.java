package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmOut extends CommandBase {

    private Arm s_Arm;
    private double time;

    public ArmOut(Arm input) {
        s_Arm = input;
        time = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        s_Arm.intake();
        System.out.println("COMMAND WAS CALLED");
    }

    @Override
    public void initialize() {
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - time > 2000;
    }
}
