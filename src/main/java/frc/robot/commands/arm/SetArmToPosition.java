package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.enums.ArmStatus;
import frc.robot.subsystems.Arm;

public class SetArmToPosition extends CommandBase {

    private Arm s_Arm;
    private ArmStatus position;

    public SetArmToPosition(Arm input, ArmStatus position) {
        s_Arm = input;
        this.position = position;
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        switch (position) {
            case INTAKE:
                s_Arm.intake();
                break;
            case INTAKE_LOW:
                s_Arm.setLowIntake();
                break;
            case CUBE_MID:
                s_Arm.setMidCube();
                break;
            case CONE_MID:
                s_Arm.setMidCone();
                break;
            case CUBE_SIMBA:
                s_Arm.simbaCube();
                break;
            case CONE_SIMBA:
                s_Arm.simbaCone();
                break;
            case LOW:
                s_Arm.setLow();
                break;
            case HOME:
                s_Arm.bringArmHome();
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Arm.leftBelt.getClosedLoopError()) <= Constants.AutoConstants.ARM_ERROR_TOLERANCE;
    }
}
