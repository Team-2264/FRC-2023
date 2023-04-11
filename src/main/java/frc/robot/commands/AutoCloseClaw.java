package frc.robot.commands;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ObjectVision;
import frc.robot.enums.ArmStatus;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class AutoCloseClaw extends CommandBase {

    private DigitalInput dioOne, dioTwo;
    private Arm arm;
    private Swerve swerve;
    private boolean isFinished;

    private double saveTime;

    public AutoCloseClaw(Arm armInput, Swerve swerveInput) {
        this.dioOne = armInput.dioOne;
        this.dioTwo = armInput.dioTwo;
        this.swerve = swerveInput;
        this.arm = armInput;
    }

    @Override
    public void initialize() {
        saveTime = System.currentTimeMillis();
        arm.openClaw();
        isFinished = false;
    }

    @Override
    public void execute() {
        if(dioOne.get() || dioTwo.get()) {
            if(System.currentTimeMillis() - saveTime > 1500)    isFinished = true;

            if(System.currentTimeMillis() - saveTime > 500) {
                swerve.driveStraightBack();
                if(arm.status == ArmStatus.INTAKE_LOW) arm.bringArmHome();
            }

            if(System.currentTimeMillis() - saveTime > 2 && System.currentTimeMillis() - saveTime < 500) {
                arm.closeClaw();
                swerve.stop();
            } 
        } else {
            saveTime = System.currentTimeMillis();
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
