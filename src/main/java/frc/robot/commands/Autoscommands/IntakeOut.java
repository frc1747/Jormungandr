// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autoscommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;

public class IntakeOut extends Command {
  private Intake intake;
  private PivotIntake pIntake;

  public IntakeOut(Intake intake, PivotIntake pIntake) {
    this.intake = intake;
    this.pIntake = pIntake;
    addRequirements(intake, pIntake);
  }

  @Override
  public void initialize() {
    intake.setRollerPower(Constants.IntakeConstants.OUT_SPEED);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intake.setRollerPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
