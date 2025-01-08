// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;

public class FloorPickup extends Command {
  private PivotIntake intakePivot;
  private Intake intake;

  public FloorPickup(Intake intake, PivotIntake intakePivot) {
    this.intake = intake;
    this.intakePivot = intakePivot;
    addRequirements(intakePivot, intake);
  }

  @Override
  public void initialize() {
    intake.setRollerPower(Constants.IntakeConstants.ROLLER_SPEED);
    intakePivot.setHingePower(Constants.IntakeConstants.PIVOT_OUT_SPEED);
  }

  @Override
  public void execute() {
    if (intakePivot.getPosition() > Constants.IntakeConstants.DROPPED) {
      intakePivot.setHingePower(0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.setRollerPower(0.0);
  }

  @Override
  public boolean isFinished() {
    return intake.switchPressed();
  }
}
