// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotIntake;

public class intakeMove extends Command {
  private PivotIntake intake;
  private DoubleSupplier pow;
  private double power;

  /** Creates a new IntakeUp. */
  public intakeMove(PivotIntake intake, DoubleSupplier pow) {
    this.intake = intake;
    this.pow = pow;
    power = pow.getAsDouble();

    if (power < 0) {
      power *= 0.5;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    //intake.liftIntake();
   // done = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // System.out.println(intake.getPosition());
    if (!intake.switchPressed() || (power > 0)){
      if (intake.getPosition() > Constants.IntakeConstants.DROPPED || power < 0)
        intake.setHingePower(power * 0.65);
      else
        intake.setHingePower(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setHingePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

