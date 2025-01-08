// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;
import frc.robot.subsystems.PivotShooter;
import frc.robot.subsystems.Shooter;

public class BringIn extends Command {
  private PivotShooter shooter;
  private PivotIntake intake;
  private boolean shooterDone = false;
  private boolean intakeDone = false;

  /** Creates a new BringIn. */
  public BringIn(PivotShooter shooter, PivotIntake intake) {
    this.shooter = shooter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shooterDone) {
      shooter.setHingePower(-0.05);
      if (shooter.switchPressed()) {
        shooterDone = true;
        shooter.setEncoderPos(0);
      }
    } else {
      shooter.setHingePower(0.0);
    }
    if (!intakeDone) {
      intake.setHingePower(-0.05);
      if (intake.switchPressed()) {
        intakeDone = true;
        intake.setEncoderPos(0);
      }
    } else {
      intake.setHingePower(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setHingePower(0.0);
    intake.setHingePower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterDone && intakeDone;
  }
}
