// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autoscommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.PivotShooter;
import frc.robot.subsystems.Shooter;

public class ManualControlShooter extends Command {
  private PivotShooter pivot;
  private Shooter shooter;
  private Feeder feeder;
  private Joystick controller;

  /** Creates a new ManualControl. */
  public ManualControlShooter(PivotShooter pivot, Shooter shooter, Feeder feeder, Joystick controller) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.controller = controller;
    this.pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(feeder);
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final int leftY = XboxController.Axis.kLeftY.value;
    final int rightY = XboxController.Axis.kRightY.value;
    final int rightX = XboxController.Axis.kRightX.value;
    shooter.setShooterPower(deadzone(controller.getRawAxis(rightY)));
    feeder.setShooterFeedPower(deadzone(controller.getRawAxis(rightX)));
    double left = deadzone(controller.getRawAxis(leftY));
    if (left == 0) {
      pivot.setHingePower(0.0);
    } else if (left < 0 && shooter.getPosition() > Constants.ShooterConstants.DOWN_LIMIT) {
      pivot.setHingePower(left);
    } else if (left > 0 && shooter.getPosition() < Constants.ShooterConstants.UP_LIMIT) {
      pivot.setHingePower(left);
    } else {
      pivot.setHingePower(0.0);
    }
  }

  private double deadzone(double power) {
    double returned = 0.0;
    if (power > 0.10) {
      returned = (power - 0.10) / 0.90;
    } else if (power < -0.10) {
      returned = (power + 0.10) / 0.90;
    }
    return returned;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setHingePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
