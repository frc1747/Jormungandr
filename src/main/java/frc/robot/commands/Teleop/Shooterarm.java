// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotShooter;

public class Shooterarm extends Command {
  private PivotShooter shooter;
  private DoubleSupplier pow;
  private double encoder ;

  /** Creates a new ShooterAlignAmp. */
  public Shooterarm(PivotShooter shooter, DoubleSupplier pow , double encoder) {
    this.shooter = shooter;
    this.pow = pow;
    this.encoder = encoder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //shooter.alignShooterAmp();
    // done = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(shooter.In_limit(encoder));
    // System.out.println(shooter.getPosition());
    
    if (shooter.In_limit(encoder) || pow.getAsDouble() <= 0) {
      shooter.setHingePower(MathUtil.applyDeadband(pow.getAsDouble(), Constants.ControllerConstants.STICK_DEADBAND) * Constants.ShooterConstants.HINGE_SPEED);
    } else {
      shooter.setHingePower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setHingePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
