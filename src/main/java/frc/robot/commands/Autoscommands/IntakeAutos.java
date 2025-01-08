// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.Autoscommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PivotIntake;
import edu.wpi.first.wpilibj.Timer;

public class IntakeAutos extends Command {
  private PivotIntake intakePivot;
  private Intake intake;
  private Timer time = new Timer();

  public IntakeAutos(Intake intake, PivotIntake intakePivot) {
    this.intake = intake;
    this.intakePivot = intakePivot;
    addRequirements(intakePivot, intake);
  }

  @Override
  public void initialize() {
    intake.setRollerPower(Constants.IntakeConstants.ROLLER_SPEED);
    intakePivot.setHingePower(Constants.IntakeConstants.PIVOT_OUT_SPEED);
    time.reset();
    time.start();
 
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
    System.out.print(time.get());
    return intake.switchPressed() || time.hasElapsed(2);
}

}


























// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Autoscommands;

// import java.util.concurrent.TimeUnit;
// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.PivotIntake;

// public class IntakeAutos extends Command {
//   /** Creates a new Intake. */
//   Intake pIntake;
//   PivotIntake pivot; 
//   String type;
//   public IntakeAutos(Intake pIntake, PivotIntake pivot, String type) {

//     this.pivot= pivot;
//     this.pIntake = pIntake;
//     this.type = type;
//     addRequirements(pivot, pIntake);



//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     boolean done = false;

//     if (type.equals("stow" )) {
//       pIntake.setRollerPower(0);
//       pivot.liftIntake();
//     } else if(type.equals("floor")) {
//       pIntake.setRollerPower(Constants.IntakeConstants.IN_SPEED);
//       pivot.dropIntake();
//       Timer.delay(.1);
      
//       // pivot.liftIntake(); 
//       // pIntake.setRollerPower(0);
//       done = true;
//     }

//     pIntake.setRollerPower(0);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     pIntake.setRollerPower(0);
//     pivot.liftIntake(); 
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return pivot.switchPressed() ;
//   }
// }
