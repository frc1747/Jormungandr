package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  private TalonFX shooterFeed;
  private DigitalInput limitSwitch;
  
  /** Creates a new Shooter. */
  public Feeder() {
    shooterFeed = new TalonFX(Constants.FeederConstants.BACK);
    shooterFeed.setNeutralMode(NeutralModeValue.Brake);
    limitSwitch = new DigitalInput(Constants.FeederConstants.LIMIT_SWITCH);
  }

  public void setShooterFeedPower(double power) {
    if (power < 0) {
      power *= 0.375;
    }
    shooterFeed.set(power);
  }
  
  public boolean switchPressed() {
    return limitSwitch.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
