package frc.robot.subsystems;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import java.util.function.BooleanSupplier;

import org.ejml.ops.FConvertArrays;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotShooter extends SubsystemBase {
  private TalonFX hinge;
  double start ;

  /** Creates a new Shooter. */
  public PivotShooter() {
    hinge = new TalonFX(Constants.ShooterConstants.HINGE);
    hinge.setNeutralMode(NeutralModeValue.Brake);

    hinge.getConfigurator().apply(new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
        .withKP(2.0)
        .withKI(0.0)
        .withKD(0.0)
        .withKA(getPosition()) // I don't know which feedforward type to use here
      )
      .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
        .withReverseLimitAutosetPositionValue(0.0)
      )
    );
    this.start =  hinge.getPosition().getValueAsDouble();
  }

  public void setHingePower(double power) {
    if (getPosition() > Constants.ShooterConstants.UP_LIMIT && power >= 0)
      hinge.set(0.0);
    else
      hinge.set(power);
  }

  public void dropShooter() {
    hinge.setPosition(Constants.ShooterConstants.STOWED);
  }

  public void alignShooterSpeaker() {
    hinge.setPosition(10.0);
  }

  public void goTo(double encoderPosition) {
    hinge.setPosition(encoderPosition);
  }

  public void alignShooterAmp() {
    // System.out.println("aligning");
    while (true) {
      if (getPosition() < Constants.ShooterConstants.AMP - 500) {
        hinge.set(0.10);
      } else if (getPosition() > Constants.ShooterConstants.AMP + 500) {
        hinge.set(-0.10);
      } else {
        hinge.set(0.0);
        break;
      }
    }
  }

  public double getPosition() {
    // System.out.println(hinge.getSelectedSensorPosition());
    return hinge.getPosition().getValueAsDouble();
  }

  public boolean switchPressed() {
    return hinge.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  public boolean In_limit(double zero){
    // System.out.println((hinge.getSelectedSensorPosition() + "+" + (Constants.ShooterConstants.UP_LIMIT + start)));
    return (this.getPosition() < Constants.ShooterConstants.UP_LIMIT);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Pivot Encoder", getPosition());
    //System.out.println(this.getPosition());
    /*
    boolean reverseLimitClosed = hinge.isRevLimitSwitchClosed() == 1;
    if (reverseLimitClosed) {
      setEncoderPos(0);
    }
    */
    // This method will be called once per scheduler run 

  }

}
