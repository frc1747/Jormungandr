// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PivotIntake extends SubsystemBase {
  private TalonFX hinge;

  /** Creates a new Intake. */
  public PivotIntake() {
    hinge = new TalonFX(Constants.IntakeConstants.HINGE);
    configPID();
    hinge.setNeutralMode(NeutralModeValue.Brake);
  }

  public void configPID() {
    hinge.getConfigurator().apply(new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
        .withKP(0.05)
        .withKI(0.0)
        .withKD(0.0)
        .withKA(getPosition()) // I don't know which feedforward type to use here
      )
      .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
        .withReverseLimitAutosetPositionValue(0.0)
      )
    );
  }

  public void setHingePower(double power) {
    if (getPosition() > Constants.IntakeConstants.DROPPED && power >= 0) {
      hinge.set(0.0);
    } else if (power < 0 && getPosition() <= Constants.IntakeConstants.DROPPED / 3) {
      hinge.set(power * Constants.IntakeConstants.IN_SLOW_FACTOR);
    } else {
      hinge.set(power);
    }
  }

  public void liftIntake() {
    hinge.setControl(new PositionDutyCycle(Constants.IntakeConstants.STOWED));
  }

  public void dropIntake() {
    hinge.setControl(new PositionDutyCycle(Constants.IntakeConstants.DROPPED));
  }

  public void setEncoderPos(double position) {
    hinge.setPosition(position);
  }

  public double getPosition() {
    // System.out.println(hinge.getSelectedSensorPosition());
    return hinge.getPosition().getValueAsDouble();
  }

  public boolean switchPressed() {
    return hinge.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
  }

  @Override
  public void periodic() {
    /*
    boolean reverseLimitClosed = hinge.isRevLimitSwitchClosed() == 1;
    if (reverseLimitClosed) {
      setEncoderPos(0);
    }
    */
    // This method will be called once per scheduler run
    //System.out.println(this.getPosition());
    SmartDashboard.putNumber("Intake Pivot Encoder", getPosition());
  }
}

