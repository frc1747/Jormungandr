package frc.robot;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import static edu.wpi.first.units.Units.Degrees;

public final class CTREConfigs {
    public CANcoderConfiguration swerveCanCoderConfig;
    public MagnetSensorConfigs magnetSensorConfig;
    public CTREConfigs() {
        swerveCanCoderConfig = new CANcoderConfiguration();
        magnetSensorConfig = new MagnetSensorConfigs();
        magnetSensorConfig.withAbsoluteSensorDiscontinuityPoint(Degrees.of(360)); // https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html
        magnetSensorConfig.withSensorDirection(Constants.DrivetrainConstants.canCoderDirection);
        magnetSensorConfig.withMagnetOffset(Degrees.of(0));
        swerveCanCoderConfig.withMagnetSensor(magnetSensorConfig);
    }
}