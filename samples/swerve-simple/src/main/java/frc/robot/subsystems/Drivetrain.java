package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.lightningrobotics.common.controller.PIDFController;
import com.lightningrobotics.common.subsystem.drivetrain.swerve.SwerveDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.swerve.SwerveGains;
import com.lightningrobotics.common.subsystem.drivetrain.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;

public class Drivetrain extends SwerveDrivetrain {

    private static final SwerveGains SWERVE_GAINS = new SwerveGains(
        22.5, // width
        22.5, // length
        16.2, // maxSpeed
        16.2, // maxRealSpeed
        10, // maxAcceleration
        16.2, // maxAngularSpeed
        new boolean[]{true, true, true, true}, // turnMotorInverts
        new boolean[]{false, false, false, false} // driveMotorInverts
    );

    public Drivetrain() {
        super(SWERVE_GAINS, new SwerveModule[]{
            makeModule(Modules.FRONT_LEFT, 8, 7, 16, Rotation2d.fromDegrees(-95.09765625)),
            makeModule(Modules.FRONT_RIGHT, 11, 12, 17, Rotation2d.fromDegrees(-12.744140625)),
            makeModule(Modules.REAR_LEFT, 10, 9, 15, Rotation2d.fromDegrees(30.673828125)),
            makeModule(Modules.REAR_RIGHT, 13, 14, 18, Rotation2d.fromDegrees(119.00390625))
        });
    }

    private static SwerveModule makeModule(Modules module, int driveID, int angleID, int encoderID, Rotation2d offset) {
        
        // Set Up Drive Motor
        WPI_TalonFX driveMotor = new WPI_TalonFX(driveID);
        driveMotor.configFactoryDefault();
        driveMotor.setNeutralMode(NeutralMode.Brake);

        // Set Up Azimuth Motor
        WPI_TalonFX azimuthMotor = new WPI_TalonFX(angleID);
        azimuthMotor.configFactoryDefault();
        azimuthMotor.setNeutralMode(NeutralMode.Brake);
        azimuthMotor.setInverted(true);

        // Set Up Encoder
        CANCoder canCoder = new CANCoder(encoderID);
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
        canCoderConfiguration.sensorDirection = true;
        canCoder.configAllSettings(canCoderConfiguration);

        // Build Module
        return new SwerveModule(
            SWERVE_GAINS,
            driveMotor, 
            azimuthMotor, 
            () -> Rotation2d.fromDegrees(canCoder.getAbsolutePosition()),
            () -> (driveMotor.getSelectedSensorVelocity() * 10d) * (4 * Math.PI / (2048d * 6.86d)),
            null,
            new PIDFController(0.22, 0.22, 0.0005, 0.054));

    }
    
}
