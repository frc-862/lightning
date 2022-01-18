package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.lightningrobotics.common.controller.PIDFController;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialGains;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Drivetrain extends DifferentialDrivetrain {

    // Foor quasar
    private static final DifferentialGains DIFFERENTIAL_GAINS = new DifferentialGains(
        5d,
        5d,
        0.5583711759,
        new boolean[]{true, false, false},
        new boolean[]{false, true, true}
    );

    public Drivetrain() {
        super(
            DIFFERENTIAL_GAINS, 
            new MotorController[]{
                new WPI_TalonFX(1),
                new WPI_TalonFX(2),
                new WPI_TalonFX(3)
            }, 
            new MotorController[]{
                new WPI_TalonFX(4),
                new WPI_TalonFX(5),
                new WPI_TalonFX(6),
            },
            () -> (new WPI_TalonFX(1).getSelectedSensorVelocity() * 10d) * (6.16 * Math.PI / (2048d * 15d)),
            () -> (new WPI_TalonFX(4).getSelectedSensorVelocity() * 10d) * (6.16 * Math.PI / (2048d * 15d)),
            // PID Values Placeholders
            new PIDFController(0.1, 0.1, 0.1, 0),
            new PIDFController(0.1, 0.1, 0.1, 0),
            new SimpleMotorFeedforward(0.1, 0.1, 0.1)
        );
    }
    
}
