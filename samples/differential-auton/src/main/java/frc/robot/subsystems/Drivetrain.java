package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialGains;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Drivetrain extends DifferentialDrivetrain {

    // Foor quasar
    private static final DifferentialGains DIFFERENTIAL_GAINS = new DifferentialGains(
        5d,
        5d,
        0.5583711759,
        new boolean[]{true, false, false},
        new boolean[]{false, true, true},
        6.16d,
        15
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
            }
        );
    }
    
}
