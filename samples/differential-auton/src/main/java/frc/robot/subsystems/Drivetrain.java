package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.lightningrobotics.common.controller.PIDFController;
import com.lightningrobotics.common.subsystem.core.LightningIMU;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialGains;

import com.lightningrobotics.common.subsystem.drivetrain.PIDFDashboardTuner;
import com.lightningrobotics.common.controller.FeedForwardController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// beginning 14803, 22081
// end:  44673, 50489
// change: 29870, 28408
public class Drivetrain extends DifferentialDrivetrain {

    private static final PIDFController leftPIDs = new PIDFController(0.0038793, 0, 0);
    private static final PIDFController rightPIDs = new PIDFController(0.0038793, 0, 0);

    private static final MotorController[] leftMotors = new MotorController[]{
        new WPI_TalonFX(1),
        new WPI_TalonFX(2),
        new WPI_TalonFX(3)
    };

    private static final MotorController[] rightMotors = new MotorController[]{
        new WPI_TalonFX(4),
        new WPI_TalonFX(5),
        new WPI_TalonFX(6)
    };

    private static final DifferentialGains DIFFERENTIAL_GAINS = new DifferentialGains(
        1d,
        1d,
        0.5583711759,
        new boolean[]{true, false, false},
        new boolean[]{false, true, true} 
    );

    
    public Drivetrain() {
        super(
            DIFFERENTIAL_GAINS, 
            leftMotors, 
            rightMotors, 
            LightningIMU.navX(), 
            () -> ((((WPI_TalonFX)leftMotors[0]).getSelectedSensorVelocity() * 10d) * (6.16 * Math.PI / (2048d * 15d)) * 0.0254),
            () -> ((((WPI_TalonFX)rightMotors[0]).getSelectedSensorVelocity() * 10d) * (6.16 * Math.PI / (2048d * 15d)) * 0.0254),
            leftPIDs, 
            rightPIDs, 
            new FeedForwardController(0.53397, 3.2953, 0.17849),
            () -> (((WPI_TalonFX)leftMotors[0]).getSelectedSensorPosition() / (2048 * 15) * (6.16 * Math.PI) * 0.0254),
            () -> (((WPI_TalonFX)rightMotors[0]).getSelectedSensorPosition() / (2048 * 15) * (6.16 * Math.PI) * 0.0254)
        );

        PIDFDashboardTuner leftPID = new PIDFDashboardTuner("Autonomous", leftPIDs, "left");
        PIDFDashboardTuner rightPID = new PIDFDashboardTuner("Autonomous", rightPIDs, "right");


    }

}