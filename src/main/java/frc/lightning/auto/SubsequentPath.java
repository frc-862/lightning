package frc.lightning.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.lightning.subsystems.LightningDrivetrain;

/**
 * A {@link Path} that will not be relative to zero or force the 
 * {@link frc.lightning.subsystems.LightningDrivetrain Drivetrain} to
 * zero its {@link edu.wpi.first.wpilibj.geometry.Pose2d Pose} before starting.
 */
public class SubsequentPath extends Path {

    /**
     * Creates a new SubsequentPath
     * @param name The name of the new path
     * @param waypoints The waypoints the path should pass through
     */
    public SubsequentPath(String name, List<Pose2d> waypoints) {
        this(name, waypoints, false);
    }

    /**
     * Creates a new SubsequentPath
     * @param name The name of the new path
     * @param waypoints The waypoints the path should pass through
     * @param reversed True if the path should be driven backwards, false otherwise
     */
    public SubsequentPath(String name, List<Pose2d> waypoints, boolean reversed) {
        super(name, waypoints, reversed);
    }

    /**
     * Creates a new SubsequentPath
     * @param name The name of the new path
     * @param jsonPath The waypoints the path should pass through as a json from PathWeaver
     * @param reversed True if the path should be driven backwards, false otherwise
     */
    public SubsequentPath(String name, String jsonPath, boolean reversed) {
        super(name, jsonPath, reversed);
    }

    /**
     * Creates a new SubsequentPath
     * @param name The name of the new path
     * @param jsonPath The waypoints the path should pass through as a json from PathWeaver
     */
    public SubsequentPath(String name, String jsonPath) {
        this(name, jsonPath, false);
    }

    @Override
    public Command getCommand(LightningDrivetrain drivetrain) {

        Trajectory trajectory = this.getTrajectory(drivetrain);

        RamseteCommand cmd = new RamseteCommand(
            trajectory,
            drivetrain::getRelativePose,
            new RamseteController(),
            drivetrain.getFeedforward(),
            drivetrain.getKinematics(),
            drivetrain::getSpeeds,
            drivetrain.getLeftPidController(),
            drivetrain.getRightPidController(),
            drivetrain::setRamseteOutput,
            drivetrain
        ) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                drivetrain.stop();
            }
        };

        return cmd;

    }
    
}

