package frc.lightning.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lightning.util.RamseteGains;

import java.util.Objects;

/**
 * Base differential drivetrain type that is capable of following motion profiles.
 */
public interface LightningDrivetrain extends Subsystem {
    
    /**
     * Sample command that can be applied to LightningDrivetrain type
     */
    public class DriveCommand {
        public double leftCommand;
        public double rightCommand;

        public DriveCommand(double left, double right) {
            leftCommand = left;
            rightCommand = right;
        }

        @Override
        public String toString() {
            return "DriveCommand{" +
                   "leftCommand=" + leftCommand +
                   ", rightCommand=" + rightCommand +
                   '}';
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            DriveCommand that = (DriveCommand) o;
            return Double.compare(that.leftCommand, leftCommand) == 0 &&
                   Double.compare(that.rightCommand, rightCommand) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(leftCommand, rightCommand);
        }
    }

    /**
     * Initialize the Drivetrain
     */
    public default void init() {
    }

    /**
     * Sets power to the drivetrain
     * @param left Power for left side
     * @param right Power for right side
     */
    public void setPower(double left, double right);

    /**
     * Sets power to the drivetrain
     * @param cmd The {@link LightningDrivetrain.DriveCommand} to use.
     */
    public default void setPower(DriveCommand cmd) {
        setPower(cmd.leftCommand, cmd.rightCommand);
    }

    /**
     * Sets closed-loop velocity to drivetrain
     * @param left Left side velocity
     * @param right Right side velocity
     */
    public void setVelocity(double left, double right);

    /**
     * Sets closed-loop velocity to drivetrain
     * @param cmd The {@link LightningDrivetrain.DriveCommand} to use.
     */
    public default void setVelocity(DriveCommand cmd) {
        setVelocity(cmd.leftCommand, cmd.rightCommand);
    }

    /**
     * Zeros encoder count on all sides
     */
    public void resetDistance();

    /**
     * Distance traveled by left side
     * @return The distance traveled by the left side of the drivetrain
     */
    public double getLeftDistance();

    /**
     * Distance traveled by right side
     * @return The distance traveled by the right side of the drivetrain
     */
    public double getRightDistance();

    /**
     * Instantaneous velocity of left side
     * @return The instantaneous velocity of the left side of the drivetrain
     */
    public double getLeftVelocity();

    /**
     * Instantaneous velocity of right side
     * @return The instantaneous velocity of the right side of the drivetrain
     */
    public double getRightVelocity();

    /**
     * Stops the drivetrain by zeroing the power applied to each side
     */
    public default void stop() {
        setPower(0d, 0d);
    }

    /**
     * Sets drivetrain output as an applied voltage
     * @param leftVolts Target voltage for left side of drivetrain
     * @param rightVolts Target voltage for right side of drivetrain
     */
    public void setOutput(double leftVolts, double rightVolts);

    /**
     * Configures all motors in brake mode for faster deceleration
     */
    public void brake();

    /**
     * Configures all motors in coast mode for slower deceleration
     */
    public void coast();

    /**
     * Configures motor directions
     */
    public void initMotorDirections();

    /**
     * Resets all sensor values to 0
     */
    public void resetSensorVals();

    /**
     * The gains of the drivetrain as used with a 
     * {@link edu.wpi.first.wpilibj2.command.RamseteCommand RamseteCommand}
     * @return Drivetrain trajectory following gains
     */
    public RamseteGains getConstants();

    /**
     * Drivetrain motor feedforward as a 
     * {@link edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward SimpleMotorFeedforward}
     * object.
     * @return The drivetrain feedforward
     */
    public default SimpleMotorFeedforward getFeedforward() {
        return null;
    }

    /**
     * Drivetrain kinematics as a 
     * {@link edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics DifferentialDriveKinematics}
     * object.
     * @return The drivetrain kinematics for use with a {@link edu.wpi.first.wpilibj2.command.RamseteCommand RamseteCommand}
     */
    public DifferentialDriveKinematics getKinematics();

    /**
     * Returns the position of the robot since start-up.
     * @return The robot's position as a {@link edu.wpi.first.wpilibj.geometry.Pose2d Pose2d} object.
     */
    public Pose2d getPose();

    /**
     * Returns the position of the robot since it was zeroed with {@link #setRelativePose()}.
     * @return The robot's position as a {@link edu.wpi.first.wpilibj.geometry.Pose2d Pose2d} object.
     */
    public Pose2d getRelativePose();

    /**
     * Zeros the pose returned bu {@link #getRelativePose()} to be relative to zero.
     * Useful for using robot-relative trajectories while still keping track of field pose.
     */
    public void setRelativePose();

    /**
     * The current real output to the drivetrain. 
     * @return The real speeds of the drivetrain as a 
     * {@link edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds DifferentialDriveWheelSpeeds}
     * object.
     */
    public DifferentialDriveWheelSpeeds getSpeeds();

    /**
     * PID controller for left side of the drivetrain
     * @return A {@link edu.wpi.first.wpilibj.controller.PIDController PIDController} for the
     * left side of the drivetrain
     */
    public PIDController getLeftPidController();
    
    /**
     * PID controller for right side of the drivetrain
     * @return A {@link edu.wpi.first.wpilibj.controller.PIDController PIDController} for the
     * right side of the drivetrain
     */
    public PIDController getRightPidController();
    
    /**
     * Current output to right motor(s)
     * @return The current real output to the right motor(s) in Volts
     */
    public double getRightVolts();
    
    /**
     * Current output to left motor(s)
     * @return The current real output to the left motor(s) in Volts
     */
    public double getLeftVolts();

    /**
     * Right side motor temperature
     * @return The temperature of the right side of the drivetrain
     */
    public double getRightTemp();
    
    /**
     * Left side motor temperature
     * @return The temperature of the left side of the drivetrain
     */
    public double getLeftTemp();

    /**
     * Sets the output of the drivetrain. Intended for specific use with 
     * {@link edu.wpi.first.wpilibj2.command.RamseteCommand RamseteCommand}
     * @param leftVolts Target left output voltage
     * @param rightVolts Target right output voltage
     */
    public void setRamseteOutput(double leftVolts, double rightVolts);

    /**
     * Net distance of robot as an average of each sides distance
     * @return Net distance of drivetrain
     */
    public default double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    /**
     * Net velocity of robot as an average of each sides velocity
     * @return Net velocity of drivetrain
     */
    public default double getVelocity() {
        return (getRightVelocity() + getLeftVelocity()) / 2;
    }
    
}
