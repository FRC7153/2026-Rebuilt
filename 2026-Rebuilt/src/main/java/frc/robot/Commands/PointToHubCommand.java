// PointToHub.java
package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;

public class PointToHubCommand extends Command {

    private static final Translation2d HUB_POSITION = new Translation2d(8.23, 4.12); 

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private final ProfiledPIDController rotationController = new ProfiledPIDController(
        5.0,  // kP 
        0.0,  // kI
        0.2,  // kD 
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(540), // max angular velocity (rad/s)
            Units.degreesToRadians(720)  // max angular acceleration (rad/s^2)
        )
    );

    // CTRE field-centric request 
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public PointToHubCommand(CommandSwerveDrivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.drivetrain = drivetrain;
        this.xSupplier  = xSupplier;
        this.ySupplier  = ySupplier;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(1.5));//Amount of error

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationController.reset(
            drivetrain.getState().Pose.getRotation().getRadians()
        );
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;

        Translation2d toHub = HUB_POSITION.minus(robotPose.getTranslation());
        double targetAngle = Math.atan2(toHub.getY(), toHub.getX());

        double rotationOutput = rotationController.calculate(
            robotPose.getRotation().getRadians(),
            targetAngle
        );

        drivetrain.setControl(
            driveRequest
                .withVelocityX(xSupplier.getAsDouble())
                .withVelocityY(ySupplier.getAsDouble())
                .withRotationalRate(rotationOutput)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}