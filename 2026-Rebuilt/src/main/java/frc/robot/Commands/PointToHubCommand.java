// PointToHub.java
package frc.robot.Commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Libs.LimelightHelpers;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class PointToHubCommand extends Command {

    // Fallback odometry position if no tags seen
    private static final Translation2d BLUE_HUB = new Translation2d(2.104, 4.034);
    private static final Translation2d RED_HUB  = new Translation2d(14.437, 4.034);

    // Hub AprilTag IDs per 2026 manual
    private static final Set<Integer> BLUE_HUB_TAGS = Set.of(2, 3, 4, 5);
    private static final Set<Integer> RED_HUB_TAGS  = Set.of(8, 9, 10, 11);

    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final String limelightName;

    private final ProfiledPIDController rotationController = new ProfiledPIDController(
        5.0,
        0.0,
        0.2,
        new TrapezoidProfile.Constraints(
            Units.degreesToRadians(540),
            Units.degreesToRadians(720)
        )
    );

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public PointToHubCommand(CommandSwerveDrivetrain drivetrain,
                             DoubleSupplier xSupplier,
                             DoubleSupplier ySupplier,
                             String limelightName) {
        this.drivetrain    = drivetrain;
        this.xSupplier     = xSupplier;
        this.ySupplier     = ySupplier;
        this.limelightName = limelightName;

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(1.5));

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
        double targetAngle = getTargetAngle(robotPose);

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

    /**
     * Returns the target angle to face.
     * Priority: Limelight tx (direct tag offset) → field pose from MT2 → odometry fallback
     */
    private double getTargetAngle(Pose2d robotPose) {
        // --- 1. Try direct tx from Limelight (fastest, lowest latency) ---
        LimelightHelpers.RawFiducial[] fiducials =
            LimelightHelpers.getRawFiducials(limelightName);

        if (fiducials != null && fiducials.length > 0) {
            // Find the best hub tag (closest to center of frame)
            double bestTx = Double.MAX_VALUE;
            boolean foundHubTag = false;

            Set<Integer> allianceHubTags = getAllianceHubTags();

            for (LimelightHelpers.RawFiducial f : fiducials) {
                if (allianceHubTags.contains(f.id) && Math.abs(f.txnc) < Math.abs(bestTx)) {
                    bestTx = f.txnc; // txnc = tx not corrected, raw horizontal offset
                    foundHubTag = true;
                }
            }

            if (foundHubTag) {
                // Rotate by tx offset — negative because tx right = need to turn right (negative CCW)
                return robotPose.getRotation().getRadians() 
                     + Units.degreesToRadians(-bestTx);
            }
        }

        // --- 2. Try MegaTag2 field pose from Limelight ---
        LimelightHelpers.PoseEstimate mt2 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (mt2 != null && mt2.tagCount > 0) {
            Translation2d toHub = getHubTranslation().minus(mt2.pose.getTranslation());
            return Math.atan2(toHub.getY(), toHub.getX());
        }

        // --- 3. Odometry fallback ---
        Translation2d toHub = getHubTranslation().minus(robotPose.getTranslation());
        return Math.atan2(toHub.getY(), toHub.getX());
    }

    private Translation2d getHubTranslation() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_HUB;
        }
        return BLUE_HUB;
    }

    private Set<Integer> getAllianceHubTags() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_HUB_TAGS;
        }
        return BLUE_HUB_TAGS;
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