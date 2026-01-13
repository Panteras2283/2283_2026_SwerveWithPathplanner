package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private static final String kLimelightName = "limelight";

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void periodic() {
        /* * 1. Send Orientation to Limelight 
         * Required for MegaTag2 to calculate the pose correctly.
         * We retrieve the robot's current heading from the drivetrain (Gyro).
         */
        var driveState = drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        
        LimelightHelpers.SetRobotOrientation(kLimelightName, headingDeg, 0, 0, 0, 0, 0);

        /* * 2. Read Pose Estimate 
         * getBotPoseEstimate_wpiBlue_MegaTag2 uses the orientation sent above 
         * to compute the robot's location on the field (Blue Alliance origin).
         */
        PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

        if (llMeasurement != null && llMeasurement.tagCount > 0) {
            /*
             * 3. Filter and Apply
             * Reject measurements if the robot is spinning too fast to avoid camera blur/latency issues.
             */
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
            if (Math.abs(omegaRps) < 2.0) {
                Matrix<N3, N1> stdDevs = getVisionStdDevs(llMeasurement);
                drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds, stdDevs);
            }
        }
    }

    /**
     * Calculates standard deviations based on the number of tags seen and their distance.
     * Lower numbers = more trust in the vision measurement.
     */
    private Matrix<N3, N1> getVisionStdDevs(PoseEstimate estimate) {
        double xyStds;
        double degStds;

        if (estimate.tagCount >= 2) {
            // High trust if multiple tags are visible
            xyStds = 0.5;
            degStds = 6;
        } else {
            // Scale trust based on distance to the single tag
            double dist = estimate.avgTagDist;
            if (dist > 4.0) {
                 xyStds = 3.0; 
                 degStds = 30;
            } else {
                 xyStds = 0.9 + (0.3 * dist);
                 degStds = 10 + (5 * dist);
            }
        }
        return VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
    }
}