package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private boolean useVision = true;

    // Simulation fields
    private AprilTagFieldLayout layout;
    private final DoubleArrayEntry simEntry;
    private final StructPublisher<Pose2d> ghostPub;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Initialize NetworkTables
        NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        // Topic to write simulated data to
        simEntry = llTable.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry(new double[0]);
        
        // Ghost Publisher for AdvantageScope (Clean visualization)
        ghostPub = llTable.getStructTopic("ghost_pose", Pose2d.struct).publish();

        // Load field layout ONLY for simulation (to calculate distance for std devs)
        if (RobotBase.isSimulation()) {
            try {
                layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
            } catch (Exception e) {
                System.out.println("Could not load AprilTag layout for simulation: " + e.getMessage());
            }
        }
    }

    @Override
    public void periodic() {
        if (!useVision) return;

        /* * 1. Send Orientation to Limelight 
         * Required for MegaTag2 to calculate the pose correctly
         */
        var driveState = drivetrain.getState();
        double headingDeg = driveState.Pose.getRotation().getDegrees();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);

        /* * 2. Read Pose Estimate 
         * This reads "botpose_orb_wpiblue". In Sim, this reads what we write in simulationPeriodic()
         */
        PoseEstimate llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (llMeasurement != null && llMeasurement.tagCount > 0) {
            // Filter out measurements if the robot is spinning too fast (blur/latency)
            if (Math.abs(omegaRps) < 2.0) {
                Matrix<N3, N1> stdDevs = getVisionStdDevs(llMeasurement);
                drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds, stdDevs);
            }
        }
    }

    @Override
    public void simulationPeriodic() {
        if (layout == null) return;

        // 1. Get "Ground Truth" Pose (In Sim, we assume Odometry is perfect ground truth)
        Pose2d truePose = drivetrain.getState().Pose;

        // 2. Find closest tag to simulate "seeing" it
        Optional<Pose3d> closestTag = layout.getTags().stream()
            .map(tag -> layout.getTagPose(tag.ID).get())
            .filter(tagPose -> tagPose.toPose2d().getTranslation().getDistance(truePose.getTranslation()) < 5.0) 
            .min((a, b) -> Double.compare(
                a.toPose2d().getTranslation().getDistance(truePose.getTranslation()),
                b.toPose2d().getTranslation().getDistance(truePose.getTranslation())
            ));

        if (closestTag.isPresent()) {
            // 3. Generate Fake Vision Data with Noise
            double noiseX = (Math.random() - 0.5) * 0.05; // +/- 2.5cm
            double noiseY = (Math.random() - 0.5) * 0.05;
            double noiseRot = (Math.random() - 0.5) * Units.degreesToRadians(1); // +/- 0.5 deg

            Pose2d noisyPose = new Pose2d(
                truePose.getX() + noiseX, 
                truePose.getY() + noiseY, 
                truePose.getRotation().plus(new edu.wpi.first.math.geometry.Rotation2d(noiseRot))
            );

            // 4. Calculate simulated metrics
            double distToTag = closestTag.get().toPose2d().getTranslation().getDistance(truePose.getTranslation());
            int tagCount = 1; 
            double latency = 0; 

            // 5. Pack into Limelight Format (DEGREES for rotation)
            // We use the standard 11-value array. We do NOT populate fiducials, preventing the crash.
            double[] botpose = new double[] {
                noisyPose.getX(),
                noisyPose.getY(),
                0.0, 
                0.0, 
                0.0, 
                noisyPose.getRotation().getDegrees(), 
                latency,
                tagCount,
                0.0, 
                distToTag,
                0.0  
            };

            // 6. Write to NT
            simEntry.set(botpose);
            
            // 7. Write to Ghost Struct (RADIANS for AdvantageScope)
            ghostPub.set(noisyPose); 

        } else {
            // Clear outputs if no tag is seen
            simEntry.set(new double[0]);
            ghostPub.set(null);
        }
    }

    private Matrix<N3, N1> getVisionStdDevs(PoseEstimate estimate) {
        double xyStds;
        double degStds;

        if (estimate.tagCount >= 2) {
            xyStds = 0.5;
            degStds = 6;
        } else {
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