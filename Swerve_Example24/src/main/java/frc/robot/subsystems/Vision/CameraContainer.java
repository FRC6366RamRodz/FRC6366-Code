package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;

public interface CameraContainer {
  PhotonPipelineResult getFilteredResult();

  Optional<Pose2d> getEstimatedPose();

  double getLatency();

  boolean hasTargets();

  int getTargetCount();

  List<EntechTargetData> getTargetData();
}