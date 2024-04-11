// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.NavigableMap;
import java.util.NoSuchElementException;
import java.util.TreeMap;

public class PoseEstimator {
  private static final double historyLengthSecs = 0.2;
  private static final double poseBufferSizeSeconds = 2.0;
  private Pose2d estimatedPose = new Pose2d();
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

  private Pose2d basePose = new Pose2d();

  private Pose2d latestPose = new Pose2d();
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();
  private final Matrix<N3, N1> q = new Matrix<>(Nat.N3(), Nat.N1());

  public PoseEstimator(Matrix<N3, N1> stateStdDevs) {
    for (int i = 0; i < 3; ++i) {
      q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }

    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(Constants.odometryStateStdDevs.get(i, 0), 2));
    }
  }

    /** Returns the latest robot pose based on drive and vision data. */
  public Pose2d getLatestPose() {
    return latestPose;
  }

  /** Resets the odometry to a known pose. */
  public void resetPose(Pose2d pose) {
    basePose = pose;
    updates.clear();
    update();
  }

  /** Records a new drive movement. */
  public void addDriveData(double timestamp, Twist2d twist) {
    updates.put(timestamp, new PoseUpdate(twist, new ArrayList<>()));
    update();
  }

  /** Records a new set of vision updates. */
  public void addVisionData(List<TimestampedVisionUpdate> visionData) {
    for (var timestampedVisionUpdate : visionData) {
      var timestamp = timestampedVisionUpdate.timestamp();
      var visionUpdate =
          new VisionUpdate(timestampedVisionUpdate.pose(), timestampedVisionUpdate.stdDevs());

      if (updates.containsKey(timestamp)) {
        // There was already an update at this timestamp, add to it
        var oldVisionUpdates = updates.get(timestamp).visionUpdates();
        oldVisionUpdates.add(visionUpdate);
        oldVisionUpdates.sort(VisionUpdate.compareDescStdDev);

      } else {
        // Insert a new update
        var prevUpdate = updates.floorEntry(timestamp);
        var nextUpdate = updates.ceilingEntry(timestamp);
        if (prevUpdate == null || nextUpdate == null) {
          // Outside the range of existing data
          return;
        }

        // Create partial twists (prev -> vision, vision -> next)
        var twist0 =GeomUtil.multiplyTwist(nextUpdate.getValue().twist(), (timestamp - prevUpdate.getKey()) / (nextUpdate.getKey() - prevUpdate.getKey()));
        
        var twist1 = GeomUtil.multiplyTwist(nextUpdate.getValue().twist(), (nextUpdate.getKey() - timestamp) / (nextUpdate.getKey() - prevUpdate.getKey()));

        // Add new pose updates
        var newVisionUpdates = new ArrayList<VisionUpdate>();
        newVisionUpdates.add(visionUpdate);
        newVisionUpdates.sort(VisionUpdate.compareDescStdDev);
        updates.put(timestamp, new PoseUpdate(twist0, newVisionUpdates));
        updates.put(nextUpdate.getKey(), new PoseUpdate(twist1, nextUpdate.getValue().visionUpdates()));
      }
    }

    // Recalculate latest pose once
    update();
  }

   public void addVisionObservation(List<TimestampedVisionUpdate> observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
          > observation.get(1).timestamp) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(observation.get(1).timestamp);
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), getLatestPose());
    var odometryToSampleTransform = new Transform2d(getLatestPose(), sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = observation.get(3).stdDevs.get(i, 0) * observation.get(3).stdDevs.get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, observation.get(2).pose);
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    latestPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  /** Clears old data and calculates the latest pose. */
  private void update() {
    // Clear old data and update base pose
    while (updates.size() > 1
        && updates.firstKey() < Timer.getFPGATimestamp() - historyLengthSecs) {
      var update = updates.pollFirstEntry();
      basePose = update.getValue().apply(basePose, q);
    }

    // Update latest pose
    latestPose = basePose;
    for (var updateEntry : updates.entrySet()) {
      latestPose = updateEntry.getValue().apply(latestPose, q);
    }
  }

  /**
   * Represents a sequential update to a pose estimate, with a twist (drive movement) and list of
   * vision updates.
   */
  private record PoseUpdate(Twist2d twist, ArrayList<VisionUpdate> visionUpdates) {
    public Pose2d apply(Pose2d lastPose, Matrix<N3, N1> q) {
      // Apply drive twist
      var pose = lastPose.exp(twist);

      // Apply vision updates
      for (var visionUpdate : visionUpdates) {
        // Calculate Kalman gains based on std devs
        // (https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/math/estimator/)
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
          r[i] = visionUpdate.stdDevs().get(i, 0) * visionUpdate.stdDevs().get(i, 0);
        }
        for (int row = 0; row < 3; ++row) {
          if (q.get(row, 0) == 0.0) {
            visionK.set(row, row, 0.0);
          } else {
            visionK.set(
                row, row, q.get(row, 0) / (q.get(row, 0) + Math.sqrt(q.get(row, 0) * r[row])));
          }
        }

        // Calculate twist between current and vision pose
        var visionTwist = pose.log(visionUpdate.pose());

        // Multiply by Kalman gain matrix
        var twistMatrix =
            visionK.times(VecBuilder.fill(visionTwist.dx, visionTwist.dy, visionTwist.dtheta));

        // Apply twist
        pose = pose.exp(new Twist2d(twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0)));
      }

      return pose;
    }
  }

  /** Represents a single vision pose with associated standard deviations. */
  public record VisionUpdate(Pose2d pose, Matrix<N3, N1> stdDevs) {
    public static final Comparator<VisionUpdate> compareDescStdDev =
        (VisionUpdate a, VisionUpdate b) -> {
          return -Double.compare(
              a.stdDevs().get(0, 0) + a.stdDevs().get(1, 0),
              b.stdDevs().get(0, 0) + b.stdDevs().get(1, 0));
        };
  }

  /** Represents a single vision pose with a timestamp and associated standard deviations. */
  public record TimestampedVisionUpdate(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}
}