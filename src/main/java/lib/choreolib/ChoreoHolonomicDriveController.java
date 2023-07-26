package lib.choreolib;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Custom version of a @HolonomicDriveController specifically for following PathPlanner paths
 *
 * <p>This controller adds the following functionality over the WPILib version: - calculate() method
 * takes in a PathPlannerState directly - Continuous input is automatically enabled for the rotation
 * controller - Holonomic angular velocity is used as a feedforward for the rotation controller,
 * which no longer needs to be a @ProfiledPIDController
 */
public class ChoreoHolonomicDriveController {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private Translation2d translationError = new Translation2d();
  private Rotation2d rotationError = new Rotation2d();
  private Pose2d tolerance = new Pose2d();
  private boolean isEnabled = true;

  /**
   * Constructs a PPHolonomicDriveController
   *
   * @param xController A PID controller to respond to error in the field-relative X direction
   * @param yController A PID controller to respond to error in the field-relative Y direction
   * @param rotationController A PID controller to respond to error in rotation
   */
  public ChoreoHolonomicDriveController(
      PIDController xController, PIDController yController, PIDController rotationController) {
    this.xController = xController;
    this.yController = yController;
    this.rotationController = rotationController;

    // Auto-configure continuous input for rotation controller
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    Translation2d translationTolerance = this.tolerance.getTranslation();
    Rotation2d rotationTolerance = this.tolerance.getRotation();

    return Math.abs(this.translationError.getX()) < translationTolerance.getX()
        && Math.abs(this.translationError.getY()) < translationTolerance.getY()
        && Math.abs(this.rotationError.getRadians()) < rotationTolerance.getRadians();
  }

  /**
   * Sets the pose error whic is considered tolerance for use with atReference()
   *
   * @param tolerance The pose error which is tolerable
   */
  public void setTolerance(Pose2d tolerance) {
    this.tolerance = tolerance;
  }

  /**
   * Enables and disables the controller for troubleshooting. When calculate() is called on a
   * disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not
   */
  public void setEnabled(boolean enabled) {
    this.isEnabled = enabled;
  }

  /**
   * Calculates the next output of the holonomic drive controller
   *
   * @param currentPose The current pose
   * @param referenceState The desired trajectory state
   * @return The next output of the holonomic drive controller
   */
  public ChassisSpeeds calculate(Pose2d currentPose, ChoreoTrajectoryState referenceState) {
    double xFF =
        referenceState.velocityX;
    double yFF =
        referenceState.velocityY;
    double rotationFF = referenceState.angularVelocity;
    
    
    this.translationError = referenceState.getPose().relativeTo(currentPose).getTranslation();
    this.rotationError = new Rotation2d(referenceState.heading).minus(currentPose.getRotation());

    if (!this.isEnabled) {
      return 
      
        ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, rotationFF, currentPose.getRotation());
    }

    double xFeedback =
        this.xController.calculate(currentPose.getX(), referenceState.x);
    double yFeedback =
        this.yController.calculate(currentPose.getY(), referenceState.y);
    double rotationFeedback =
        this.rotationController.calculate(
            currentPose.getRotation().getRadians(), referenceState.heading);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
  }
}

