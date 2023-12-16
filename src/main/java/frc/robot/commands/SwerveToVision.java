package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveToVision extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<PhotonPipelineResult> visionInfo;
    private final PIDController xSpdController, ySpdController, turningSpdController;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private double xSpeed, ySpeed;
    private boolean finishOnTargetLoss;
    private PhotonPipelineResult visionResult;
    private int targetLostCounter;
    /**
     * This command writes swerveModuleStates to a photonvision target.
     * @param swerveSubsystem Requirement parameter
     * @param visionInfo PhotonPipelineResult class, input the result
     * @param finishOnTargetLoss IF true, command stops on target loss if LOST and NOT regained 
     * for 20 scheduler cycles. (400 ms) For every cycle there is a target, the counter loses 1.
     */
    public SwerveToVision(SwerveSubsystem swerveSubsystem, Supplier<PhotonPipelineResult> visionInfo, boolean finishOnTargetLoss) {
        this.swerveSubsystem = swerveSubsystem;
        this.visionInfo = visionInfo;
        xSpdController = new PIDController(0.011, 0, 0);
        xSpdController.setTolerance(2);
        ySpdController = new PIDController(0.013, 0, 0);
        ySpdController.setTolerance(2.5);
        turningSpdController = new PIDController(0.01, 0, 0);
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.finishOnTargetLoss = finishOnTargetLoss;

        targetLostCounter = 0;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // get vision info
        visionResult = visionInfo.get();
        double yaw;
        double pitch;
        double skew;
        if (visionResult.hasTargets() == true) {
                PhotonTrackedTarget target = visionResult.getBestTarget();
                yaw = target.getYaw();
                pitch = target.getPitch();
                skew = target.getSkew();
                // if (skew > 85) {
                //     yaw += 4;
                //     pitch += 6;
                // } else if (skew < -85) {
                //     yaw -= 4;
                //     pitch += 6;
                // }
                //xSpeed = xSpdController.calculate(yaw, 21.7);
               // ySpeed = -ySpdController.calculate(pitch, 3);
                xSpeed = -xSpdController.calculate(yaw, 0);
                ySpeed = ySpdController.calculate(pitch, 0);
                targetLostCounter = targetLostCounter > 0 ? (targetLostCounter - 1) : 0;
        } else {
            // if no target, all speeds are ZERO.
            //xSpdController.calculate(0, 21.7);
           // ySpdController.calculate(0, 3);
           // xSpdController.calculate(0, 19.5);
            //ySpdController.calculate(0, 9);
            xSpeed = 0;
            ySpeed = 0;
            targetLostCounter++;
        }

        // 1. Get real-time joystick inputs
        
        double turningSpeed = 0;
        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.3, 0);
        SmartDashboard.putNumber("visionxspeed", xSpeed);
        SmartDashboard.putNumber("visionyspeed", ySpeed);
        SmartDashboard.putNumber("visionturningspeed", turningSpeed);
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SmartDashboard.putString("testBRD", moduleStates[3].toString());
        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() { 
        // If finishontargetloss is true, the command will exit if it loses its target. 
        // Good for distinction between autonomous and teleoperated control. 
        // We would want it to complete if it loses target in teleop, but not in autonomous such that it does not complete the rest of the sequence.
        return finishOnTargetLoss == true ? 
        (xSpdController.atSetpoint() && ySpdController.atSetpoint()) || (targetLostCounter >= 20): 
        xSpdController.atSetpoint() && ySpdController.atSetpoint();
    }
}
