package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Dimensoes;
import frc.robot.commands.Teleoperado;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

public class SwerveDrive extends SubsystemBase {
    public swervelib.SwerveDrive swerveDrive;

    Teleoperado teleop;

    double velocity;

    CommandXboxController driverCommand = new CommandXboxController(0);

    @SuppressWarnings("unused")
    private Field2d field2d = new Field2d();

    public SwerveDrive(File directory) {
        // Seta a telemetria como nível mais alto
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Acessa os arquivos do diretório .JSON
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(teleop.limitSpeed(),
                    Dimensoes.angleConversion, Dimensoes.driveConversion);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    @SuppressWarnings("unused")
    private final boolean visionDriveTest = false;

    @Override
    public void periodic() {
    }

    public void setHeadingCorrection(boolean heading) {
        swerveDrive.setHeadingCorrection(heading);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveController getSwerveController() {
        return swerveDrive.getSwerveController();
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
                getHeading().getRadians());
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d heading) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                heading.getRadians(),
                getHeading().getRadians(),
                teleop.limitSpeed());
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double maxSpeed) {
        return swerveDrive.swerveController.getTargetSpeeds(xInput,
                yInput, 0, 0, 0, maxSpeed);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    public void fowardDef() {
        swerveDrive.zeroGyro();
    }

    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    public void drive(Translation2d translation, double rotation) {
        swerveDrive.drive(translation, rotation, true, false);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {
            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                    translationY.getAsDouble(),
                    rotation.getAsDouble() * Math.PI,
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumVelocity()));
        });
    }
}