package frc.robot;

import frc.robot.Constants.Controle;
import frc.robot.commands.ConfigAuto;
import frc.robot.commands.Teleoperado;
import frc.robot.subsystems.SwerveDrive;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  static final SwerveDrive swerve = new SwerveDrive(new File(Filesystem.getDeployDirectory(), "swerve"));

  CommandXboxController driverCommand = new CommandXboxController(0);
  CommandXboxController operatorCommand = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // É onde definimos os analógicos para movimentação do robô.
    swerve.setDefaultCommand(
      new Teleoperado(swerve, 
      () -> -MathUtil.applyDeadband(driverCommand.getRawAxis(1), 0.1),
      () -> -MathUtil.applyDeadband(driverCommand.getRawAxis(0), 0.1),
      () -> -MathUtil.applyDeadband(driverCommand.getRawAxis(4), 0.1)));

    configureBindings();
  }
  
  private void configureBindings() {
    driverCommand.button(1).onTrue(Commands.runOnce(() -> swerve.fowardDef(), swerve));
  }

      // Usado para adquirir o gatilho direito para um Shooter mais automático, e facilitar a pontuação no Speaker.
    private boolean getRight() {
    if (operatorCommand.getRawAxis(Controle.rightTrigger) != 0) {
      return true;
    }
    return false;
  }

  
    // Usado para adquirir o gatilho esquerdo para um Shooter mais automático, e facilitar a pontuação no Amp.
  private boolean getLeft() {
    if (operatorCommand.getRawAxis(Controle.leftTrigger) != 0) {
      return true;
    }
    return false;
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public void setHeadingCorrection(boolean headingCorrection) {
    swerve.swerveDrive.setHeadingCorrection(headingCorrection);
  }
}