
package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.Teleoperado;

import java.util.function.DoubleSupplier;
import swervelib.SwerveController;

public class Teleoperado extends Command {

  // Variáveis que guardam nossas funções do gamepad
  DoubleSupplier y;
  DoubleSupplier x;
  DoubleSupplier turn;

  // Objetos necessárias para acessar funções e variáveis
  SwerveDrive swerve;
  SwerveController controller;

  // Variáveis que guardam a translação e velocidade angular do swerve
  Translation2d translation;
  double angle;
  double omega;
  double velocity;
  double angleVelocity;

  XboxController controle1;
  CommandXboxController driverCommand = new CommandXboxController(0);
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /** Creates a new Teleoperado. */
  public Teleoperado(
      SwerveDrive swerve,
      DoubleSupplier y,
      DoubleSupplier x,
      DoubleSupplier turn) {

    this.y = y;
    this.x = x;
    this.turn = turn;
    this.swerve = swerve;

    controller = swerve.getSwerveController(); // Obtemos o controlador do swerve
    // Adiciona a tração como requerimento
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Classes feitas para limitar a velocidade máxima do robô.
  public double limitSpeed() {
    if (driverCommand.getLeftTriggerAxis() != 0) {
      return velocity = 1.0;
    } else {
      return velocity = 4.0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}