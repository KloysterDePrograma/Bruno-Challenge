// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@SuppressWarnings("unused")
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
}