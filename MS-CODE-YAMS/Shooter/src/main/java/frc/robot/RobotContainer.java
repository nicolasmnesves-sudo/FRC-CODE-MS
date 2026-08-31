// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Joysticks.KeyboardController;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;




public class RobotContainer
{
  private ShooterSubsystem shooter = new ShooterSubsystem();
  private ArmSubsystem arm = new ArmSubsystem();
  private IntakeSubsystem intake = new IntakeSubsystem();
  
  public CommandXboxController xboxController = new CommandXboxController(0);
  public KeyboardController keyboard = new KeyboardController(1);

  public RobotContainer()
  {
 
    DriverStation.silenceJoystickConnectionWarning(true);
    shooter.setDefaultCommand(shooter.setDutyCycle(0));
    configureBindings();
  }


  private void configureBindings()
  {
    //Shooter Xbox Commands
    xboxController.a().onTrue(shooter.setVelocity(RPM.of(300)));
    xboxController.b().onTrue(shooter.setVelocity(RPM.of(0)));


    //Arm Xbox Commands
    xboxController.leftBumper().whileTrue(arm.armCmd(0.2));
    xboxController.rightBumper().whileTrue(arm.armCmd(-0.2));

    //Arm Keyboard Commands
    keyboard.getZTrigger().onTrue(arm.setAngle(Degrees.of(0)));
    keyboard.getXTrigger().onTrue(arm.setAngle(Degrees.of(45)));
    keyboard.getCTrigger().onTrue(arm.setAngle(Degrees.of(90)));

    //Intake keyboard Commands
    keyboard.getVTrigger().onTrue(intake.setDutyCycle(0.8));
    keyboard.getBTrigger().onTrue(intake.setDutyCycle(-0.8));

    xboxController.x().onTrue(intake.setDutyCycle(-1));
    xboxController.y().onTrue(intake.setDutyCycle(1));
  }


  public Command getAutonomousCommand()
  {
    return Commands.print("No autonomous command configured");
  }
}