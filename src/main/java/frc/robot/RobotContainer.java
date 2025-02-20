// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AutoAim;
import frc.robot.commands.HookDPAD;
import frc.robot.commands.NoteIntake;
import frc.robot.commands.ShootNote;
import frc.robot.commands.SwerveJoystickAuto;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.ReverseShooter;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final HookSubsystem hook = new HookSubsystem(Constants.Hook.leftHook, Constants.Hook.rightHook);
  // private final IntakeSubsystem intake = new IntakeSubsystem(Constants.Intake.m_1ID, Constants.Intake.m_1Inverted, Constants.Intake.m_2ID, Constants.Intake.m_2Inverted);
  private final ShooterSubsystem shooter = new ShooterSubsystem(Constants.Shooter.mID, Constants.Shooter.mInverted);
  private final PhotonVision photonVision = new PhotonVision("1");
  
  // * declare motor
  private final DCMotor dcMotor = new DCMotor(Constants.DCMotor.motorID); // TODO: add actual ID (1)
  private final double dcMotorSpeed = Constants.DCMotor.motorSpeed;

  private final XboxController xbox = new XboxController(0);
  private final Joystick joystick = new Joystick(0);
  private final Dpad dpad = new Dpad(joystick);


  public RobotContainer() {

    //create named commands for pathplanner here
      NamedCommands.registerCommand("Drop", new ShootNote(shooter, ()-> 0.1));

      
      swerveSubsystem.setDefaultCommand(new SwerveJoystickAuto(
      swerveSubsystem, 
      () -> xbox.getLeftY(),
      () -> -xbox.getLeftX(),
      () -> -xbox.getRightY(),
      () -> -xbox.getRightX()));


      shooter.setDefaultCommand(new ShootNote(shooter, () -> xbox.getRightTriggerAxis()));
    configureBindings();
  }

  private void configureBindings() {

    // ! old way of controlling motor. sets speed to 0.5 when button 1 is pressed, and -0.5 when button 2 is pressed
    new POVButton(joystick, 0).onTrue(new InstantCommand(() -> dcMotor.setSpeed(dcMotorSpeed)));
    new POVButton(joystick, 180).onTrue(new InstantCommand(() -> dcMotor.setSpeed(-dcMotorSpeed)));

    // * new way. adds 0.25 to speed or subtracts 0.25 from speed when button 1 or 2 is pressed, respectively
    // new POVButton(joystick, 0).onTrue(new InstantCommand(() -> dcMotor.addSpeed()));
    // new POVButton(joystick, 180).onTrue(new InstantCommand(() -> dcMotor.subtractSpeed()));
    // new POVButton(joystick, 270).onTrue(new InstantCommand(() -> dcMotor.stop()));

    new JoystickButton(xbox, 1).whileTrue(new InstantCommand(() -> dcMotor.setSpeed(dcMotorSpeed)));
    new JoystickButton(xbox, 2).whileTrue(new InstantCommand(() -> dcMotor.stop()));
    new JoystickButton(xbox, 3).whileTrue(new InstantCommand(() -> dcMotor.setSpeed(-dcMotorSpeed)));

    new JoystickButton(xbox, 4).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new POVButton(joystick, 0).whileTrue(new HookDPAD(hook, true));
    new POVButton(joystick, 180).whileTrue(new HookDPAD(hook, false));
    new JoystickButton(xbox, 5).whileTrue(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> xbox.getLeftY(),
      () -> -xbox.getLeftX(),
      () -> -xbox.getRightX()
      ));
    new JoystickButton(xbox, 3).whileTrue(new AutoAim(swerveSubsystem, photonVision));

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Drop It");
  }
}
