// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Elevator elevator;
    private final Arm arm;

    // Controller
    private final CommandPS4Controller controller = new CommandPS4Controller(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                elevator = new Elevator(new ElevatorIOReal());

                arm = new Arm(new ArmIOReal());

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                elevator = new Elevator(new ElevatorIOSim());

                arm = new Arm(new ArmIOSim());

                break;

            case REPLAY:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});

                elevator = new Elevator(new ElevatorIO() {});

                arm = new Arm(new ArmIO() {});

                break;
            default:
                throw new AssertionError(String.format("Unknown mode: %s", Constants.currentMode));
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive.
        // Xbox controller that I got is busted or something, the getRightY() binds to a trigger for some reason.
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> controller.getRightX()));

        // TODO: Enable later
        // Lock to 0° when A button is held
        //        controller
        //                .a()
        //                .whileTrue(DriveCommands.joystickDriveAtAngle(
        //                        drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> new
        // Rotation2d()));
        //
        //        // Switch to X pattern when X button is pressed
        //        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro to 0° when B button is pressed
        //        controller
        //                .b()
        //                .onTrue(Commands.runOnce(
        //                                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new
        // Rotation2d())),
        //                                drive)
        //                        .ignoringDisable(true));

        controller.povUp().onTrue(elevator.setElevatorPositionCommand(Elevator.ElevatorHeight.UP));
        controller.povLeft().onTrue(elevator.setElevatorPositionCommand(Elevator.ElevatorHeight.MIDDLE));
        controller.povDown().onTrue(elevator.setElevatorPositionCommand(Elevator.ElevatorHeight.DOWN));

        //        controller.y().onTrue(arm.setArmPositionCommand(Arm.ArmPositions.NORTH));
        //        controller.b().onTrue(arm.setArmPositionCommand(Arm.ArmPositions.EAST));
        //        controller.a().onTrue(arm.setArmPositionCommand(Arm.ArmPositions.SOUTH));
        //        controller.x().onTrue(arm.setArmPositionCommand(Arm.ArmPositions.WEST));

        controller.triangle().onTrue(arm.setArmPositionCommand(Arm.ArmPositions.NORTH));
        controller.circle().onTrue(arm.setArmPositionCommand(Arm.ArmPositions.EAST));
        controller.cross().onTrue(arm.setArmPositionCommand(Arm.ArmPositions.SOUTH));
        controller.square().onTrue(arm.setArmPositionCommand(Arm.ArmPositions.WEST));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
