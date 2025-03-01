// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.autonomous.PathCommand;
import frc.robot.subsystems.rollers.RollerSensorsIOComp;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.RollerState;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.rollers.intake.IntakeIOTalonFX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.superstructure.pivot.Pivot;
import frc.robot.subsystems.superstructure.pivot.PivotIO;
import frc.robot.subsystems.superstructure.pivot.PivotIOTalonFX;
import frc.robot.subsystems.superstructure.tongue.Tongue;
import frc.robot.subsystems.superstructure.tongue.TongueIO;
import frc.robot.subsystems.superstructure.tongue.TongueIOServo;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIOPigeon2;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonvision;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final RobotState robotState = RobotState.getInstance();

  private SendableChooser<Command> autoChooser;

  private final CommandXboxController driverA = new CommandXboxController(0);
  private final CommandXboxController driverB = new CommandXboxController(1);

  private Drive swerve;
  private Vision vision;
  private Intake intake;
  private Rollers rollers;
  private Elevator elevator;
  private Pivot pivot;
  private Tongue tongue;
  private Superstructure superstructure;

  public RobotContainer() {
    intake = null;

    if (Constants.getRobotMode() != Mode.REPLAY) {
      switch (Constants.getRobotType()) {
        case COMP -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          vision =
              new Vision(
                  new VisionIOPhotonvision(1),
                  new VisionIOPhotonvision(2),
                  new VisionIOPhotonvision(3),
                  new VisionIOPhotonvision(4),
                  new VisionIOPhotonvision(5));
          intake = new Intake(new IntakeIOTalonFX());
          // superstructure stuff
          elevator = new Elevator(new ElevatorIOTalonFX());
          pivot = new Pivot(new PivotIOTalonFX());
          tongue = new Tongue(new TongueIOServo());
        }
        case PROG -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX());
          elevator = new Elevator(new ElevatorIOTalonFX());
          pivot = new Pivot(new PivotIOTalonFX());
        }
        case ALPHA -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          intake = new Intake(new IntakeIOTalonFX());
          pivot = new Pivot(new PivotIOTalonFX());
          elevator = new Elevator(new ElevatorIOTalonFX());
        }
        case SIM -> {
          swerve =
              new Drive(
                  new GyroIOPigeon2(),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[0]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[1]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[2]),
                  new ModuleIOTalonFX(DriveConstants.MODULE_CONFIGS[3]));
          pivot = new Pivot(new PivotIOTalonFX());
        }
      }
    }

    if (swerve == null) {
      swerve =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      vision = new Vision();
    }

    rollers = new Rollers(intake, new RollerSensorsIOComp());

    if (elevator == null) {
      elevator = new Elevator(new ElevatorIO() {});
    }
    if (pivot == null) {
      pivot = new Pivot(new PivotIO() {});
    }
    if (tongue == null) {
      tongue = new Tongue(new TongueIO() {});
    }
    superstructure = new Superstructure(elevator, pivot, tongue);

    configureBindings();
    configureAutos();
  }

  private void configureBindings() {

    // -----Driver Controls-----
    swerve.setDefaultCommand(
        swerve
            .run(
                () -> {
                  swerve.driveTeleopController(
                      -driverA.getLeftY(),
                      -driverA.getLeftX(),
                      driverA.getLeftTriggerAxis() - driverA.getRightTriggerAxis());
                  if (Math.abs(driverA.getLeftTriggerAxis()) > 0.1
                      || Math.abs(driverA.getRightTriggerAxis()) > 0.1) {
                    swerve.clearHeadingControl();
                  }
                })
            .withName("Drive Teleop"));

    new Trigger(
            () -> (Math.abs(driverA.getRightY()) > 0.2) || (Math.abs(driverA.getRightX()) > 0.2))
        .whileTrue(
            new FunctionalCommand(
                () -> {},
                () ->
                    swerve.setTargetHeading(
                        calculateSnapTargetHeading(
                            new Rotation2d(
                                Math.atan2(
                                    MathUtil.applyDeadband(-driverA.getRightX(), 0.1),
                                    MathUtil.applyDeadband(-driverA.getRightY(), 0.1))))),
                interrupted -> {},
                () -> false));

    driverA.start().onTrue(swerve.zeroGyroCommand());

    driverA
        .x()
        .onTrue(
            new InstantCommand(() -> swerve.setTargetHeading(new Rotation2d(Math.toRadians(128)))));
    driverA
        .b()
        .onTrue(
            new InstantCommand(() -> swerve.setTargetHeading(new Rotation2d(Math.toRadians(232)))));

    // driverA
    //     .y()
    //     .onTrue(
    //         new InstantCommand(() -> tongue.setPositionTarget()));
    // -----Superstructure Controls-----
    // L1
    new Trigger(
            () ->
                ((rollers.readyToRaise()
                        || superstructure.getTargetState() != SuperstructureState.INTAKE))
                    && driverB.povDown().getAsBoolean())
        .onTrue(
            superstructure
                .goToStateCommand(SuperstructureState.L1));
    // L2
    new Trigger(
            () ->
                ((rollers.readyToRaise()
                        || superstructure.getTargetState() != SuperstructureState.INTAKE))
                    && driverB.povRight().getAsBoolean())
        .onTrue(
            superstructure
                .goToStateCommand(SuperstructureState.L2));
    //Go to L3
    new Trigger(
            () ->
                ((rollers.readyToRaise()
                        || superstructure.getTargetState() != SuperstructureState.INTAKE))
                    && driverB.povLeft().getAsBoolean())
        .onTrue(superstructure.goToStateCommand(SuperstructureState.SCORE_L3));
    //Go to L4
    new Trigger(
            () ->
                ((rollers.readyToRaise()
                        || superstructure.getTargetState() != SuperstructureState.INTAKE))
                    && driverB.povUp().getAsBoolean())
        .onTrue(superstructure.goToStateCommand(SuperstructureState.SCORE_L4));

    driverB // ZERO our mechanism
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.setCurrentState(SuperstructureState.ZERO);
                },
                superstructure));
  
    //Stop everything
    driverB
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  superstructure.setStopped(true);
                  ;
                  rollers.setTargetState(RollerState.IDLE);
                }));

    //kinda manual commands
    driverB.leftBumper().onTrue(superstructure.goToStateCommand(SuperstructureState.STOW));
    driverB.b().onTrue(superstructure.goToStateCommand(SuperstructureState.TOP).alongWith(superstructure.oneTimeOverrideCommand()));

    // Manual override
    driverB
        .y()
        .onTrue(
          superstructure.oneTimeOverrideCommand());

    driverB // intake
        .leftTrigger()
        .onTrue(
            new SequentialCommandGroup(
                superstructure.goToStateCommand(SuperstructureState.INTAKE),
                rollers.setTargetCommand(RollerState.FORCE_INTAKE)));

    //Eject on L1 and L2
    new Trigger(
            () ->
                (superstructure.getTargetState().equals(SuperstructureState.L1)
                        || superstructure.getTargetState().equals(SuperstructureState.L2))
                    && driverB.rightTrigger().getAsBoolean())
        .onTrue(
            rollers
                .setTargetCommand(RollerState.INTAKE)
                .andThen(
                    new WaitCommand(0.5)
                        .andThen(rollers.setTargetCommand(RollerState.INTAKE))
                        .andThen(superstructure.goToStateCommand(SuperstructureState.INTAKE))));
    //Eject if not at L1 or L2
    new Trigger(
            () ->
                !(superstructure.getTargetState().equals(SuperstructureState.L1)
                        || superstructure.getTargetState().equals(SuperstructureState.L2))
                    && driverB.rightTrigger().getAsBoolean())
        .onTrue(
            rollers
                .setTargetCommand(RollerState.EJECT)
                .andThen(
                    new WaitCommand(0.5)
                        .andThen(rollers.setTargetCommand(RollerState.INTAKE))
                        .andThen(superstructure.goToStateCommand(SuperstructureState.INTAKE))));
    //Eject on L4 with sensors
    new Trigger(() -> (superstructure.getCurrentState() == SuperstructureState.SCORE_L4))
        .onTrue(
            new SequentialCommandGroup(
                new WaitCommand(0.1),
                rollers.setTargetCommand(RollerState.EJECT),
                new WaitCommand(0.2),
                superstructure.goToStateCommand(SuperstructureState.INTAKE),
                new WaitCommand(0.9),
                rollers.setTargetCommand(RollerState.FORCE_INTAKE)));
    // new SequentialCommandGroup(
    //     new ParallelCommandGroup(
    //         elevator.goToPositionCommand(ElevatorTarget.SETUP_INTAKE),
    //         pivot.goToPositionCommand(PivotTarget.INTAKE)),
    //     rollers.setTargetCommand(RollerState.INTAKE),
    //     elevator.goToPositionCommand(ElevatorTarget.INTAKE),
    //     new WaitUntilCommand(() -> rollers.getTargetState() == RollerState.HOLD),
    //     elevator.goToPositionCommand(ElevatorTarget.SETUP_INTAKE),
    //     pivot.goToPositionCommand(PivotTarget.TOP),
    //     elevator
    //         .goToPositionCommand(ElevatorTarget.BOTTOM)
    //         .alongWith(rollers.setTargetCommand(RollerState.IDLE))));

    // driverB // eject
    //     .rightTrigger()
    //     .onTrue(
    //         rollers
    //             .setTargetCommand(Rollers.RollerState.EJECT)
    //             .alongWith(pivot.goToPositionCommand(PivotTarget.SCORE_L4))
    //             .andThen(elevator.goToPositionCommand(ElevatorTarget.L1))
    //             .andThen(rollers.setTargetCommand(RollerState.IDLE)));
  }

  private void configureAutos() {

    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      robotConfig = null;
    }

    var passRobotConfig = robotConfig; // workaround

    BooleanSupplier flipAlliance =
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        };

    AutoBuilder.configureCustom(
        (path) -> new PathCommand(path, flipAlliance, swerve, passRobotConfig),
        () -> RobotState.getInstance().getEstimatedPose(),
        (pose) -> RobotState.getInstance().resetPose(pose),
        flipAlliance,
        true);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutoCommand() {
    return autoChooser.getSelected();
  }

  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    double a = ((currentAngle - newAngle) % 360 + 360) % 360;
    double b = ((currentAngle - newAngle) % 360 + 360) % 360;
    return a < b ? a : -b;
  }

  public static Rotation2d calculateSnapTargetHeading(Rotation2d targetHeading) {
    targetHeading =
        targetHeading.rotateBy(
            new Rotation2d(Math.PI + Math.toRadians(30))); // because back of robot
    double closest = DriveConstants.REEF_SNAP_ANGLES[0];
    for (double snap : DriveConstants.REEF_SNAP_ANGLES) {
      if (Math.abs(relativeAngularDifference(targetHeading.getDegrees(), snap))
          < Math.abs(relativeAngularDifference(targetHeading.getDegrees(), closest))) {
        closest = snap;
      }
    }
    return new Rotation2d(Math.toRadians(closest));
  }
}
