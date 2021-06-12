// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.JoystickPOVButton;
import frc.robot.util.XboxControllerAxisButton;
import frc.robot.util.XboxPOVButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /**
   * Declare Driver Buttons
   */
  private JoystickButton a_xBox_Driver;
  private JoystickButton b_xBox_Driver;
  private JoystickButton x_xBox_Driver;
  private JoystickButton y_xBox_Driver;
  private JoystickButton lb_xBox_Driver;
  private JoystickButton rb_xBox_Driver;
  private JoystickButton r_Stick_Button_xbox_Driver;
  private JoystickButton l_Stick_Button_xbox_Driver;
  private JoystickButton start_xBox_Driver;
  private JoystickButton reset_xBox_Driver;

  private XboxControllerAxisButton rt_xBox_Driver;
  private XboxControllerAxisButton lt_xBox_Driver;

  private XboxPOVButton povNorth_xBox_Driver;
  private XboxPOVButton povNorthEast_xBox_Driver;
  private XboxPOVButton povNorthWest_xBox_Driver;
  private XboxPOVButton povSouth_xBox_Driver;
  private XboxPOVButton povSouthEast_xBox_Driver;
  private XboxPOVButton povSouthWest_xBox_Driver;
  private XboxPOVButton povWest_xBox_Driver;
  private XboxPOVButton povEast_xBox_Driver;

  /**
   * Declare CoDriver Buttons
   */
  private JoystickButton a_xBox_CoDriver;
  private JoystickButton b_xBox_CoDriver;
  private JoystickButton x_xBox_CoDriver;
  private JoystickButton y_xBox_CoDriver;
  private JoystickButton lb_xBox_CoDriver;
  private JoystickButton rb_xBox_CoDriver;
  private JoystickButton r_Stick_Button_xBox_CoDriver;
  private XboxControllerAxisButton rJoystickUpDown_CoDriver;
  private JoystickButton l_Stick_Button_xBox_CoDriver;
  private XboxControllerAxisButton lJoystickUpDown_CoDriver;
  private JoystickButton start_xBox_CoDriver;
  private JoystickButton reset_xBox_CoDriver;
  private JoystickButton lsb_xBox_CoDriver;

  private XboxControllerAxisButton rt_xBox_CoDriver;
  private XboxControllerAxisButton lt_xBox_CoDriver;

  private XboxPOVButton povNorth_xBox_CoDriver;
  private XboxPOVButton povNorthEast_xBox_CoDriver;
  private XboxPOVButton povNorthWest_xBox_CoDriver;
  private XboxPOVButton povSouth_xBox_CoDriver;
  private XboxPOVButton povSouthEast_xBox_CoDriver;
  private XboxPOVButton povSouthWest_xBox_CoDriver;
  private XboxPOVButton povWest_xBox_CoDriver;
  private XboxPOVButton povEast_xBox_CoDriver;

  private JoystickButton btn1_launchPad;
  private JoystickButton btn2_launchPad;
  private JoystickButton btn3_launchPad;
  private JoystickButton btn4_launchPad;
  private JoystickButton btn5_launchPad;
  private JoystickButton btn6_launchPad;
  private JoystickButton btn7_launchPad;
  private JoystickButton btn8_launchPad;
  private JoystickButton btn9_launchPad;
  private JoystickButton btn10_launchPad;
  private JoystickButton btn11_launchPad;

  private static RobotContainer m_robotContainer = new RobotContainer();

  // The robot's subsystems
  private final TurretFeed m_turretFeed = new TurretFeed();
  private final TurretAim m_turretAim = new TurretAim();
  private final FlyWheel m_FlyWheel = new FlyWheel();
  private final Intake m_intake = new Intake();
  private final DriveTrain m_driveTrain = new DriveTrain();

  // Joysticks
  private final XboxController coDriverControlls = new XboxController(1);
  private final XboxController driverControlls = new XboxController(0);
  private final Joystick launchpad = new Joystick(2);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_driveTrain.setDefaultCommand(new DriveWithJoystick(m_driveTrain, driverControlls));

    // Configure autonomous sendable chooser

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    diverbuttons();
    codriverButtons();
    launchPadButtons();
    SmartDashboardButtons();

  }

  private void diverbuttons() {

    a_xBox_Driver = new JoystickButton(driverControlls, XboxController.Button.kA.value);
    a_xBox_Driver.whenPressed(new IntakeExtend(m_intake), true);

    b_xBox_Driver = new JoystickButton(driverControlls, XboxController.Button.kB.value);
    b_xBox_Driver.whenPressed(new IntakeRetract(m_intake), false);

    x_xBox_Driver = new JoystickButton(driverControlls, XboxController.Button.kX.value);
    x_xBox_Driver.whileHeld(new IntakePowercell(m_intake), true);
    ;

    y_xBox_Driver = new JoystickButton(driverControlls, XboxController.Button.kY.value);
    y_xBox_Driver.whileHeld(new IntakeEjectPowercell(m_intake), true);

    lb_xBox_Driver = new JoystickButton(driverControlls, XboxController.Button.kBumperLeft.value);
    lb_xBox_Driver.whenPressed(new IntakeExtend(m_intake), true);

    rb_xBox_Driver = new JoystickButton(driverControlls, XboxController.Button.kBumperRight.value);
    rb_xBox_Driver.whileHeld(new FlyWheel_Run(m_FlyWheel, driverControlls));

    r_Stick_Button_xbox_Driver = new JoystickButton(driverControlls, XboxController.Button.kStickRight.value);
    // r_Stick_Button_xbox_Driver;

    l_Stick_Button_xbox_Driver = new JoystickButton(driverControlls, XboxController.Button.kStickLeft.value);
    // l_Stick_Button_xbox_Driver;

    start_xBox_Driver = new JoystickButton(driverControlls, XboxController.Button.kStart.value);
    // start_xBox_Driver;

    reset_xBox_Driver = new JoystickButton(driverControlls, XboxController.Button.kBack.value);
    // reset_xBox_Driver;

    rt_xBox_Driver = new XboxControllerAxisButton(driverControlls, XboxController.Axis.kRightTrigger);
    // rt_xBox_Driver;

    lt_xBox_Driver = new XboxControllerAxisButton(driverControlls, XboxController.Axis.kLeftTrigger);
    // lt_xBox_Driver;

    povNorth_xBox_Driver = new XboxPOVButton(driverControlls, XboxPOVButton.NORTH);
    // povNorth_xBox_Driver;

    povNorthEast_xBox_Driver = new XboxPOVButton(driverControlls, XboxPOVButton.NORTHEAST);
    // povNorthEast_xBox_Driver;

    povNorthWest_xBox_Driver = new XboxPOVButton(driverControlls, XboxPOVButton.NORTHWEST);
    // povNorthWest_xBox_Driver;

    povSouth_xBox_Driver = new XboxPOVButton(driverControlls, XboxPOVButton.SOUTH);
    // povSouth_xBox_Driver;

    povSouthEast_xBox_Driver = new XboxPOVButton(driverControlls, XboxPOVButton.SOUTHEAST);
    // povSouthEast_xBox_Driver;

    povSouthWest_xBox_Driver = new XboxPOVButton(driverControlls, XboxPOVButton.SOUTHWEST);
    // povSouthWest_xBox_Driver;

    povWest_xBox_Driver = new XboxPOVButton(driverControlls, XboxPOVButton.WEST);
    // povWest_xBox_Driver;

    povEast_xBox_Driver = new XboxPOVButton(driverControlls, XboxPOVButton.EAST);
    // povEast_xBox_Driver;

  }

  private void codriverButtons() {

    a_xBox_CoDriver = new JoystickButton(coDriverControlls, XboxController.Button.kA.value);
    a_xBox_CoDriver.whenPressed(new IntakeExtend(m_intake), true);

    b_xBox_CoDriver = new JoystickButton(coDriverControlls, XboxController.Button.kB.value);
    b_xBox_CoDriver.whenPressed(new IntakeRetract(m_intake), false);

    x_xBox_CoDriver = new JoystickButton(coDriverControlls, XboxController.Button.kX.value);
    x_xBox_CoDriver.whileHeld(new IntakePowercell(m_intake), true);
    ;

    y_xBox_CoDriver = new JoystickButton(coDriverControlls, XboxController.Button.kY.value);
    y_xBox_CoDriver.whileHeld(new IntakeEjectPowercell(m_intake), true);

    lb_xBox_CoDriver = new JoystickButton(coDriverControlls, XboxController.Button.kBumperLeft.value);
    lb_xBox_CoDriver.whenPressed(new IntakeExtend(m_intake), true);

    rb_xBox_CoDriver = new JoystickButton(coDriverControlls, XboxController.Button.kBumperRight.value);
    rb_xBox_CoDriver.whileHeld(new FlyWheel_Run(m_FlyWheel, coDriverControlls));

    r_Stick_Button_xBox_CoDriver = new JoystickButton(coDriverControlls, XboxController.Button.kStickRight.value);
    // r_Stick_Button_xBox_CoDriver;

    l_Stick_Button_xBox_CoDriver = new JoystickButton(coDriverControlls, XboxController.Button.kStickLeft.value);
    // l_Stick_Button_xBox_CoDriver;

    start_xBox_CoDriver = new JoystickButton(coDriverControlls, XboxController.Button.kStart.value);
    // start_xBox_CoDriver;

    reset_xBox_CoDriver = new JoystickButton(coDriverControlls, XboxController.Button.kBack.value);
    // reset_xBox_CoDriver;

    rt_xBox_CoDriver = new XboxControllerAxisButton(coDriverControlls, XboxController.Axis.kRightTrigger);
    // rt_xBox_CoDriver;

    lt_xBox_CoDriver = new XboxControllerAxisButton(coDriverControlls, XboxController.Axis.kLeftTrigger);
    // lt_xBox_CoDriver;

    povNorth_xBox_CoDriver = new XboxPOVButton(coDriverControlls, XboxPOVButton.NORTH);
    // povNorth_xBox_CoDriver;

    povNorthEast_xBox_CoDriver = new XboxPOVButton(coDriverControlls, XboxPOVButton.NORTHEAST);
    // povNorthEast_xBox_CoDriver;

    povNorthWest_xBox_CoDriver = new XboxPOVButton(coDriverControlls, XboxPOVButton.NORTHWEST);
    // povNorthWest_xBox_CoDriver;

    povSouth_xBox_CoDriver = new XboxPOVButton(coDriverControlls, XboxPOVButton.SOUTH);
    // povSouth_xBox_CoDriver;

    povSouthEast_xBox_CoDriver = new XboxPOVButton(coDriverControlls, XboxPOVButton.SOUTHEAST);
    // povSouthEast_xBox_CoDriver;

    povSouthWest_xBox_CoDriver = new XboxPOVButton(coDriverControlls, XboxPOVButton.SOUTHWEST);
    // povSouthWest_xBox_CoDriver;

    povWest_xBox_CoDriver = new XboxPOVButton(coDriverControlls, XboxPOVButton.WEST);
    // povWest_xBox_CoDriver;

    povEast_xBox_CoDriver = new XboxPOVButton(coDriverControlls, XboxPOVButton.EAST);
    // povEast_xBox_CoDriver;
  }

  private void launchPadButtons() {
  }

  private void SmartDashboardButtons() {

    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
    SmartDashboard.putData("IntakeIntake", new IntakeIntake(m_intake));
    SmartDashboard.putData("IntakeOutake", new IntakeOutake(m_intake));
    SmartDashboard.putData("IntakeExtend", new IntakeExtend(m_intake));
    SmartDashboard.putData("IntakeRetract", new IntakeRetract(m_intake));
    SmartDashboard.putData("TurretTurn", new TurretTurn(m_turretAim));

    SmartDashboard.putData("Xbox Button 4", new IntakePowercell(m_intake));
    SmartDashboard.putData("Xbox Button 3", new IntakeEjectPowercell(m_intake));
    SmartDashboard.putData("Xbox Button 2", new IntakeRetract(m_intake));
    SmartDashboard.putData("Xbox Button 1", new IntakeExtend(m_intake));
  }

  public XboxController getDriverControlls() {
    return driverControlls;
  }

  public XboxController getCoDriverControlls() {
    return coDriverControlls;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return m_chooser.getSelected();
  }

}
