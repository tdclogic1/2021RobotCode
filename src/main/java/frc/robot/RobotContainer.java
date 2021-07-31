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
import oi.limelightvision.limelight.frc.LimeLight;
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

  // Joysticks
  private final XboxController m_coDriverControlls = new XboxController(1);
  private final XboxController m_driverControlls = new XboxController(0);
  private final Joystick m_launchpad = new Joystick(2);

  // The robot's subsystems
  private final TurretFeed m_turretFeed = new TurretFeed();
  private final TurretAim_MM m_TurretAim_MM = new TurretAim_MM();
  private final FlyWheel_Vel_PID m_FlyWheel_Vel_PID = new FlyWheel_Vel_PID();
  private final Intake m_intake = new Intake();
  private final DriveTrain m_driveTrain = new DriveTrain(() -> m_launchpad.getRawAxis(1));



  //Camera
  private final LimeLight limeLight1 = new LimeLight("limelight");

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_driveTrain.setDefaultCommand(new DriveWithJoystick(m_driveTrain, m_driverControlls));
    m_TurretAim_MM.setDefaultCommand(new Turret_Jog_MotionMagic(m_TurretAim_MM, () -> m_coDriverControlls.getRawAxis(0)));

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

    a_xBox_Driver = new JoystickButton(m_driverControlls, XboxController.Button.kA.value);
    a_xBox_Driver.whenPressed(new IntakeExtend(m_intake), true);

    b_xBox_Driver = new JoystickButton(m_driverControlls, XboxController.Button.kB.value);
    b_xBox_Driver.whenPressed(new IntakeRetract(m_intake), false);

    x_xBox_Driver = new JoystickButton(m_driverControlls, XboxController.Button.kX.value);
    //x_xBox_Driver.whileHeld(new IntakePowercell(m_intake), true);

    y_xBox_Driver = new JoystickButton(m_driverControlls, XboxController.Button.kY.value);
    //y_xBox_Driver

    lb_xBox_Driver = new JoystickButton(m_driverControlls, XboxController.Button.kBumperLeft.value);
    //lb_xBox_Driver.whenPressed(new IntakeExtend(m_intake), true);

    rb_xBox_Driver = new JoystickButton(m_driverControlls, XboxController.Button.kBumperRight.value);
    rb_xBox_Driver.whileHeld(new IntakePowercell(m_intake), true);//whileHeld(new IntakeEjectPowercell(m_intake), true);


    r_Stick_Button_xbox_Driver = new JoystickButton(m_driverControlls, XboxController.Button.kStickRight.value);
    // r_Stick_Button_xbox_Driver;

    l_Stick_Button_xbox_Driver = new JoystickButton(m_driverControlls, XboxController.Button.kStickLeft.value);
    l_Stick_Button_xbox_Driver.whenPressed(new DriveTrain_Toggle_Shifter(m_driveTrain));

    start_xBox_Driver = new JoystickButton(m_driverControlls, XboxController.Button.kStart.value);
    //start_xBox_Driver.whileHeld(new FlyWheel_Velocity(m_FlyWheel_Vel_PID, () -> SmartDashboard.getNumber("Fly Wheel Setpoint", 0)));

    reset_xBox_Driver = new JoystickButton(m_driverControlls, XboxController.Button.kBack.value);
    //reset_xBox_Driver.whileHeld(new Turret_Cycle_PowerCells(m_turretFeed));

    rt_xBox_Driver = new XboxControllerAxisButton(m_driverControlls, XboxController.Axis.kRightTrigger.value);
    rt_xBox_Driver.whileHeld(new IntakeEjectPowercell(m_intake), true);

    lt_xBox_Driver = new XboxControllerAxisButton(m_driverControlls, XboxController.Axis.kLeftTrigger.value);
    // lt_xBox_Driver;

    povNorth_xBox_Driver = new XboxPOVButton(m_driverControlls, XboxPOVButton.NORTH);
    //povNorth_xBox_Driver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> 180 ));

    povNorthEast_xBox_Driver = new XboxPOVButton(m_driverControlls, XboxPOVButton.NORTHEAST);
    //povNorthEast_xBox_Driver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> m_TurretAim_MM.get_MaxPos() ));

    povNorthWest_xBox_Driver = new XboxPOVButton(m_driverControlls, XboxPOVButton.NORTHWEST);
    //povNorthWest_xBox_Driver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> 90+45 ));

    povSouth_xBox_Driver = new XboxPOVButton(m_driverControlls, XboxPOVButton.SOUTH);
    //povSouth_xBox_Driver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM, () -> 0 ));

    povSouthEast_xBox_Driver = new XboxPOVButton(m_driverControlls, XboxPOVButton.SOUTHEAST);
    //povSouthEast_xBox_Driver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> m_TurretAim_MM.get_minPos() ));

    povSouthWest_xBox_Driver = new XboxPOVButton(m_driverControlls, XboxPOVButton.SOUTHWEST);
    //povSouthWest_xBox_Driver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> 45));

    povWest_xBox_Driver = new XboxPOVButton(m_driverControlls, XboxPOVButton.WEST);
    //povWest_xBox_Driver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> 90 ));

    povEast_xBox_Driver = new XboxPOVButton(m_driverControlls, XboxPOVButton.EAST);
    // povEast_xBox_Driver;

  }

  private void codriverButtons() {

    a_xBox_CoDriver = new JoystickButton(m_coDriverControlls, XboxController.Button.kA.value);
    //a_xBox_CoDriver.whileHeld(new Turret_Jog_MotionMagic(m_TurretAim_MM, () -> m_coDriverControlls.getRawAxis(0)));

    b_xBox_CoDriver = new JoystickButton(m_coDriverControlls, XboxController.Button.kB.value);
    //b_xBox_CoDriver.whenPressed(new IntakeRetract(m_intake), false);

    x_xBox_CoDriver = new JoystickButton(m_coDriverControlls, XboxController.Button.kX.value);
    //x_xBox_CoDriver.whileHeld(new FlyWheel_Velocity(m_FlyWheel_Vel_PID, () -> SmartDashboard.getNumber("Fly Wheel Setpoint", 0)));
    x_xBox_CoDriver.whileHeld(new FlyWheel_Velocity(m_FlyWheel_Vel_PID, () -> SmartDashboard.getNumber("Fly Wheel Setpoint", 0), () -> m_launchpad.getRawAxis(0)));

    y_xBox_CoDriver = new JoystickButton(m_coDriverControlls, XboxController.Button.kY.value);
    //y_xBox_CoDriver.whileHeld(new IntakeEjectPowercell(m_intake), true);

    lb_xBox_CoDriver = new JoystickButton(m_coDriverControlls, XboxController.Button.kBumperLeft.value);
    //lb_xBox_CoDriver.whenPressed(new Turret_Jog_MotionMagic(m_TurretAim_MM, () -> m_coDriverControlls.getRawAxis(0)));

    rb_xBox_CoDriver = new JoystickButton(m_coDriverControlls, XboxController.Button.kBumperRight.value);
    //rb_xBox_CoDriver.whenPressed(new Turret_Vision_MotionMagic(m_TurretAim_MM, () -> m_driveTrain.getHeading(), limeLight1));

    r_Stick_Button_xBox_CoDriver = new JoystickButton(m_coDriverControlls, XboxController.Button.kStickRight.value);
    // r_Stick_Button_xBox_CoDriver;

    l_Stick_Button_xBox_CoDriver = new JoystickButton(m_coDriverControlls, XboxController.Button.kStickLeft.value);
    // l_Stick_Button_xBox_CoDriver;

    start_xBox_CoDriver = new JoystickButton(m_coDriverControlls, XboxController.Button.kStart.value);
    //start_xBox_CoDriver.whileHeld(new FlyWheel_Velocity(m_FlyWheel_Vel_PID, () -> SmartDashboard.getNumber("Fly Wheel Setpoint", 0)));

    reset_xBox_CoDriver = new JoystickButton(m_coDriverControlls, XboxController.Button.kBack.value);
    //reset_xBox_CoDriver.whileHeld(new Turret_Cycle_PowerCells(m_turretFeed));

    rt_xBox_CoDriver = new XboxControllerAxisButton(m_coDriverControlls, XboxController.Axis.kRightTrigger.value);
    rt_xBox_CoDriver.whileHeld(new Turret_Cycle_PowerCells(m_turretFeed, true));

    lt_xBox_CoDriver = new XboxControllerAxisButton(m_coDriverControlls, XboxController.Axis.kLeftTrigger.value);
    //lt_xBox_CoDriver.whileHeld(new Turret_Reverse_PowerCells(m_turretFeed));

    povNorth_xBox_CoDriver = new XboxPOVButton(m_coDriverControlls, XboxPOVButton.NORTH);
    povNorth_xBox_CoDriver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> 180 ));

    povNorthEast_xBox_CoDriver = new XboxPOVButton(m_coDriverControlls, XboxPOVButton.NORTHEAST);
    povNorthEast_xBox_CoDriver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> m_TurretAim_MM.get_MaxPos() ));

    povNorthWest_xBox_CoDriver = new XboxPOVButton(m_coDriverControlls, XboxPOVButton.NORTHWEST);
    povNorthWest_xBox_CoDriver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> 90+45 ));

    povSouth_xBox_CoDriver = new XboxPOVButton(m_coDriverControlls, XboxPOVButton.SOUTH);
    povSouth_xBox_CoDriver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM, () -> 0 ));

    povSouthEast_xBox_CoDriver = new XboxPOVButton(m_coDriverControlls, XboxPOVButton.SOUTHEAST);
    povSouthEast_xBox_CoDriver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> m_TurretAim_MM.get_minPos() ));

    povSouthWest_xBox_CoDriver = new XboxPOVButton(m_coDriverControlls, XboxPOVButton.SOUTHWEST);
    povSouthWest_xBox_CoDriver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> 45));

    povWest_xBox_CoDriver = new XboxPOVButton(m_coDriverControlls, XboxPOVButton.WEST);
    povWest_xBox_CoDriver.whileHeld(new Turret_To_Setpoint_MotionMagic(m_TurretAim_MM,() -> 90 ));

    povEast_xBox_CoDriver = new XboxPOVButton(m_coDriverControlls, XboxPOVButton.EAST);
    // povEast_xBox_CoDriver;
  }

  private void launchPadButtons() {
    btn11_launchPad = new JoystickButton(m_launchpad, 11);
    btn11_launchPad.whileHeld(new Turret_Reference(m_TurretAim_MM));

    btn6_launchPad = new JoystickButton(m_launchpad, 6);
    btn6_launchPad.whileHeld(new Turret_Reverse_PowerCells(m_turretFeed, m_FlyWheel_Vel_PID));

    btn10_launchPad = new JoystickButton(m_launchpad, 10);
    btn10_launchPad.whileHeld(new Turret_Vision_MotionMagic(m_TurretAim_MM, () -> m_driveTrain.getHeading(), limeLight1));

  }

  private void SmartDashboardButtons() {

    SmartDashboard.putData(m_turretFeed);
    SmartDashboard.putData(m_TurretAim_MM);
    SmartDashboard.putData(m_FlyWheel_Vel_PID);
    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_driveTrain);


    SmartDashboard.putData("Drive Train Loop Mod Toggle", new DriveTrain_Toggle_Controlmode(m_driveTrain));
    SmartDashboard.putData("Drive Train Mantain Heading Toggle", new DriveTrain_Toggle_MantainHeading(m_driveTrain));


    SmartDashboard.putData("Pipeline 1",new LimeLight_Pipline(limeLight1, 1));

    SmartDashboard.putData("Turret Gyro tracking", new Turret_Gyro_Tracking(m_TurretAim_MM, () -> m_driveTrain.getHeading()));
    SmartDashboard.putData("Turret Gyro Vision tracking", new Turret_Vision_MotionMagic(m_TurretAim_MM, () -> m_driveTrain.getHeading(), limeLight1));

    SmartDashboard.putData("Reset Gyro", new DriveTrain_Reset_Gyro(m_driveTrain));

    SmartDashboard.putNumber("Fly Wheel Setpoint", 5000);
    SmartDashboard.putData("Reset Turt Pos", new Turret_Reset_Pos(m_TurretAim_MM));

    SmartDashboard.putData("Jog Turret", new Turret_Jog_MotionMagic(m_TurretAim_MM, () -> m_driverControlls.getRawAxis(0)));

    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());

  }

  public XboxController getDriverControlls() {
    return m_driverControlls;
  }

  public XboxController getCoDriverControlls() {
    return m_coDriverControlls;
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
