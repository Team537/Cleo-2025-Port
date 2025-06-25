// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OceanViewConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.network.TCPSender;
import frc.robot.network.UDPReceiver;
import frc.robot.commands.XboxParkerManualDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.CLEO.Arm;
import frc.robot.subsystems.CLEO.Intake;
import frc.robot.subsystems.CLEO.Shooter;
import frc.robot.subsystems.upper_assembly.UpperAssemblyBase;
import frc.robot.subsystems.vision.OceanViewManager;
import frc.robot.subsystems.vision.odometry.PhotonVisionCamera;
import frc.robot.subsystems.vision.odometry.VisionOdometry;
import frc.robot.util.autonomous.Alliance;
import frc.robot.util.autonomous.AutonomousRoutine;
import frc.robot.util.swerve.DrivingMotorType;
import frc.robot.util.upper_assembly.UpperAssemblyFactory;
import frc.robot.util.upper_assembly.UpperAssemblyType;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should actually be handled 
 * in the {@link Robot} periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final XboxController driverController = new XboxController(0);

    private final Arm Arm = new Arm();
    private final Intake Intake = new Intake();
    private final Shooter Shooter = new Shooter();


    JoystickButton startButton = new JoystickButton(driverController, Button.kStart.value);
    JoystickButton backButton = new JoystickButton(driverController, Button.kBack.value);
    JoystickButton rightStick = new JoystickButton(driverController, Button.kRightStick.value);
    JoystickButton leftStick = new JoystickButton(driverController, Button.kLeftStick.value);
    JoystickButton rightBumper = new JoystickButton(driverController, Button.kRightBumper.value);
    JoystickButton leftBumper = new JoystickButton(driverController, Button.kLeftBumper.value);
    JoystickButton aButton = new JoystickButton(driverController, Button.kA.value);
    JoystickButton bButton = new JoystickButton(driverController, Button.kB.value);
    JoystickButton yButton = new JoystickButton(driverController, Button.kY.value);
    JoystickButton xButton = new JoystickButton(driverController, Button.kX.value);
    POVButton dPadUpButton = new POVButton(driverController, 0);
    POVButton dPadDownButton = new POVButton(driverController, 180);
    POVButton dPadRightButton = new POVButton(driverController, 90);
    POVButton dPadLeftButton = new POVButton(driverController, 270);

 

    TriggerButton leftTrigger = new TriggerButton(driverController);


    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final XboxController xBoxController = new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    // Networking
    private UDPReceiver udpReceiver;
    private TCPSender tcpSender;

    // Subsystems
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private DriveSubsystem driveSubsystem = new DriveSubsystem();
    private UpperAssemblyBase upperAssembly = UpperAssemblyFactory.createUpperAssembly(Constants.Defaults.DEFAULT_UPPER_ASSEMBLY);
    
    private VisionOdometry visionOdometry = new VisionOdometry(driveSubsystem.getSwerveDrivePoseEstimator()); // TODO: Add logic to add cameras to adjust odometry. visionOdometry.addCamera(PhotonVisionCamera camera);
    
    @SuppressWarnings("unused") // The class is used due to how WPILib treats and stores subsystems.
    private OceanViewManager oceanViewManager;

    // Commands
    Command manualDriveCommand = new XboxParkerManualDriveCommand(driveSubsystem, xBoxController);

    // Smart Dashboard Inputs
    private final SendableChooser<AutonomousRoutine> autonomousSelector = new SendableChooser<>();
    private final SendableChooser<Alliance> allianceSelector = new SendableChooser<>();

    private final SendableChooser<UpperAssemblyType> upperAssemblySelector = new SendableChooser<>();
    private final SendableChooser<DrivingMotorType> drivingMotorSelector = new SendableChooser<>();

    /**
     * Creates a new RobotContainer object and sets up SmartDashboard an the button inputs.
          */
    public RobotContainer() {

            //triggers -----------------------------------------------
        leftTrigger.onTrue(new ParallelCommandGroup(new StartEndCommand(Shooter::ShooterAmp, Shooter::ShooterAmp, Shooter),
        new StartEndCommand(Intake::IntakeAmp, Intake::IntakeAmp, Intake)));

        leftTrigger.onFalse(new ParallelCommandGroup(new StartEndCommand(Shooter::ShooterStop, Shooter::ShooterStop, Shooter),
        new StartEndCommand(Intake::IntakeStop, Intake::IntakeStop, Intake)));
        
        //Bumpers ------------------------------------------------

        leftBumper.onTrue(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterForward, Shooter::ShooterForward,Shooter), 
            new StartEndCommand(Intake::IntakeStop, Intake::IntakeMax, Intake).withTimeout(1)));

        leftBumper.onFalse(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterForward, Shooter::ShooterStop,Shooter).withTimeout(0.25), 
            new StartEndCommand(Intake::IntakeStop, Intake::IntakeStop, Intake)));
        

        rightBumper.toggleOnTrue(new ParallelCommandGroup(new StartEndCommand(Intake::IntakeForward, Intake::IntakePIDOff, Intake).until(()-> Intake.GetSwitchHit()),
            new StartEndCommand(Arm::ArmIntake, Arm::ArmSubwoofer, Arm).until(()-> Intake.GetSwitchHit())));

        //   rightBumper.onFalse(new StartEndCommand(Intake::IntakeOff, Intake::IntakeOff, Intake));

        //ABXY ---------------------------------------------------------

        aButton.onTrue(new StartEndCommand(Arm::ArmIntake, Arm::ArmIntake, Arm));

        // aButton.onFalse(null


        bButton.onTrue(new StartEndCommand(Arm::ArmSubwoofer, Arm::ArmSubwoofer, Arm));

        // bButton.onFalse(null);


        xButton.onTrue(new StartEndCommand(Arm::ArmMid, Arm::ArmMid, Arm));

        // xButton.onFalse(null);


        yButton.onTrue(new StartEndCommand(Arm::ArmAmp, Arm::ArmAmp, Arm));

        // yButton.onFalse(null);


        //D-PAD ---------------------------------------------

        // dPadUpButton.onTrue(null);

        // dPadUpButton.onFalse(null);


        // dPadDownButton.onTrue(new StartEndCommand(Arm::ArmSmartSet, Arm::ArmSmartSet, Arm).withTimeout(0));

        // dPadDownButton.onFalse(null);


        dPadLeftButton.onTrue(new StartEndCommand(Arm::ArmManualUp, Arm::ArmManualUp, Arm));

        dPadLeftButton.onFalse(new StartEndCommand(Arm::ArmManualStop, Arm::ArmManualStop, Arm));


        dPadRightButton.onTrue(new StartEndCommand(Arm::ArmManualDown, Arm::ArmManualDown, Arm));

        dPadRightButton.onFalse(new StartEndCommand(Arm::ArmManualStop, Arm::ArmManualStop, Arm));


        //Start and Back --------------------------------------------

        // Reset the IMU when the start button is pressed.
        startButton.onTrue(new InstantCommand(driveSubsystem::zeroHeading));
        
        //startButton.onFalse(null);


        backButton.onTrue(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterReverse, Shooter::ShooterStop,Shooter), 
            new StartEndCommand(Intake::IntakeReverse, Intake::IntakeStop, Intake)));

        backButton.onFalse(new ParallelCommandGroup( new StartEndCommand(Shooter::ShooterStop, Shooter::ShooterStop,Shooter), 
            new StartEndCommand(Intake::IntakeStop, Intake::IntakeStop, Intake)));
            
        // Setup OceanView & all of its networking dependencies.
        // setupOceanViewManager();

        // Add cameras to the VisionOdometry object.
        // visionOdometry.addCamera(new PhotonVisionCamera(VisionConstants.FRONT_CAMERA_NAME, new Transform3d()));
        // visionOdometry.addCamera(new PhotonVisionCamera(VisionConstants.SLIDE_CAMERA_NAME, new Transform3d()));

        // Setup Dashboard
        setupSmartDashboard();

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Sets up the OceanViewManager instance used by the robot. 
     * If the PI is not connected to the robot, nothing will happen.
     */
    private void setupOceanViewManager() {

        // Attempted to create a new TCPSender and UDPReceiver object.
        try {
            this.udpReceiver = new UDPReceiver(OceanViewConstants.UDP_PORT_NUMBER);    
            this.tcpSender = new TCPSender(OceanViewConstants.PI_IP, OceanViewConstants.TCP_PORT_NUMBER);
            System.out.println("Successfully created TCPSender and UDPReceiver object!");
        } catch (Exception e) {
            System.err.println("Failed to construct TCPSender object: " + e.getMessage());
        }

        // An OceanView manager instance cannot be created if either the TCPSender or UDPReceiver is null.
        // Thus, we stop setting up the OceanViewManager.
        if (this.udpReceiver == null || this.tcpSender == null) {
            return;
        }

        // Start the UDPReceiver.
        this.udpReceiver.start();

        // Create a new OceanViewManager object.
        this.oceanViewManager = new OceanViewManager(this.udpReceiver, this.tcpSender, driveSubsystem::getRobotPose);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s 
     * subclasses for {@link CommandXboxControllerXbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4ControllerPS4} 
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(exampleSubsystem::exampleCondition)
        //         .onTrue(new ExampleCommand(exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed, cancelling on release.
        // driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
    }

    /**
     * This method sets up the dashboard so that the drivers can configure the robots settings.
     */
    private void setupSmartDashboard() {
            
        // Setup Autonomous Routine Selection
        autonomousSelector.setDefaultOption("LEFT_HIGH_SCORE", AutonomousRoutine.LEFT_HIGH_SCORE);
        for (AutonomousRoutine autonomousRoutine : AutonomousRoutine.values()) {
            autonomousSelector.addOption(autonomousRoutine.toString(), autonomousRoutine);
        }

        // Setup Alliance Selection
        allianceSelector.setDefaultOption("RED", Alliance.RED);
        for (Alliance alliance : Alliance.values()) {
            allianceSelector.addOption(alliance.toString(), alliance);
        }

        // Add the selectors to the dashboard.
        SmartDashboard.putData(autonomousSelector);
        SmartDashboard.putData(allianceSelector);
        SmartDashboard.putData(upperAssemblySelector);
        SmartDashboard.putData(drivingMotorSelector);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     */
    public Command getAutonomousCommand() {

        // Get and display the currently selected autonomous routine.
        AutonomousRoutine selectedAutonomousRoutine = autonomousSelector.getSelected();
        Alliance selectedAlliance = allianceSelector.getSelected();
        SmartDashboard.putString("Selected Autonomous", selectedAutonomousRoutine.toString());
        SmartDashboard.putString("Selected Alliance", selectedAlliance.toString());

        // An example command will be run in autonomous
        return Autos.exampleAuto(exampleSubsystem);
    }

    /**
     * sets the upper assembly to the given type
     * 
     * @param upperAssemblyType the type of upper assembly to set to
     */
    public void setUpperAssembly(UpperAssemblyType upperAssemblyType) {
        upperAssembly = UpperAssemblyFactory.createUpperAssembly(upperAssemblyType);
    }

    /**
     * Schedules commands used exclusively during TeleOp.
     */
    public void scheduleTeleOp() {
        // The Drive Command
        driveSubsystem.setDefaultCommand(manualDriveCommand);
        // upperAssembly.setDefaultCommand(upperAssembly.getManualCommand(xBoxController));
    }
}