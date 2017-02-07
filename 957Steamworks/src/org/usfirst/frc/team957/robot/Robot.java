package org.usfirst.frc.team957.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	//Joystick Defining
	Joystick Joy1 = new Joystick(1); //flight stick 1
	Joystick Joy2 = new Joystick(2); //flight stick 2
	Joystick controller1 = new Joystick(0); //controller
	CANTalon fl = new CANTalon(2);
	CANTalon bl = new CANTalon(3);
	CANTalon fr = new CANTalon(4);
	CANTalon br = new CANTalon(5);
	int speedSwitch;
	RobotDrive m_Drive = new RobotDrive(fl, bl, fr, br);
	int DriveToggle; 
	double rotation;
	double driveX;
	double driveY;
	int JoyToggle;
	double ContChooseDual;
	double ContChooseSingle;
	double ContChoose360;
	Relay Lights;
	Boolean DriveModeSwitch;
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	Command ControllerCommand;
	int selectedValue;
	SendableChooser ControllerChooser;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		speedSwitch = 1;
		DriveToggle = 0;
		JoyToggle = 0;
		ControllerChooser = new SendableChooser();
		ControllerChooser.addDefault("Dual JoySticks",0);
		ControllerChooser.addObject("Single JoySticks",1);
		ControllerChooser.addObject("360 Controller",2);
		SmartDashboard.putData("Controller Chooser",ControllerChooser);
		Lights = new Relay (0);
		Lights.setDirection(Relay.Direction.kForward);
		m_Drive.setInvertedMotor(MotorType.kFrontRight, true);
        m_Drive.setInvertedMotor(MotorType.kRearRight, true);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		//autoSelected = chooser.getSelected();
		 //autoSelected = SmartDashboard.getString("Auto Selector",defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		JoyToggle =  (int) ControllerChooser.getSelected();
		SmartDashboard.putNumber("Joy Toggle value",JoyToggle);
		Relay.Value light=Relay.Value.kOff;

		//Drive Code for each controller type selected by Java Dashboard
		switch(JoyToggle){
	        case 0://Dual Joystick tank
	        	rotation = (((Joy2.getRawAxis(1))-(Joy1.getRawAxis(1)))/2);
        		driveX = (((Joy1.getRawAxis(0))+(Joy2.getRawAxis(0)))/2);
        		driveY = (((Joy1.getRawAxis(1))+(Joy2.getRawAxis(1)))/2);
        		DriveModeSwitch = (Joy1.getRawButton(3));
                light=(Joy1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
	        	break;
			 case 1://Single Joystick
				rotation = (Joy1.getRawAxis(2));
				driveX = (Joy1.getRawAxis(0));
				driveY = (Joy1.getRawAxis(1));
				DriveModeSwitch = (Joy1.getRawButton(3));
				light=(Joy1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
			 	break;
			 case 2://Xbox Controller
		        rotation = (((controller1.getRawAxis(5))-(controller1.getRawAxis(1)))/2);
        		driveX = (((controller1.getRawAxis(0))+(controller1.getRawAxis(4)))/2);
        		driveY = (((controller1.getRawAxis(1))+(controller1.getRawAxis(5)))/2);
        		DriveModeSwitch = (controller1.getRawButton(7));
        		light=(controller1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
                //If the controller input is less than our threshold then make it equal to 0

		        break;
		}
        if(Math.abs(driveX)<0.2) driveX=0;
        if(Math.abs(driveY)<0.2) driveY=0;
        if(Math.abs(rotation)<0.2) rotation=0;

		Lights.set(light);
		//using field orientation using the gyro vs normal drive
		switch(DriveToggle){
			case 0://GyroDrive is in use, waiting for button to be pressed
				m_Drive.mecanumDrive_Cartesian(driveX,driveY,rotation,gyro.getAngle());
				if(DriveModeSwitch)//Waiting for button press
					DriveToggle = 1;
				break;
			case 1://Drive 2 selected, waiting for release
				m_Drive.mecanumDrive_Cartesian(.5*driveX,.5*driveY,rotation,0);
				if(!DriveModeSwitch)//Waiting for button release
					DriveToggle = 2;
				break;
			case 2://Drive 2 selected, looking for pressed
				m_Drive.mecanumDrive_Cartesian(.5*driveX,.5*driveY,rotation,0);
				if(DriveModeSwitch)//Waiting for button press
					DriveToggle = 3;
				break;
			case 3://GyroDrive is in use, looking for release
				m_Drive.mecanumDrive_Cartesian(driveX,driveY,rotation,gyro.getAngle());
				if(!DriveModeSwitch)//Waiting for button release
					DriveToggle = 0;
				break;
		}
		SmartDashboard.putNumber("Drive Toggle value",DriveToggle);

	}	

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

