package org.usfirst.frc.team957.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
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
	//ADXRS450_Gyro gyro = new ADXRS450_Gyro();
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
		Relay.Value light;
		
		
		
		/**boolean but8C = controller1.getRawButton(8); //Start Button on Controller
		boolean but6F = Joy1.getRawButton(8); //Button 6 on Flight Stick 1
		
		double speed;
		
		switch(speedSwitch){
		case 1: speed = 1;
		if (but8C == true || but6F == true){
			speedSwitch = 2;
		}	 
		break;
		case 2: 
		if (but8C == false || but6F == false){
			speedSwitch = 3;
		} 
		break;
		case 3: speed = 0.25;
		if (but8C == true || but6F == true){
			speedSwitch = 4;
		} 
		break;
		case 4:
		if (but8C == false || but6F == false){
			speedSwitch = 1;
		} 
		SmartDashboard.putNumber("SpeedSwitch", speedSwitch);
		*/
		//Drive Code
		 switch(JoyToggle){
	        case 0://Dual Joystick tank, waiting for dash input to switch
	        	rotation = (((Joy1.getRawAxis(1))-(Joy2.getRawAxis(1)))/2);
	        		driveX = (((Joy1.getRawAxis(0))+(Joy2.getRawAxis(0)))/2);
	        		driveY = (((Joy1.getRawAxis(1))+(Joy2.getRawAxis(1)))/2);
	        		DriveModeSwitch = (Joy1.getRawButton(3));
	                light=(Joy1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
	                Lights.set(light);
	        	break;
			 case 1://Single Joystick, waiting for dash input to switch
				 rotation = (Joy1.getRawAxis(2));
					 driveX = (Joy1.getRawAxis(0));
					 driveY = (Joy1.getRawAxis(1));
					 DriveModeSwitch = (Joy1.getRawButton(3));
		                light=(Joy1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
		                Lights.set(light);
			 	break;
			 case 2://Xbox Controller, waiting for dash input to switch
		         rotation = (((controller1.getRawAxis(1))-(controller1.getRawAxis(5)))/2);
		        		driveX = (((controller1.getRawAxis(0))+(controller1.getRawAxis(4)))/2);
		        		driveY = (((controller1.getRawAxis(1))+(controller1.getRawAxis(5)))/2);
		        		DriveModeSwitch = (controller1.getRawButton(7));
		        		light=(controller1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
		                Lights.set(light);
		        break;
	}
		 switch(DriveToggle){
	        case 0://GyroDrive is in use, waiting for button to be pressed
	        	m_Drive.mecanumDrive_Cartesian(driveX,driveY,rotation,0);//Switch Drive modes
	        	if(DriveModeSwitch){
	        		DriveToggle = 1;
	        	}
	        	break;
			 case 1://Drive 2 selected, waiting for release
				 if(!DriveModeSwitch)
					 DriveToggle = 2;
				 break;
			 case 2://Drive 2 selected, looking for pressed
				 m_Drive.mecanumDrive_Cartesian(driveX,driveY,rotation,0);
				 if(DriveModeSwitch){
					 DriveToggle = 3;
				 }
			 	break;
			 case 3://GyroDrive is in use, looking for release
				 if(!DriveModeSwitch)
					 DriveToggle = 0;
			 	break;}
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

