package org.usfirst.frc.team957.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon;


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
	Joystick controller1 = new Joystick(3); //controller
	
	CANTalon t1 = new CANTalon(1);
	CANTalon t2 = new CANTalon(2);
	CANTalon t3 = new CANTalon(3);
	CANTalon t4 = new CANTalon(4);
	int speedSwitch;
	RobotDrive m_Drive = new RobotDrive(t1, t2, t3, t4);
	
	
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
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
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
		
		boolean but8C = controller1.getRawButton(8); //Start Button on Controller
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
		}
		
			
		
    
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

