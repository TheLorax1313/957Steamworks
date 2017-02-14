package org.usfirst.frc.team957.robot;

import edu.wpi.first.wpilibj.Encoder;
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
import edu.wpi.first.wpilibj.DoubleSolenoid;

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
	// We want to use 1X encoding. We would use 4X if we pull back to the Talons. 
    Encoder m_encoderFL = new Encoder(0, 1, false, Encoder.EncodingType.k1X);
    Encoder m_encoderBL = new Encoder(2, 3, false, Encoder.EncodingType.k1X);
    Encoder m_encoderFR = new Encoder(4, 5, true, Encoder.EncodingType.k1X);	// Right motors are reversed. 
    Encoder m_encoderBR = new Encoder(6, 7, true, Encoder.EncodingType.k1X);	// Right motors are reversed. 
	int speedSwitch;
	RobotDrive m_Drive = new RobotDrive(fl, bl, fr, br);
	int DriveToggle; 
	int LidToggle; 
	double rotation;
	double driveX;
	double driveY;
	int JoyToggle;
	double ContChooseDual;
	double ContChooseSingle;
	double ContChoose360;
	Relay Lights;
	Boolean DriveModeSwitch;
	Boolean LidModeSwitch;
	ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	Command ControllerCommand;
	int selectedValue;
	SendableChooser<Integer> ControllerChooser;
	SendableChooser<Integer> SpeedChooser;
	SendableChooser<Integer> gyroReset;
	int speedChooserSel; 
	int GyroBut;
	double speedMultiplier;
	String DriveMode;
	String LidMode;
	DoubleSolenoid LidDouble = new DoubleSolenoid(0, 1);
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
		LidToggle = 0;
		DriveMode = "Field Oriented";
		LidMode = "Down";
		JoyToggle = 0;
		ControllerChooser = new SendableChooser<Integer>();
		ControllerChooser.addDefault("Dual JoySticks",0);
		ControllerChooser.addObject("Single JoySticks",1);
		ControllerChooser.addObject("360 Controller",2);		
		SmartDashboard.putData("Controller Chooser",ControllerChooser);

		SpeedChooser = new SendableChooser<Integer>();
		SpeedChooser.addDefault("Full Speed",0);
		SpeedChooser.addObject("Half Speed",1);
		SpeedChooser.addObject("Quarter Speed",2);
		SmartDashboard.putData("Speed Chooser",SpeedChooser);
		gyroReset = new SendableChooser<Integer>();
		gyroReset.addDefault("waiting",0);
		gyroReset.addObject("Reset",1);
		SmartDashboard.putData("Gyro Reset",gyroReset);
		Lights = new Relay (0);
		Lights.setDirection(Relay.Direction.kForward);
		m_Drive.setInvertedMotor(MotorType.kFrontRight, true);
        m_Drive.setInvertedMotor(MotorType.kRearRight, true);
        gyro.calibrate();
        speedMultiplier = 1;
        // Reset encoders
        m_encoderFL.reset();
        m_encoderBL.reset();
        m_encoderFR.reset();
        m_encoderBR.reset();
        // Our encoders are 360 Cycles Per Revolution and 1440 Pulses Per Revolution
        // For the gearbox, we are using an 8.46 to 1 gear ratio. This doesn't matter since we're measuring the output shaft. 
        // Calculation is 2*pi*Radius(our wheels are 6") / Cycles Per Revolution.
        // 
        m_encoderFL.setDistancePerPulse(Math.PI*6/1440);
        m_encoderBL.setDistancePerPulse(Math.PI*6/1440);
        m_encoderFR.setDistancePerPulse(Math.PI*6/1440);
        m_encoderBR.setDistancePerPulse(Math.PI*6/1440);
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
		gyro.reset();
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

	@Override
	public void teleopInit() {
		//System.out.println("Auto selected: " + autoSelected);
	}
	
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		JoyToggle = (int) ControllerChooser.getSelected();
		speedChooserSel = (int) SpeedChooser.getSelected();
		GyroBut = (int) gyroReset.getSelected();
		SmartDashboard.putNumber("Joy Toggle value",JoyToggle);
		Relay.Value light=Relay.Value.kOff;
		int countFL = m_encoderFL.get();
		int countBL = m_encoderBL.get();
		int countFR = m_encoderFR.get();
		int countBR = m_encoderBR.get();
		
		SmartDashboard.putNumber("Front Left Encoder count: ",countFL);
		SmartDashboard.putNumber("Back Left Encoder count: ",countBL);
		SmartDashboard.putNumber("Front Right Encoder count: ",countFR);
		SmartDashboard.putNumber("Back Right Encoder count: ",countBR);

		double rateFL = m_encoderFL.getRate();
		double rateBL = m_encoderBL.getRate();
		double rateFR = m_encoderFR.getRate();
		double rateBR = m_encoderBR.getRate();
		
		SmartDashboard.putNumber("Front Left Encoder rate: ",rateFL);
		SmartDashboard.putNumber("Back Left Encoder rate: ",rateBL);
		SmartDashboard.putNumber("Front Right Encoder rate: ",rateFR);
		SmartDashboard.putNumber("Back Right Encoder rate: ",rateBR);

		double distanceFL = m_encoderFL.getDistance();
		double distanceBL = m_encoderBL.getDistance();
		double distanceFR = m_encoderFR.getDistance();
		double distanceBR = m_encoderBR.getDistance();
		
		SmartDashboard.putNumber("Front Left Encoder distance: ",distanceFL);
		SmartDashboard.putNumber("Back Left Encoder distance: ",distanceBL);
		SmartDashboard.putNumber("Front Right Encoder distance: ",distanceFR);
		SmartDashboard.putNumber("Back Right Encoder distance: ",distanceBR);

		if(GyroBut==1) gyro.reset();
		//Drive Code for each controller type selected by Java Dashboard
		switch(JoyToggle){
	        case 0://Dual Joystick tank
	        	rotation = (((Joy2.getRawAxis(1))-(Joy1.getRawAxis(1)))/2);
        		driveX = (((Joy1.getRawAxis(0))+(Joy2.getRawAxis(0)))/2);
        		driveY = (((Joy1.getRawAxis(1))+(Joy2.getRawAxis(1)))/2);
        		DriveModeSwitch = (Joy1.getRawButton(3));
                light=(Joy1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
				LidModeSwitch = false;
	        	break;
			 case 1://Single Joystick
				rotation = (Joy1.getRawAxis(2));
				driveX = (Joy1.getRawAxis(0));
				driveY = (Joy1.getRawAxis(1));
				DriveModeSwitch = (Joy1.getRawButton(3));
				light=(Joy1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
				LidModeSwitch = false;
			 	break;
			 case 2://Xbox Controller
		        rotation = (((controller1.getRawAxis(5))-(controller1.getRawAxis(1)))/2);
        		driveX = (((controller1.getRawAxis(0))+(controller1.getRawAxis(4)))/2);
        		driveY = (((controller1.getRawAxis(1))+(controller1.getRawAxis(5)))/2);
        		DriveModeSwitch = (controller1.getRawButton(7));
        		light=(controller1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
                //If the controller input is less than our threshold then make it equal to 0
        		LidModeSwitch = (controller1.getRawButton(2));
		        break;
		       
		}
		
		switch(LidToggle){
			case 0://GyroDrive is in use, waiting for button to be pressed
				LidDouble.set(DoubleSolenoid.Value.kForward);
				if(LidModeSwitch)//Waiting for button press
					LidToggle = 1;
				break;
			case 1://Drive 2 selected, waiting for release
				LidDouble.set(DoubleSolenoid.Value.kReverse);
				if(!LidModeSwitch)//Waiting for button release
					LidToggle = 2;
				LidMode = "Up";
				break;
			case 2://Drive 2 selected, looking for pressed
				LidDouble.set(DoubleSolenoid.Value.kReverse);
				if(LidModeSwitch)//Waiting for button press
					LidToggle = 3;
				break;
			case 3://GyroDrive is in use, looking for release
				LidDouble.set(DoubleSolenoid.Value.kForward);
				if(!LidModeSwitch)//Waiting for button release
					LidToggle = 0;
				LidMode = "Down";
				break;
		}
		SmartDashboard.putString("Lid Status: ",LidMode );
    
        if(Math.abs(driveX)<0.1) driveX=0;
        if(Math.abs(driveY)<0.1) driveY=0;
        if(Math.abs(rotation)<0.15) rotation=0;
		switch(speedChooserSel){
	        case 0://Dual Joystick tank
	        	speedMultiplier = 1;
	        	break;
	        case 1: 
	        	speedMultiplier = 0.5;
	        	break;
	        case 2: 
	        	speedMultiplier = 0.25;
	        	break;
			default:
	        	speedMultiplier = 1;
		}
        driveX = driveX * speedMultiplier; 
		SmartDashboard.putNumber("Drive X value",driveX);
        driveY = driveY * speedMultiplier; 
		SmartDashboard.putNumber("Drive Y value",driveY);
        rotation = rotation * speedMultiplier; 		
        SmartDashboard.putNumber("Drive Rotation value",rotation);


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
				DriveMode = "Robot Oriented";
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
				DriveMode = "Field Oriented";
				break;
		}
		SmartDashboard.putString("Drive mode",DriveMode );
	}	

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
}

