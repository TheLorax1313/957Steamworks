package org.usfirst.frc.team957.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
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
	// RiverTODO: Rename class member variables to start with m_
	final int m_DoNothing = 0;
	final int m_CrosstheLine = 1;
	final int m_TurnRight = 2;
	final int m_TurnLeft = 3;
	final int m_Forward = 4;
	// RiverTODO: Add autoState integer variable to track the state we're in for all auto modes. 
	int m_autoState;
	int autoSelected;
	SendableChooser<Integer> autoChooser = new SendableChooser<>();
	//Joystick Defining
	Joystick Joy1 = new Joystick(1); //flight stick 1
	Joystick Joy2 = new Joystick(2); //flight stick 2
	Joystick controller1 = new Joystick(0); //controller
	Joystick navController = new Joystick(3); //controller
	CANTalon fl = new CANTalon(2);
	CANTalon bl = new CANTalon(3);
	CANTalon fr = new CANTalon(4);
	CANTalon br = new CANTalon(5);
	Spark Climb = new Spark(0);
	// We want to use 1X encoding. We would use 4X if we pull back to the Talons. 
    Encoder m_encoderFL = new Encoder(0, 1, false, Encoder.EncodingType.k1X);
    Encoder m_encoderBL = new Encoder(2, 3, false, Encoder.EncodingType.k1X);
    Encoder m_encoderFR = new Encoder(4, 5, true , Encoder.EncodingType.k1X);	// Right motors are reversed. 
    Encoder m_encoderBR = new Encoder(6, 7, true, Encoder.EncodingType.k1X);	// Right motors are reversed. 
	int speedSwitch;
	RobotDrive m_Drive = new RobotDrive(fl, bl, fr, br);
	int DriveToggle; 
	int LidToggle; 
	double rotation;
	double driveX;
	double driveY;
	int m_startStop = 12;
	int m_ramp = 24;
	boolean m_storedValueTF;
	boolean m_autoTurnRight;
	boolean m_autoTurnLeft;
	boolean m_NeedEncoderReset;
	int autoCase;
	int m_storedAngle;
	int JoyToggle;
	double ContChooseDual;
	double ContChooseSingle;
	double ContChoose360;
	Relay Lights;
	Boolean DriveModeSwitch;
	Boolean LidModeSwitch;
	ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
	Command ControllerCommand;
	int selectedValue;
	SendableChooser<Integer> ControllerChooser;
	// RiverTODO: Add Enum for ControllerChooser values  (see: https://docs.oracle.com/javase/tutorial/java/javaOO/enum.html )
	SendableChooser<Integer> SpeedChooser;
	// RiverTODO: Add Enum for SpeedChooser values  (see: https://docs.oracle.com/javase/tutorial/java/javaOO/enum.html )
	SendableChooser<Integer> gyroReset;
	int speedChooserSel; 
	int GyroBut;
	double speedMultiplier;
	String DriveMode;
	String LidMode;
	DoubleSolenoid LidDouble = new DoubleSolenoid(1, 0, 1);
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		autoChooser = new SendableChooser<Integer>();
		autoChooser.addDefault("Do Nothing", m_DoNothing);
		autoChooser.addObject("Cross the Line", m_CrosstheLine);
		autoChooser.addObject("Turn Right", m_TurnRight);
		autoChooser.addObject("Turn Left", m_TurnLeft);
		autoChooser.addObject("Forward", m_Forward);
		SmartDashboard.putData("Auto choices", autoChooser);
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
        m_gyro.calibrate();
        speedMultiplier = 1;
        // Reset encoders
        m_encoderFL.reset();
        m_encoderBL.reset();
        m_encoderFR.reset();
        m_encoderBR.reset();
        m_storedValueTF=false;
        m_NeedEncoderReset=false;
        // Our encoders are 360 Cycles Per Revolution and 1440 Pulses Per Revolution
        // For the gearbox, we are using an 8.46 to 1 gear ratio. This doesn't matter since we're measuring the output shaft. 
        // Calculation is 2*pi*Radius(our wheels are 6") / Cycles Per Revolution.
        // 
        m_encoderFL.setDistancePerPulse(Math.PI*6/360);
        m_encoderBL.setDistancePerPulse(Math.PI*6/360);
        m_encoderFR.setDistancePerPulse(Math.PI*6/360);
        m_encoderBR.setDistancePerPulse(Math.PI*6/360);
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
		m_NeedEncoderReset=true;
		m_gyro.reset();
		System.out.println("Auto selected: " + autoSelected);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		autoSelected = (int) autoChooser.getSelected();
		switch (autoSelected) {
		case 0:	// RiverTODO: Rename to correct name
			default:
			// Put default auto code here
			
			break;
		case 1:
			// Put custom auto code here
			
			driveForDistance(80,0.5,true);
			break;
		case 2:
			// Put custom auto code here
			m_autoTurnRight=true;
			switch (autoCase){
				case 0:
					if(driveForDistance(50,0.5,true)) 
					autoCase=1;
				case 1:
					if(turnXDegrees(45,0.5)){
					autoCase=2;
					m_NeedEncoderReset=true;
					}
				case 2:
					driveForDistance(30,0.5,false);
				}			
						// 6: Add placeholder comment for Caleb's vision tracking
			break;
		case 3:
			// Put custom auto code here
			m_autoTurnLeft=true;
			switch (autoCase){
				case 0:
					if(driveForDistance(50,0.5,true)) 
					autoCase=1;
				case 1:
					if(turnXDegrees(45,0.5)){
					autoCase=2;
					m_NeedEncoderReset=true;
					}
				case 2:
					driveForDistance(30,0.5,false);
				}			

						// 6: Add placeholder comment for Caleb's vision tracking
			break;
		case 4:
			// Put custom auto code here
			driveForDistance(80,0.5,true);
						// 6: Add placeholder comment for Caleb's vision tracking
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
		speedChooserSel = (int) SpeedChooser.getSelected();		// RiverTODO: rename variable to something like selectedSpeed
		GyroBut = (int) gyroReset.getSelected();
		SmartDashboard.putNumber("Joy Toggle value",JoyToggle);
		SmartDashboard.putNumber("Gyro Reset value",GyroBut);
		Relay.Value light=Relay.Value.kOff;
		int countFL = m_encoderFL.get();
		int countBL = m_encoderBL.get();
		int countFR = m_encoderFR.get();
		int countBR = m_encoderBR.get();
		/**
		SmartDashboard.putNumber("Front Left Encoder count: ",countFL);
		SmartDashboard.putNumber("Back Left Encoder count: ",countBL);
		SmartDashboard.putNumber("Front Right Encoder count: ",countFR);
		SmartDashboard.putNumber("Back Right Encoder count: ",countBR);
**/
		double rateFL = m_encoderFL.getRate();
		double rateBL = m_encoderBL.getRate();
		double rateFR = m_encoderFR.getRate();
		double rateBR = m_encoderBR.getRate();
		/**
		
		SmartDashboard.putNumber("Front Left Encoder rate: ",rateFL);
		SmartDashboard.putNumber("Back Left Encoder rate: ",rateBL);
		SmartDashboard.putNumber("Front Right Encoder rate: ",rateFR);
		SmartDashboard.putNumber("Back Right Encoder rate: ",rateBR);
**/
		double distanceFL = m_encoderFL.getDistance();
		double distanceBL = m_encoderBL.getDistance();
		double distanceFR = m_encoderFR.getDistance();
		double distanceBR = m_encoderBR.getDistance();
		double avgDistance = (distanceFL + distanceBL + distanceFR + distanceBR ) / 4;
		/**
		
		SmartDashboard.putNumber("Front Left Encoder distance (in): ",distanceFL);
		SmartDashboard.putNumber("Back Left Encoder distance (in): ",distanceBL);
		SmartDashboard.putNumber("Front Right Encoder distance (in): ",distanceFR);
		SmartDashboard.putNumber("Back Right Encoder distance (in): ",distanceBR);
		SmartDashboard.putNumber("Average Encoder distance (in): ",avgDistance);
**/
		if(GyroBut==1) {
			m_gyro.reset();
		}
		//Drive Code for each controller type selected by Java Dashboard
		switch(JoyToggle){
		// RiverTODO: Use Enum for ControllerChooser values in JoyToggle cases (see: https://docs.oracle.com/javase/tutorial/java/javaOO/enum.html )
	        case 0://Dual Joystick tank
	        	rotation = (((Joy2.getRawAxis(1))-(Joy1.getRawAxis(1)))/2);
        		driveX = (((Joy1.getRawAxis(0))+(Joy2.getRawAxis(0)))/2);
        		driveY = (((Joy1.getRawAxis(1))+(Joy2.getRawAxis(1)))/2);
                light=(Joy1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
        		DriveModeSwitch = (Joy1.getRawButton(3));
				//LidModeSwitch = false;
				//Climb.set((-(Joy1.getRawAxis(3))+1)/2);
	        	break;
			 case 1://Single Joystick
				rotation = (Joy1.getRawAxis(2));
				driveX = (Joy1.getRawAxis(0));
				driveY = (Joy1.getRawAxis(1));
				//light=(Joy1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;

				DriveModeSwitch = (Joy1.getRawButton(2));
				//LidModeSwitch = false;
				//Climb.set((-(Joy1.getRawAxis(3))+1)/2);
			 	break;
			 case 2://Xbox Controller
		        rotation = (((controller1.getRawAxis(5))-(controller1.getRawAxis(1)))/2);
        		driveX = (((controller1.getRawAxis(0))+(controller1.getRawAxis(4)))/2);
        		driveY = (((controller1.getRawAxis(1))+(controller1.getRawAxis(5)))/2);
        		light=(controller1.getRawButton(1))?Relay.Value.kOn:Relay.Value.kOff;
                //If the controller input is less than our threshold then make it equal to 0
        		DriveModeSwitch = (controller1.getRawButton(7));
        		LidModeSwitch = (navController.getRawButton(2));
        		Climb.set(navController.getRawAxis(3));
		        break;
		       
		}
		Climb.set(navController.getRawAxis(3));
		LidModeSwitch = (navController.getRawButton(2));
 		
		switch(LidToggle){
			case 0://GyroDrive is in use, waiting for button to be pressed
				LidDouble.set(DoubleSolenoid.Value.kForward);
				if(LidModeSwitch)//Waiting for button press
					LidToggle = 1;
				LidMode = "Up";
				break;
			case 1://Drive 2 selected, waiting for release
				LidDouble.set(DoubleSolenoid.Value.kReverse);
				if(!LidModeSwitch)//Waiting for button release
					LidToggle = 2;
				LidMode = "Down";
				break;
			case 2://Drive 2 selected, looking for pressed
				LidDouble.set(DoubleSolenoid.Value.kReverse);
				if(LidModeSwitch)//Waiting for button press
					LidToggle = 3;
				LidMode = "Down";
				break;
			case 3://GyroDrive is in use, looking for release
				LidDouble.set(DoubleSolenoid.Value.kForward);
				if(!LidModeSwitch)//Waiting for button release
					LidToggle = 0;
				LidMode = "Up";
				break;
		}
		SmartDashboard.putString("Lid Status: ",LidMode );
        if(Math.abs(driveX)<0.1) driveX=0;
        if(Math.abs(driveY)<0.1) driveY=0;
        if(Math.abs(rotation)<0.25) {
        	rotation=0;
        } else{
        	if (rotation < 0 )
            	rotation = rotation + 0.25; // Since we're creating a dead zone, make range 0 to .75 not .25 to 1. 
        	else
        		rotation = rotation - 0.25; // Since we're creating a dead zone, make range 0 to .75 not .25 to 1. 
        }
/**
		switch(speedChooserSel){
		// RiverTODO: Use Enum for ControllerChooser values in speedChooserSel cases (see: https://docs.oracle.com/javase/tutorial/java/javaOO/enum.html )
	        case 0:
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
		} **/
        driveX = driveX * speedMultiplier; 
		SmartDashboard.putNumber("Drive X value",driveX);
		SmartDashboard.putNumber("Gyro value",m_gyro.getAngle());
        driveY = driveY * speedMultiplier; 
		SmartDashboard.putNumber("Drive Y value",driveY);
        rotation = rotation * speedMultiplier; 		
        SmartDashboard.putNumber("Drive Rotation value",rotation);


		Lights.set(light);
		//using field orientation using the gyro vs normal drive
		switch(DriveToggle){
			case 0://GyroDrive is in use, waiting for button to be pressed
				m_Drive.mecanumDrive_Cartesian(driveX,driveY,rotation,m_gyro.getAngle());
				if(DriveModeSwitch)//Waiting for button press
					DriveToggle = 1;
				break;
			case 1://Drive 2 selected, waiting for release
				m_Drive.mecanumDrive_Cartesian(.75*driveX,.75*driveY,rotation,0);
				if(!DriveModeSwitch)//Waiting for button release
					DriveToggle = 2;
				DriveMode = "Robot Oriented";
				break;
			case 2://Drive 2 selected, looking for pressed
				m_Drive.mecanumDrive_Cartesian(.75*driveX,.75*driveY,rotation,0);
				if(DriveModeSwitch)//Waiting for button press
					DriveToggle = 3;
				break;
			case 3://GyroDrive is in use, looking for release
				m_Drive.mecanumDrive_Cartesian(driveX,driveY,rotation,m_gyro.getAngle());
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
	public void resetEncoders(){
		m_encoderFL.reset();
        m_encoderBL.reset();
        m_encoderFR.reset();
        m_encoderBR.reset();
	}
	//Encoder count needs to be reset each time it is first called
	public boolean driveForDistance(int Distance,double MaxPower,boolean UseGyro){
		boolean retVal=false;
		double distanceFL = m_encoderFL.getDistance();
		double distanceBL = m_encoderBL.getDistance();
		double distanceFR = m_encoderFR.getDistance();
		double distanceBR = m_encoderBR.getDistance();
		double avgDistance = (distanceFL + distanceBL + distanceFR + distanceBR ) / 4;
		SmartDashboard.putNumber("Average Encoder distance (in): ",avgDistance);
		if (m_NeedEncoderReset){
			resetEncoders();
			m_NeedEncoderReset=false;
		}

		if (avgDistance < m_startStop){
			MaxPower= MaxPower*0.5;
			retVal=false;
		}

		if (avgDistance > m_startStop && avgDistance < m_ramp){
			MaxPower= MaxPower*0.75;
			retVal=false;
		}
		
		if (avgDistance > m_ramp && avgDistance < Distance - 2*m_ramp){
			retVal=false;
		}
		if (avgDistance > Distance - 2*m_ramp && avgDistance < Distance - 2*m_startStop){
			MaxPower= MaxPower*0.5;
			retVal=false;
		}
		if (avgDistance > Distance - 2*m_startStop){
			MaxPower= MaxPower*0.25;
			retVal=false;
		}
		if (avgDistance > Distance){
			MaxPower=0;
			retVal=true;
		}
		if (UseGyro)
			m_Drive.mecanumDrive_Cartesian(0,-MaxPower,0,m_gyro.getAngle());
		else
			m_Drive.mecanumDrive_Cartesian(0,-MaxPower,0,0);
		return retVal;
	}
	public boolean turnXDegrees(int Turn,double TurnPower){
		boolean retVal=false;
		if (m_storedValueTF=false){
			m_storedAngle=(int) m_gyro.getAngle();
		}
		if (m_autoTurnRight=true){
			if (m_gyro.getAngle() < m_storedAngle + Turn){
			}
			if (m_gyro.getAngle() > m_storedAngle + Turn){
			m_storedValueTF=false;
			m_autoTurnRight=false;
			retVal=true;
			}
			}
		if (m_autoTurnLeft=true){
			if (m_gyro.getAngle() > m_storedAngle - Turn){
		}
			if (m_gyro.getAngle() < m_storedAngle - Turn){
			m_storedValueTF=false;
			m_autoTurnLeft=false;
			retVal=true;
		}
		}
		m_Drive.mecanumDrive_Cartesian(0,0,TurnPower,m_gyro.getAngle());
		return retVal;
	}
	// RiverTODO: Add "resetEncoders" method to reset encoder counts to 0 when needed. 
	
	// RiverTODO: Add "driveForDistance" method that accepts a distance and a max power value and returns boolean completed value. 
		// Eventually it would be good to have this method allow for driving backwards too. 
		// Switch statement based on distance traveled
		// If encoder average distance is less than 6 (inches) motors @ 1/4 max power value. return false.
		// distance > 6" & distance < 12" go 1/2 max power value. return false.
		// distance > 12" & < input distance - 12" full max power value.  return false.
		// distance > input distance - 12" & distance < input distance - 6" go 1/2 max power value. return false.
		// distance > input distance - 6" motors @ 1/4 max power value. return false.
		// distance > input distance stop return true.

	// RiverTODO: Add "turnXDegrees" method to accept an angle, boolean direction (ie: turnRight) and max power value and turn to that angle. 
	// need a member variable to track original heading value (double) and a boolean flag to indicate if heading has been saved. 
		// save original heading if it has not been saved and set flag. 
		// Switch statement based on turn direction
		// Right: current heading < original + desired,  rotate right, return false
			// If current heading > original + desired, stop, clear flag and return true
		// Left: current heading > original - desired,  rotate left, return false 
			// If current heading < original - desired, stop, clear flag and return true
		// For now I would ignore ramping because it will be a lot more tricky than driving forward or back.
		// 
	

}

