package org.usfirst.frc.team957.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import org.opencv.core.Mat;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
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
	
	final int m_DoNothing = 0;
	final int m_CrosstheLine = 1;
	final int m_TurnRight = 2;
	final int m_TurnLeft = 3;
	final int m_Forward = 4;
	int m_autoState;
	int m_autoSelected;
	SendableChooser<Integer> m_AutoChooser;
	SendableChooser<Boolean> m_GyroResetChooser;
	//Joystick Defining
	Joystick m_Joy1 = new Joystick(1); //flight stick 1
	Joystick m_NavController = new Joystick(3); //controller
	CANTalon m_fl = new CANTalon(2);
	CANTalon m_bl = new CANTalon(3);
	CANTalon m_fr = new CANTalon(4);
	CANTalon m_br = new CANTalon(5);
	Spark m_Climb = new Spark(0);
	// We want to use 1X encoding. We would use 4X if we pull back to the Talons. 
    Encoder m_encoderFL = new Encoder(0, 1, false, Encoder.EncodingType.k1X);
    Encoder m_encoderBL = new Encoder(2, 3, false, Encoder.EncodingType.k1X);
    Encoder m_encoderFR = new Encoder(4, 5, true , Encoder.EncodingType.k1X);	// Right motors are reversed. 
    Encoder m_encoderBR = new Encoder(6, 7, true, Encoder.EncodingType.k1X);	// Right motors are reversed. 
	int m_speedSwitch;
	RobotDrive m_Drive = new RobotDrive(m_fl, m_bl, m_fr, m_br);
	int m_DriveToggle; 
	int m_LidToggle; 
	double m_rotation;
	double m_driveX;
	double m_driveY;
	double m_POVFinal;
	int m_startStop = 6;
	int m_ramp = 12;
	boolean m_storedValueTF;
	boolean m_autoTurnRight;
	boolean m_NeedEncoderReset;
	int m_autoCase=0;
	int m_storedAngle;
	int m_JoyToggle;
	double m_ContChooseDual;
	double m_ContChooseSingle;
	double m_ContChoose360;
	Relay m_LightsRelay;
	Boolean m_DriveModeSwitch;
	Boolean m_LidModeSwitch;
	ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
	Command m_ControllerCommand;
	int m_selectedValue;
	Boolean m_ResetGyro;
	double m_speedMultiplier;
	String m_DriveMode;
	String m_LidMode;
	String m_TeleAuto = "Driver";
	DoubleSolenoid m_LidSolenoid = new DoubleSolenoid(1, 0, 1);
	Boolean m_ShowAllData = false; 
	AutonomusFinder Auto = new AutonomusFinder();
	int m_autoTarget = 0;
	double m_CameraSwitch; 
	double m_distance;
	double m_prevDistance;
	double m_distanceLeft;
	stream streamCamera = new stream();
	ledCommunication LED = new ledCommunication();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_AutoChooser = new SendableChooser<Integer>();
		m_AutoChooser.addDefault("Do Nothing", m_DoNothing);
		m_AutoChooser.addObject("Cross the Line", m_CrosstheLine);
		m_AutoChooser.addObject("Turn Right", m_TurnRight);
		m_AutoChooser.addObject("Turn Left", m_TurnLeft);
		m_AutoChooser.addObject("Forward", m_Forward);
		SmartDashboard.putData("autoChoices", m_AutoChooser);

		m_GyroResetChooser = new SendableChooser<Boolean>();
		m_GyroResetChooser.addDefault("waiting",false);
		m_GyroResetChooser.addObject("Reset",true);
		SmartDashboard.putData("Gyro Reset",m_GyroResetChooser);
		m_speedSwitch = 1;
		m_DriveToggle = 0;
		m_LidToggle = 0;
		m_LidMode = "Up";
		m_DriveMode = "Field Oriented";
		m_JoyToggle = 2;
		m_LightsRelay = new Relay(0);
		m_LightsRelay.setDirection(Relay.Direction.kForward);
		m_Drive.setInvertedMotor(MotorType.kFrontRight, true);
        m_Drive.setInvertedMotor(MotorType.kRearRight, true);
        m_Drive.setSafetyEnabled(false);
        m_gyro.calibrate();
        m_speedMultiplier = 1; // Speed multiplier used to control maximum output values. 1 = full power.
        m_storedValueTF=false; // Used by turnXDegrees to indicate if the original gyro heading needs to be stored.
        m_NeedEncoderReset=false;
        // Our encoders are 360 Cycles Per Revolution and 1440 Pulses Per Revolution
        // For the gearbox, we are using an 8.46 to 1 gear ratio. This doesn't matter since we're measuring the output shaft. 
        // Calculation is 2*pi*Radius(our wheels are 6") / Cycles Per Revolution.
        m_encoderFL.setDistancePerPulse(Math.PI*6/360);
        m_encoderBL.setDistancePerPulse(Math.PI*6/360);
        m_encoderFR.setDistancePerPulse(Math.PI*6/360);
        m_encoderBR.setDistancePerPulse(Math.PI*6/360);
        resetEncoders();
        m_distance = 80;
        m_prevDistance = 80;
		Auto.socketSetup(5800);
		LED.socketSetup(5801);
		streamCamera.start();

	}

	/**
	 * This function is called to initialize the bot while the it is disabled.
	 */
	@Override
	public void disabledInit() {
		
	}

	/**
	 * This function is called periodically while the bot is disabled.
	 */
	@Override
	public void disabledPeriodic() {
		//m_ResetGyro = SmartDashboard.getBoolean("gyroReset", false);
		m_ResetGyro = m_GyroResetChooser.getSelected();
		if (m_ResetGyro){
			m_gyro.reset();
			//SmartDashboard.putBoolean("gyroReset", false);
		}
		m_autoSelected = m_AutoChooser.getSelected();
		showInstrumentation();
		Auto.AutoDetect();
		LED.updateAlliance();
		LED.updateClimbingBoolean(false);
		
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
		m_autoCase=0;
		m_storedValueTF=false;
		m_NeedEncoderReset=true;
		m_gyro.reset();
		resetEncoders();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		LED.updateAlliance();
		LED.updateClimbingBoolean(false);
		m_LidSolenoid.set(DoubleSolenoid.Value.kForward);  // open the lid
		m_autoSelected = m_AutoChooser.getSelected();
		m_LightsRelay.set(Relay.Value.kForward);
		double distanceFL = m_encoderFL.getDistance();
		double distanceBL = m_encoderBL.getDistance();
		double distanceFR = m_encoderFR.getDistance();
		double distanceBR = m_encoderBR.getDistance();
		double avgDistance = (distanceFL + distanceBL + distanceFR + distanceBR ) / 4;
		Auto.AutoDetect();
		double XFinal = Auto.acceptedXFinal();
		double YFinal = Auto.acceptedYFinal();
		
		switch (m_autoSelected) {
		case 0://Do Nothing 
			
			default:
			break;
		case 1://Cross the line
			driveForDistance(80,0.35,true);
			break;
		case 2://Turn Right
			m_autoTurnRight=true;
			switch (m_autoCase){
				case 0:
					if(driveForDistance(80,0.5,true)) 
						m_autoCase=1;
					break;
				case 1:
					if(turnToHeading(45,0.3)){
						m_NeedEncoderReset=true;
						m_autoCase=2;
					}
					break;
				case 2:
					m_Drive.mecanumDrive_Cartesian(0,0,0,m_gyro.getAngle());
					resetEncoders();
					m_autoCase = 3;
					break;
				case 3:
				if(avgDistance <= 50){
						AutoDrive(0.233,XFinal);
					}else{
						AutoDrive(0,0);
					} 
				break;
				}			
			break;
		case 3://Turn Left
			m_autoTurnRight=false;
			switch (m_autoCase){
				case 0:
					if(driveForDistance(80,0.5,true)) 
						m_autoCase=1;
					break;
				case 1:
					if(turnToHeading(-45,0.3)){
						m_NeedEncoderReset=true;
						m_autoCase=2;
					}
					break;
				case 2:
					m_Drive.mecanumDrive_Cartesian(0,0,0,m_gyro.getAngle());
					resetEncoders();
					m_autoCase = 3;
					break;
				case 3:
					if(avgDistance <= 50){
						AutoDrive(0.233,XFinal);
					}else{
						AutoDrive(0,0);
					}
				break;
				}			
			break;
		case 4://Drive forward
			if((avgDistance <= 85)){
				AutoDrive(0.233,XFinal);
			}else{
				AutoDrive(0,0);
			}
			break;
		}
		showInstrumentation();
	}

	@Override
	public void teleopInit() {
	}
	
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		LED.updateAlliance();
		LED.updateClimbingBoolean(m_NavController.getRawAxis(3) > 0.5);
		Auto.AutoDetect();
		boolean autoButton = false;
		boolean AutoAimEnabled = false;
		m_ResetGyro = m_GyroResetChooser.getSelected();
		//m_ResetGyro = SmartDashboard.getBoolean("gyroReset", false);
		m_LightsRelay.set(Relay.Value.kForward);
		if (m_ResetGyro){
			m_gyro.reset();
			//SmartDashboard.putBoolean("gyroReset", false);
		}
		
		//Drive Code for each controller type selected by Java Dashboard
				m_rotation = (m_Joy1.getRawAxis(2));
				m_driveX = (m_Joy1.getRawAxis(0));
				m_driveY = (m_Joy1.getRawAxis(1));
				m_DriveModeSwitch = (m_Joy1.getRawButton(2));
				m_POVFinal = m_Joy1.getPOV();
				autoButton = (m_Joy1.getRawButton(3));
		
		switch(m_autoTarget){
		case 0:
			AutoAimEnabled=false;
			m_TeleAuto = "Driver";
			if(autoButton)//Waiting for button press
				m_autoTarget = 1;
			break;
		case 1:
			AutoAimEnabled=true;
			m_TeleAuto = "Auto Aim";
			if(!autoButton)//Waiting for button release
				m_autoTarget = 2;
			break;
		case 2:
			AutoAimEnabled=true;
			m_TeleAuto = "Auto Aim";
			if(autoButton)//Waiting for button press
				m_autoTarget = 3;
			break;
		case 3:
			AutoAimEnabled=false;
			m_TeleAuto = "Driver";
			if(!autoButton)//Waiting for button release
				m_autoTarget = 0;
			break;
			
		}
		
        if(m_POVFinal == 45 || m_POVFinal == 225){
        	m_POVFinal = m_POVFinal + 45;
        
        }
        if(m_POVFinal == 135 || m_POVFinal == 315){
        	m_POVFinal = m_POVFinal - 45;
        }
		m_Climb.set(m_NavController.getRawAxis(3));
		m_LidModeSwitch = (m_NavController.getRawButton(2));
 		
		switch(m_LidToggle){
			case 0://Lid is up, waiting for button to be pressed
				m_LidSolenoid.set(DoubleSolenoid.Value.kForward);
				if(m_LidModeSwitch)//Waiting for button press
					m_LidToggle = 1;
				m_LidMode = "Up";
				break;
			case 1://Lid is down, waiting for release
				m_LidSolenoid.set(DoubleSolenoid.Value.kReverse);
				if(!m_LidModeSwitch)//Waiting for button release
					m_LidToggle = 2;
				m_LidMode = "Down";
				break;
			case 2://Lid is down, looking for pressed
				m_LidSolenoid.set(DoubleSolenoid.Value.kReverse);
				if(m_LidModeSwitch)//Waiting for button press
					m_LidToggle = 3;
				m_LidMode = "Down";
				break;
			case 3://Lid is up, looking for release
				m_LidSolenoid.set(DoubleSolenoid.Value.kForward);
				if(!m_LidModeSwitch)//Waiting for button release
					m_LidToggle = 0;
				m_LidMode = "Up";
				break;
		}
		// Create movement input dead zones.
        if(Math.abs(m_driveX)<0.1) m_driveX=0;
        if(Math.abs(m_driveY)<0.1) m_driveY=0;
        if(Math.abs(m_rotation)<0.25) {
        	m_rotation=0;	// Create dead zone if rotation input is less than .25
        } else{
        	if (m_rotation < 0 )
            	m_rotation = m_rotation + 0.25; // Since we're creating a dead zone, make range 0 to .75 not .25 to 1. 
        	else
        		m_rotation = m_rotation - 0.25; // Since we're creating a dead zone, make range 0 to .75 not .25 to 1. 
        }
        
        if(m_POVFinal == -1){
        	// We are not POV strafing
	        // Allow for speed multiplier to control maximum speed. 
	        m_driveX = m_driveX * m_speedMultiplier; 
	        m_driveY = m_driveY * m_speedMultiplier; 
	        m_rotation = m_rotation * m_speedMultiplier; 	
	        
			if(AutoAimEnabled){
				double XFinal = Auto.acceptedXFinal();
				AutoDrive(0.233,XFinal);
			}else{
				//using field orientation using the gyro vs normal drive

				switch(m_DriveToggle){
					case 0://GyroDrive is in use, waiting for button to be pressed
						m_Drive.mecanumDrive_Cartesian(m_driveX,m_driveY,m_rotation,m_gyro.getAngle());
						if(m_DriveModeSwitch)//Waiting for button press
							m_DriveToggle = 1;
						break;
					case 1://Drive 2 selected, waiting for release
						m_Drive.mecanumDrive_Cartesian(m_driveX,m_driveY,m_rotation,0);
						if(!m_DriveModeSwitch)//Waiting for button release
							m_DriveToggle = 2;
						m_DriveMode = "Robot Oriented";
						break;
					case 2://Drive 2 selected, looking for pressed
						m_Drive.mecanumDrive_Cartesian(m_driveX,m_driveY,m_rotation,0);
						if(m_DriveModeSwitch)//Waiting for button press
							m_DriveToggle = 3;
						break;
					case 3://GyroDrive is in use, looking for release
						m_Drive.mecanumDrive_Cartesian(m_driveX,m_driveY,m_rotation,m_gyro.getAngle());
						if(!m_DriveModeSwitch)//Waiting for button release
							m_DriveToggle = 0;
						m_DriveMode = "Field Oriented";
						break;
				}
			}
        }else{
	 		 //left
	        if(m_POVFinal == 90){
	        	m_driveX = m_speedMultiplier; 
	        }	
	        //right
	        if(m_POVFinal == 270){
	        	m_driveX = -m_speedMultiplier; 
	        }   
			m_Drive.mecanumDrive_Cartesian(m_driveX,0,0,0);
	    }

		showInstrumentation();
		}	
	
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void showInstrumentation() {
		// We always want to display these values.
		double roundedGyro = (m_gyro.getAngle() % 360) * 10  ;
		roundedGyro = Math.round(roundedGyro); 
		roundedGyro = roundedGyro / 10;
		SmartDashboard.putNumber("gyroValue",roundedGyro);
		SmartDashboard.putString("lidStatus",m_LidMode );
		SmartDashboard.putString("driveMode",m_DriveMode );
		SmartDashboard.putString("autoAim",m_TeleAuto);
		// Values to show what is going on in Auto. 
		SmartDashboard.putNumber("Selected",m_autoSelected);
		SmartDashboard.putNumber("Step",m_autoCase);
        if(m_ShowAllData){
			SmartDashboard.putNumber("Joy Toggle value",m_JoyToggle);

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
			double avgDistance = (distanceFL + distanceBL + distanceFR + distanceBR ) / 4;
			SmartDashboard.putNumber("Front Left Encoder distance (in): ",distanceFL);
			SmartDashboard.putNumber("Back Left Encoder distance (in): ",distanceBL);
			SmartDashboard.putNumber("Front Right Encoder distance (in): ",distanceFR);
			SmartDashboard.putNumber("Back Right Encoder distance (in): ",distanceBR);
			SmartDashboard.putNumber("Average Encoder distance (in): ",avgDistance);
			
			SmartDashboard.putNumber("Drive Y value",m_driveY);
	        SmartDashboard.putNumber("Drive Rotation value",m_rotation);
			SmartDashboard.putNumber("Drive X value",m_driveX);
		}	
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
		if (!m_storedValueTF){
			m_storedAngle=(int) m_gyro.getAngle();
			m_storedValueTF=true;
		}
		if (m_autoTurnRight){
			if (m_gyro.getAngle() > m_storedAngle + Turn){
			m_storedValueTF=false;
			retVal=true;
			TurnPower=0;
			}
		}
		if (!m_autoTurnRight){
			TurnPower=-TurnPower;
			if (m_gyro.getAngle() < m_storedAngle - Turn){
			m_storedValueTF=false;
			retVal=true;
			TurnPower=0;
			}
		}
		m_Drive.mecanumDrive_Cartesian(0,0,TurnPower,m_gyro.getAngle());
		return retVal;
	}
	
	public boolean turnToHeading(int Heading,double TurnPower){
		boolean retVal=false;
		if (m_autoTurnRight){
			if (m_gyro.getAngle() >= Heading){
			retVal=true;
			TurnPower=0;
			}
		}
		if (!m_autoTurnRight){
			TurnPower=-TurnPower;
			if (m_gyro.getAngle() <= Heading){
			retVal=true;
			TurnPower=0;
			}
		}
		m_Drive.mecanumDrive_Cartesian(0,0,TurnPower,m_gyro.getAngle());
		return retVal;
	}
	
	public void AutoDrive(double speed, double XFinal){
		SmartDashboard.putNumber("XFinal",XFinal);
		if(!(XFinal == -666)){
			if(XFinal < 5 && XFinal > -5){
				m_Drive.mecanumDrive_Cartesian(0,-speed*0.75,0,0);
			}else{
				if(XFinal < 0){
					m_Drive.mecanumDrive_Cartesian((-speed*1.25),0,0,0);
				}else{
					m_Drive.mecanumDrive_Cartesian((speed*1.25),0,0,0);
				}				
			}
		}else{
			m_Drive.mecanumDrive_Cartesian(0,-speed,0,0);		
		}
	}

	public class stream extends Thread{
		// Whatever you do, DO NOT use two of the same port number.
		public void run(){
				
				int port = 1180;
				UsbCamera camera1 = new UsbCamera("DC", 0);
				camera1.setResolution(320, 240);

				//Creates server for streaming camera. Name is
				//local to the thread, but port # is not.
				MjpegServer mjpegStream = new MjpegServer("MjpegStream port: "+port, port);       //Starts main camera stream
				//Creates an image sink to feed the camera into
				CvSink imageSink = new CvSink("imageSink port "+port); //Creates a CV image sink
				imageSink.setSource(camera1);
				//Creates a CV source to pump MATS into to stream
				CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
				mjpegStream.setSource(imageSource);
				//Creates a MAT to add camera feed to
				Mat inputImage = new Mat();
				
				//Streams the camera on selected port using OpenCV.
				while(true){
					
					//Updates the Mat with feed info and checks framerate, and skips frame if error
					long frameTime = imageSink.grabFrame(inputImage);
	                if (frameTime == 0) {
	                	continue;
	                }
	              //Puts the frame grabbed into the CV source attached to the mjpeg server, thereby streaming it
	                imageSource.putFrame(inputImage); 
				}
		}
	}
}

