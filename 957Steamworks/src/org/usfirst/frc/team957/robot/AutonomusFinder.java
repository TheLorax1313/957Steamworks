package org.usfirst.frc.team957.robot;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.StringReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class AutonomusFinder{
	double acceptedXFinal = -666;	//Starts up variables for vision processing
	double acceptedYFinal = -666;
	double contourCounter;
	double acceptedContour1;
	double acceptedContour2;
	double minAR = 2;	//Sets up aspect ratio min and max
	double maxAR = 3;
	double CameraResX = 320;	//Sets up camera resolution
	double CameraResY= 280;
	int skew=-5;		//Gives processing formulas a skew value in case the camera is offset
	
	ServerSocket serverSocket = null;
	String text = null;
	String[] stringParts = {"0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","0","false"};
	clientManagement client = new clientManagement();
	
	ledCommunication LED = new ledCommunication();
	
	public void socketSetup(int server){	
		try {
			serverSocket =new ServerSocket(server);
			client.start();
		} catch (IOException e) {
			System.out.println("Unable to establish a Driver Station Server");
		}  	
	}
	
	public void AutoDetect(){
		
		//Defaults variables
		contourCounter = 0;	
		acceptedContour1 = -1;	
		acceptedContour2 = -1;
		double x0 = -9000;
		double x1 = -9000;
		double x2 = -9000;
		double x3 = -9000;
		double y0 = -9000;
		double y1 = -9000;
		double y2 = -9000;
		double y3 = -9000;
		double width0 = -9000;
		double width1 = -9000;
		double width2 = -9000;
		double width3 = -9000;
		double height0 = -9000;
		double height1 = -9000;
		double height2 = -9000;
		double height3 = -9000;
		double acceptedWidth1 = -1;		
		double acceptedWidth2 = -1;
		double acceptedHeight1 = -1;
		double acceptedHeight2 = -1;
		double acceptedX1 = -1;
		double acceptedX2 = -1;
		double acceptedY1 = -1;
		double acceptedY2 = -1;
		
		//Grabs socket information for vision processing
		x0 = Double.parseDouble(stringParts[0]);		
		x1 = Double.parseDouble(stringParts[1]);
		x2 = Double.parseDouble(stringParts[2]);
		x3 = Double.parseDouble(stringParts[3]);
		
		y0 = Double.parseDouble(stringParts[4]);
		y1 = Double.parseDouble(stringParts[5]);
		y2 = Double.parseDouble(stringParts[6]);
		y3 = Double.parseDouble(stringParts[7]);
		
		width0 = Double.parseDouble(stringParts[8]);
		width1 = Double.parseDouble(stringParts[9]);
		width2 = Double.parseDouble(stringParts[10]);
		width3 = Double.parseDouble(stringParts[11]);
		
		height0 = Double.parseDouble(stringParts[12]);
		height1 = Double.parseDouble(stringParts[13]);
		height2 = Double.parseDouble(stringParts[14]);
		height3 = Double.parseDouble(stringParts[15]);
	
		//Processes contours for aspect ratios
		contourAccepted(x0,y0,height0,width0,0);	
		contourAccepted(x1,y1,height1,width1,1);
		contourAccepted(x2,y2,height2,width2,2);
		contourAccepted(x3,y3,height3,width3,3);
		
				
		if(contourCounter == 2){	//If 2 acceptable contours are found, runs code to put accepted contours to a common variable for additional processing
			
			checkIfEqual((acceptedContour1 == 0), width0, height0, x0, y0, acceptedWidth1, acceptedHeight1, acceptedX1, acceptedY1);
			checkIfEqual((acceptedContour1 == 1), width1, height1, x1, y1, acceptedWidth1, acceptedHeight1, acceptedX1, acceptedY1);
			checkIfEqual((acceptedContour1 == 2), width2, height2, x2, y2, acceptedWidth1, acceptedHeight1, acceptedX1, acceptedY1);
			checkIfEqual((acceptedContour1 == 3), width3, height3, x3, y3, acceptedWidth1, acceptedHeight1, acceptedX1, acceptedY1);
			
			checkIfEqual((acceptedContour2 == 0), width0, height0, x0, y0, acceptedWidth2, acceptedHeight2, acceptedX2, acceptedY2);
			checkIfEqual((acceptedContour2 == 1), width1, height1, x1, y1, acceptedWidth2, acceptedHeight2, acceptedX2, acceptedY2);
			checkIfEqual((acceptedContour2 == 2), width2, height2, x2, y2, acceptedWidth2, acceptedHeight2, acceptedX2, acceptedY2);
			checkIfEqual((acceptedContour2 == 3), width3, height3, x3, y3, acceptedWidth2, acceptedHeight2, acceptedX2, acceptedY2);
			
			acceptedXFinal = ((acceptedX1 + acceptedX2)/2) - ((CameraResX)/2) + skew;	//Gives a variable that can be returned that can be used to drive to
            acceptedYFinal = ((acceptedY1 + acceptedY2)/2) - ((CameraResY)/2);    
            LED.update_xFinal(acceptedXFinal);
            
		}else{

			acceptedXFinal = -666;	//If no contours seen, sets the variable to return to nil
		}	
	}	
			
	public void contourAccepted(double x, double y, double h, double w, double cont){	//For contour processing, called above
		if(h/w > minAR && h/w < maxAR){		//If the contour has an accepted aspect ratio,
			if(acceptedContour1 == -1){		//check if accepted contours have been seen before,
				acceptedContour1 = cont;	//and puts the accepted contour in an open contour slot
				contourCounter++;
			}else{
				if(acceptedContour2 == -1){
					acceptedContour2 = cont;
					contourCounter++;
				}
			}
		}
	}
	
	public void checkIfEqual(boolean check, double o1, double o2, double o3, double o4, double e1, double e2, double e3, double e4){
		if(check == true){
			e1 = o1;
			e2 = o2;
			e3 = o3;
			e4 = o4;
		}
	}
	
	//Returns XFinal for driving
	public double acceptedXFinal(){
		return acceptedXFinal;		
	}
	
	//Returns YFinal for driving.
	public double acceptedYFinal(){
		return acceptedYFinal;		
	}
	
	//Connects a socket to a client and opens an InputStream to steal data from the client
	public class clientManagement extends Thread{
		public void run(){
			
			Socket socketClient = null;
			DataInputStream input = null;
			
			while (socketClient == null){
				try{	
					socketClient = serverSocket.accept();
					input = new DataInputStream(socketClient.getInputStream());
				}catch (Exception e){
					socketClient = null;
				}
			}
			
			while (true){
				
				try {
					text = input.readUTF();
				} catch (Exception e) {
					//Auto-generated catch block
					text = "ERROR";
					client.start();
					Thread.currentThread().interrupt();
				}
				if(stringParts[16].equals("true")){
					SmartDashboard.putString("text", "The robot does not have a Gear");
					LED.updateGearBoolean(true);

				}else{
					SmartDashboard.putString("text", "The robot has a gear");
					LED.updateGearBoolean(false);
					
				}
			}	
		}
	}
}
