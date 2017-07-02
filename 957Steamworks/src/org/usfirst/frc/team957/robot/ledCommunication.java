package org.usfirst.frc.team957.robot;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;

import edu.wpi.first.wpilibj.DriverStation;

/*
 * Class for sending LED information to an
 * Arduino with an Ethernet shield or an
 * Arduino with a built in Ethernet jack.
 * 
 * Data interfaced via voids to be called
 * in "Robot.java" which adjust variables
 * that can be read by a seperate thread.
*/

public class ledCommunication {
	
	// Data Init
	String alliance = "Invalid";
	String climbing = "false";
	String gear = "false";
	double xFinal = 2;
	String data = "Invalid,false,false,2";
	
	// Socket Init
	ServerSocket serverSocket = null;
	
	// Thread init
	clientManagement client = new clientManagement();
	
	public void socketSetup(int server){ // Activates a single client once comms are established
		
		try {
			serverSocket = null;
			serverSocket = new ServerSocket(server);
			client.start();
		} catch (IOException e) {
			System.out.println("Unable to init Arduino server");
		}  
		
	}
	
	public void updateAlliance(){ // Updates the Alliance Color in data transmission
		
		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();
		
		if(color == DriverStation.Alliance.Blue){
			alliance = "blue   ";
		}
		if(color == DriverStation.Alliance.Red){
			alliance = "red    ";
		}
		if(color == DriverStation.Alliance.Invalid){
			alliance = "invalid";	
		}
	}
	
	public void updateClimbingBoolean(boolean trigger){ // Updates if we are climbing
		if(trigger == true){
			climbing = "true ";
		}else{
			climbing = "false";
		}
	}
	
	public void updateGearBoolean(boolean trigger){ // Updates if we have a gear
		if(trigger == true){
			gear = "true ";
		}else{
			gear = "false";
		}								
	}
	
	public void update_xFinal(double acceptedXFinal){ // Updates xFinal in data
		if(acceptedXFinal > 20){
			xFinal = 3;
		}else{
			if(acceptedXFinal < -20){
				xFinal = 1;
			}else{
				xFinal = 2;
			}
		}
	}
	
	public class clientManagement extends Thread{
		
		public void run(){
			
			Socket socketClient = null;
			DataOutputStream output = null;
			
			while(socketClient == null){ // Attempts until client is found
				try {
					socketClient = serverSocket.accept();
				} catch (IOException e) {
					socketClient = null;
				}
			}
			
			while(output == null){	// Attempts until Data Output Stream declared
				
				try {
					output = new DataOutputStream(socketClient.getOutputStream());
					output.flush();
				} catch (IOException e) {
					output = null;
				}
			}
			
			while(true){
				
				// Compiles data into one nice neat string
				
				if(DriverStation.getInstance().isOperatorControl() == true){
					data = alliance + "," + gear  + "," + climbing + "," + xFinal + "," + "true ";
				}else{
					data = alliance + "," + gear + "," + climbing + "," + xFinal + "," + "false";
				}
				
				
				// Sends data to the Arduino/Ethernet Shield
				try {
					output.writeUTF(data);
					output.flush();
				} catch (Exception e) {
					System.out.println("Unable to send Arduino data");
					
					try{
						output.close();
						socketClient.close();
						serverSocket = null;
					}catch(Exception E){
					}
					
					socketSetup(5801);
					return;
				}  
				
				// Sleeps to attempt to prevent a DOS overload
                try {
					Thread.sleep(250);
				} catch (Exception e) {
				}
				
			
                
			}
			
		}			
	}	
}

