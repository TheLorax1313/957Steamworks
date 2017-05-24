package org.usfirst.frc.team957.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class AutonomusFinder {
	double acceptedXFinal;	//Starts up variables for vision processing
	double acceptedYFinal;
	double contourCounter;
	double acceptedContour1;
	double acceptedContour2;
	double minAR = 2.25;	//Sets up aspect ratio min and max
	double maxAR = 2.75;
	double CameraResX = 320;	//Sets up camera resolution
	double CameraResY= 280;
	int skew=0;		//Gives processing formulas a skew value in case the camera is offset
	NetworkTable PiVision;	//Starts up networkTables information
	NetworkTable Pi_RioCom = NetworkTable.getTable("datatable");	//Connects NetworkTables to the variable
	
	public void AutoDetect(){
		Pi_RioCom.putNumber("X22", 1);	//Tells the Pi to process data (NOT IMPLETMENTED AT THIS TIME)
		acceptedXFinal = -666;	//Resets the point to drive to to nil
		acceptedYFinal = -666;
		contourCounter = 0;	//Resets correct contours seen to zero
		acceptedContour1 = -1;	//Sets accepted contours to nil
		acceptedContour2 = -1;
		
		double x0 = Pi_RioCom.getNumber("X0",-666);		//Grabs NetworkTables information for vision processing
		double x1 = Pi_RioCom.getNumber("X1",-666);
		double x2 = Pi_RioCom.getNumber("X2",-666);
		double x3 = Pi_RioCom.getNumber("X3",-666);
		
		double y0 = Pi_RioCom.getNumber("Y0",-666);
		double y1 = Pi_RioCom.getNumber("Y1",-666);
		double y2 = Pi_RioCom.getNumber("Y2",-666);
		double y3 = Pi_RioCom.getNumber("Y3",-666);
		
		double width0 = Pi_RioCom.getNumber("X4",-666);
		double width1 = Pi_RioCom.getNumber("X5",-666);
		double width2 = Pi_RioCom.getNumber("X6",-666);
		double width3 = Pi_RioCom.getNumber("X7",-666);
		
		double height0 = Pi_RioCom.getNumber("Y4",13);
		double height1 = Pi_RioCom.getNumber("Y5",14);
		double height2 = Pi_RioCom.getNumber("Y6",15);
		double height3 = Pi_RioCom.getNumber("Y7",16);
		
		double acceptedWidth1 = -1;		//Sets accepted points for processing to nil
		double acceptedWidth2 = -1;
		double acceptedHeight1 = -1;
		double acceptedHeight2 = -1;
		double acceptedX1 = -1;
		double acceptedX2 = -1;
		double acceptedY1 = -1;
		double acceptedY2 = -1;
		
		contourAccepted(x0,y0,height0,width0,0);	//Processes contours for aspect ratios
		contourAccepted(x1,y1,height1,width1,1);
		contourAccepted(x2,y2,height2,width2,2);
		contourAccepted(x3,y3,height3,width3,3);
		
		//If 1 contour found, runs code to attempt to align itself with the center of the the contour found.
		//Aligns with the CONTOUR, not the peg. This shouldn't matter, as this code will only run when it doesn't
		//see the second contour.
		
		
		
		if(contourCounter == 2){	//If 2 acceptable contours are found, runs code to put accepted contours to a common variable for additional processing
			
			if(acceptedContour1 == 0){
				acceptedWidth1 = width0;
				acceptedHeight1 = height0;
				acceptedX1 = x0;
				acceptedY1 = y0;
			}
			if(acceptedContour1 == 1){
				acceptedWidth1 = width1;
				acceptedHeight1 = height1;
				acceptedX1 = x1;
				acceptedY1 = y1;
			}
			if(acceptedContour1 == 2){
				acceptedWidth1 = width2;
				acceptedHeight1 = height2;
				acceptedX1 = x2;
				acceptedY1 = y2;
			}
			if(acceptedContour1 == 3){
				acceptedWidth1 = width3;
				acceptedHeight1 = height3;
				acceptedX1 = x3;
				acceptedY1 = y3;
			}
			if(acceptedContour2 == 0){
				acceptedWidth2 = width0;
				acceptedHeight2 = height0;
				acceptedX2 = x0;
				acceptedY2 = y0;
			}
			if(acceptedContour2 == 1){
				acceptedWidth2 = width1;
				acceptedHeight2 = height1;
				acceptedX2 = x1;
				acceptedY2 = y1;
			}
			if(acceptedContour2 == 2){
				acceptedWidth2 = width2;
				acceptedHeight2 = height2;
				acceptedX2 = x2;
				acceptedY2 = y2;
			}
			if(acceptedContour2 == 3){
				acceptedWidth2 = width3;
				acceptedHeight2 = height3;
				acceptedX2 = x3;
				acceptedY2 = y3;
			}
            
			acceptedXFinal = ((acceptedX1 + acceptedX2)/2) - ((CameraResX)/2);	//Gives a variable that can be returned that can be used to drive to
            acceptedYFinal = ((acceptedY1 + acceptedY2)/2) - ((CameraResY)/2);    
            
			}else{

				if(contourCounter == 1){	
					
					if(acceptedContour1 == 0){
						acceptedWidth1 = width0;
						acceptedHeight1 = height0;
						acceptedX1 = x0;
						acceptedY1 = y0;
					}
					if(acceptedContour1 == 1){
						acceptedWidth1 = width1;
						acceptedHeight1 = height1;
						acceptedX1 = x1;
						acceptedY1 = y1;
					}
					if(acceptedContour1 == 2){
						acceptedWidth1 = width2;
						acceptedHeight1 = height2;
						acceptedX1 = x2;
						acceptedY1 = y2;
					}
					if(acceptedContour1 == 3){
						acceptedWidth1 = width3;
						acceptedHeight1 = height3;
						acceptedX1 = x3;
						acceptedY1 = y3;
					}
					
					acceptedXFinal = acceptedX1 - ((CameraResX)/2);	//Gives a variable that can be returned that can be used to drive to
		            acceptedYFinal = acceptedY1 - ((CameraResY)/2);    
				}else{
					acceptedXFinal = -666;	//If no contours seen, sets the variable to return to nil
				}
			
				
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
	public double acceptedXFinal(){
		return acceptedXFinal;		//Returns XFinal for driving
	}
		
	public double acceptedYFinal() {
		return acceptedYFinal;		//Returns YFinal for driving.
	}
		
}
