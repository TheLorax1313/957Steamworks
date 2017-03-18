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
	double acceptedXFinal = -666;
	double acceptedYFinal = -666;
	double acceptedXFinal_ret = -666;
	NetworkTable PiVision;
	double contourCounter = 0;
	double acceptedContour1;
	double acceptedContour2;
	double minAR = 2.0;
	double maxAR = 3;
	double CameraResX = 320;
	double CameraResY= 280;
	double distance;
	double trueHeight = 5;
	double trueWidth = 8.25;
	double FOV = 69;
	double DashboardDistance;
	double botHeight =  19.25; //minus tape height
	int skew = 10;
	public void distanceInit(){
		distance = 80; //this is to give the variable distance an initial 
		//value that won't be given again during Auto to make it so there 
		//will not be any NaN errors.
	}
	public void AutoDetect(){
		NetworkTable Pi_RioCom = NetworkTable.getTable("datatable");
		acceptedXFinal = -666;
		acceptedYFinal = -666;
		contourCounter = 0;
		acceptedContour1 = -1;
		acceptedContour2 = -1;
		boolean B_Distance;
		
		double x0 = Pi_RioCom.getNumber("X0",0);
		double x1 = Pi_RioCom.getNumber("X1",2);
		double x2 = Pi_RioCom.getNumber("X2",4);
		double x3 = Pi_RioCom.getNumber("X3",6);
		
		double y0 = Pi_RioCom.getNumber("Y0",1);
		double y1 = Pi_RioCom.getNumber("Y1",3);
		double y2 = Pi_RioCom.getNumber("Y2",5);
		double y3 = Pi_RioCom.getNumber("Y3",7);
		
		double width0 = Pi_RioCom.getNumber("X4",8);
		double width1 = Pi_RioCom.getNumber("X5",9);
		double width2 = Pi_RioCom.getNumber("X6",10);
		double width3 = Pi_RioCom.getNumber("X7",11);
		
		double height0 = Pi_RioCom.getNumber("Y4",13);
		double height1 = Pi_RioCom.getNumber("Y5",14);
		double height2 = Pi_RioCom.getNumber("Y6",15);
		double height3 = Pi_RioCom.getNumber("Y7",16);
		
		double acceptedWidth1 = -1;
		double acceptedWidth2 = -1;
		double acceptedHeight1 = -1;
		double acceptedHeight2 = -1;
		double acceptedX1 = -1;
		double acceptedX2 = -1;
		double acceptedY1 = -1;
		double acceptedY2 = -1;
		
		
	
		contourAccepted(x0,y0,height0,width0,0);
		contourAccepted(x1,y1,height1,width1,1);
		contourAccepted(x2,y2,height2,width2,2);
		contourAccepted(x3,y3,height3,width3,3);
			
		if(contourCounter == 2){
			
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
			acceptedXFinal_ret = ((acceptedX1 + acceptedX2)/2) - ((CameraResX)/2) + skew;
			acceptedYFinal = ((acceptedY1 + acceptedY2)/2) - ((CameraResY)/2);	
			acceptedXFinal = ((acceptedX1 + acceptedX2)/2) - ((CameraResX)/2);
			double widthPixels = Math.abs(acceptedX1 - acceptedX2);
			double objectFOV_Tri = ((widthPixels/CameraResX)*FOV)/2;
			double hyp = (trueWidth/2)/(Math.tan(objectFOV_Tri)) ;
			double distAng = Math.cos(botHeight/hyp);
			distance = Math.tan(distAng)*botHeight;
		}
		
		SmartDashboard.putNumber("distance",distance);	
	}
	
		

	public void contourAccepted(double x, double y, double h, double w, double cont){
		if(h/w > minAR && h/w < maxAR){	
			if(acceptedContour1 == -1){
				acceptedContour1 = cont;
				contourCounter++;
			}else{
			if(acceptedContour2 == -1){
				acceptedContour2 = cont;
				contourCounter++;
			}
		}
	}
	}
	
	public double acceptedXFinal() {
		return acceptedXFinal_ret;
	}
	public double acceptedYFinal() {
		return acceptedYFinal;
	}
	
	public double distance(){
		return distance;
	}
	
}
