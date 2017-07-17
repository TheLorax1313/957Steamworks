package org.usfirst.frc.team957.robot;

import java.io.BufferedReader;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.StringReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import com.ctre.CANTalon;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
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
	double CameraResY= 240;
	int skew=0;		//Gives processing formulas a skew value in case the camera is offset
	boolean gear = false;
	ServerSocket serverSocket = null;
	GripPipeline gp = new GripPipeline();
	GripPipelineGear gpg = new GripPipelineGear();
	ledCommunication LED = new ledCommunication();
	processCamera processCamera = new processCamera();
	
	
	public void startProcessing(){
		processCamera.start();
	}
	
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
	
	public void AutoDetect(){
		
		//Defaults variables
		contourCounter = 0;	
		acceptedContour1 = -1;	
		acceptedContour2 = -1;
		
		double acceptedWidth1 = -1;		
		double acceptedWidth2 = -1;
		double acceptedHeight1 = -1;
		double acceptedHeight2 = -1;
		double acceptedX1 = -1;
		double acceptedX2 = -1;
		double acceptedY1 = -1;
		double acceptedY2 = -1;
		
		
	
		//Processes contours for aspect ratios
		contourAccepted(x0,y0,height0,width0,0);	
		contourAccepted(x1,y1,height1,width1,1);
		contourAccepted(x2,y2,height2,width2,2);
		contourAccepted(x3,y3,height3,width3,3);
		
				
        if(contourCounter == 2){    //If 2 acceptable contours are found, runs code to put accepted contours to a common variable for additional processing
            
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
            
            acceptedXFinal = ((acceptedX1 + acceptedX2)/2) - ((CameraResX)/2);    //Gives a variable that can be returned that can be used to drive to

            
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
			o1 = e1;
			o2 = e2;
			o2 = e3;
			o2 = e4;
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
	
	public boolean gear(){
		return gear;
	}
	
	public class processCamera extends Thread{
		public void run(){
			
			ArrayList<MatOfPoint> gpArray = new ArrayList<MatOfPoint>();
			ArrayList<MatOfPoint> gpgArray = new ArrayList<MatOfPoint>();
			UsbCamera driveCam = new UsbCamera ("DC", 0);
			driveCam.setResolution(320, 240);
			
			UsbCamera gearCam = new UsbCamera ("GC", 1);
			gearCam.setResolution(160, 120);
			gearCam.setFPS(15);
			CvSink gearSink = new CvSink ("gear");
			gearSink.setSource(gearCam);
			
			CvSink imageSink = new CvSink("CV Image Grabber");      //Starts a CV sink to pull camera footage into a MAT image file
            imageSink.setSource(driveCam);
            
            MjpegServer stream1180 = new MjpegServer("DC", 1180);
            stream1180.setSource(driveCam);
            
            MjpegServer stream1181 = new MjpegServer("GC", 1181);
            stream1181.setSource(gearCam);
            
            CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);     //Starts a CV Source to pass MATs into
            MjpegServer cvStream = new MjpegServer("CV Image Stream", 1182);        //Starts a camera server and sets it to stream what is passed into the CV Source, the processed image, most likely
            cvStream.setSource(imageSource);
            Mat inputImage = new Mat(); 
            Mat gearImage = new Mat(); 
            
            while(true){
            	long frameTime = imageSink.grabFrame(inputImage);
            	if (frameTime == 0){    //Skips whatever below if the framerate = 0
                    continue;
            	}
            	
            	 gp.process(inputImage);
            	 gpArray = gp.filterContoursOutput();
            	 int gpLength = gpArray.size();
            	 
            	 if(gpLength > 0){       //1 contour seen
                     Rect r0 = Imgproc.boundingRect(gp.filterContoursOutput().get(0));       //Creates a rectangle based on information from a contour
                     x0 = r0.x+(r0.width/2);       //Puts information into a variable that will later be sent to the Rio regarding bounding box center, width, and height of the contour
                     y0 = r0.y+(r0.height/2);
                     width0 = r0.width;
                     height0 = r0.height;
                     Imgproc.rectangle(inputImage, new Point(r0.x,r0.y), new Point(r0.x+r0.width , r0.y+r0.height), new Scalar(0, 255, 0),5);        //Draws the rectangle on the image itself
	             }else{
                     x0 = -9000;   //Sets networkTables variables to an easily recognizable error value
                     y0 = -9000;
                     width0 = -9000;
                     height0 = -9000;
	             }
            	 
            	 if(gpLength > 1){       //1 contour seen
                     Rect r1 = Imgproc.boundingRect(gp.filterContoursOutput().get(1));       //Creates a rectangle based on information from a contour
                     x1 = r1.x+(r1.width/2);       //Puts information into a variable that will later be sent to the Rio regarding bounding box center, width, and height of the contour
                     y1 = r1.y+(r1.height/2);
                     width1 = r1.width;
                     height1 = r1.height;
                     Imgproc.rectangle(inputImage, new Point(r1.x,r1.y), new Point(r1.x+r1.width , r1.y+r1.height), new Scalar(0, 255, 0),5);        //Draws the rectangle on the image itself
	             }else{
                     x1 = -9000;   //Sets networkTables variables to an easily recognizable error value
                     y1 = -9000;
                     width1 = -9000;
                     height1 = -9000;
	             }
            	 
            	 if(gpLength > 2){       //1 contour seen
                     Rect r2 = Imgproc.boundingRect(gp.filterContoursOutput().get(2));       //Creates a rectangle based on information from a contour
                     x2 = r2.x+(r2.width/2);       //Puts information into a variable that will later be sent to the Rio regarding bounding box center, width, and height of the contour
                     y2 = r2.y+(r2.height/2);
                     width2 = r2.width;
                     height2 = r2.height;
                     Imgproc.rectangle(inputImage, new Point(r2.x,r2.y), new Point(r2.x+r2.width , r2.y+r2.height), new Scalar(0, 255, 0),5);        //Draws the rectangle on the image itself
	             }else{
                     x2 = -9000;   //Sets networkTables variables to an easily recognizable error value
                     y2 = -9000;
                     width2 = -9000;
                     height2 = -9000;
	             }
            	 
            	 if(gpLength > 3){       //1 contour seen
                     Rect r3 = Imgproc.boundingRect(gp.filterContoursOutput().get(3));       //Creates a rectangle based on information from a contour
                     x3 = r3.x+(r3.width/2);       //Puts information into a variable that will later be sent to the Rio regarding bounding box center, width, and height of the contour
                     y3 = r3.y+(r3.height/2);
                     width2 = r3.width;
                     height2 = r3.height;
                     Imgproc.rectangle(inputImage, new Point(r3.x,r3.y), new Point(r3.x+r3.width , r3.y+r3.height), new Scalar(0, 255, 0),5);        //Draws the rectangle on the image itself
	             }else{
                     x3 = -9000;   //Sets networkTables variables to an easily recognizable error value
                     y3 = -9000;
                     width3 = -9000;
                     height3 = -9000;
	             }
            	 imageSource.putFrame(inputImage);  
            	 
            	long frameTimeGear = gearSink.grabFrame(gearImage);
             	if (frameTimeGear == 0){    //Skips whatever below if the framerate = 0
                     continue;
             	}
             	
             	gpg.process(gearImage);
             	
             	gpgArray = gpg.filterContoursOutput();
             	int gpgLength = gpgArray.size();
             	
             	gear = gpgLength > 0;  

            }
		}
	}
}
