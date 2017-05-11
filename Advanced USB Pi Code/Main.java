import java.util.ArrayList;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.*;
import edu.wpi.cscore.*;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;
import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
import java.io.*;
import java.util.Scanner;

public class Main {
        public static void main(String[] args) throws Exception{

                System.loadLibrary("opencv_java310");   //Loads OpenCV libraries
                NetworkTable.setClientMode();           //Initialies NetworkTables and sets team #
                NetworkTable.setTeam(957);
                NetworkTable.initialize();
                NetworkTable Pi_RioCom = NetworkTable.getTable("datatable");
                
                int drivePort = -1;
                int gearPort = -1;
                int climbPort = -1;
                int lineCount = 0;
                String s;

                // Creates a scanner to scan for serial numbers in the text file created before launching 
                // the program, and compares it to a list of serial numbers that correspond to our cameras
                Scanner sc = new Scanner(new File("/home/pi/usbData/usbDataFinal.txt"));
                while(sc.hasNext()){
                        s = sc.next();
                        System.out.println("|Serial Number for /dev/video" + lineCount +" : " + s+"|");
                        if(s.equals("4C30AB10")){
                                drivePort = lineCount;
                                System.out.println("Drive Camera Detected");
                        }
                        lineCount++;
                }

                if(drivePort == -1){
                        drivePort = 4;
                }
                if(gearPort == -1){
                        gearPort = 5;
                }
                if(climbPort == -1){
                        climbPort = 6;
                }
                
                GripPipeline gp = new GripPipeline();           //Initializes the GripPipeline Class 
                ArrayList<MatOfPoint> gpArray = new ArrayList<MatOfPoint>();    //Starts contour counting variable
                MjpegServer DSStream = new MjpegServer("DSStream", 1180);       //Starts main camera stream

                UsbCamera camera0 = new UsbCamera("Drive/Test Camera", drivePort);        //Starts drive cam and sets resolution
                camera0.setResolution(320,240);

                UsbCamera camera1 = new UsbCamera("Gear Camera", gearPort);        //Starts gear cam, sets resolution, and sets framerate
                camera1.setResolution(320,240);
                camera1.setFPS(10);

                UsbCamera camera2 = new UsbCamera("Climber Camera", climbPort);        //Starts climb cam, sets resolution, and sets framerate
                camera2.setResolution(320,240);
                camera2.setFPS(10);

                CvSink imageSink = new CvSink("CV Image Grabber");      //Starts a CV sink to pull camera footage into a MAT image file
                imageSink.setSource(camera0);           //Sets CV sink to Camera 0

                CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);     //Starts a CV Source to pass MATs into
                MjpegServer cvStream = new MjpegServer("CV Image Stream", 1181);        //Starts a camera server and sets it to stream what is passed into the CV Source
                cvStream.setSource(imageSource);

                Mat inputImage = new Mat();     //Defines a MAT to pass camera feed into, process, mark up, and stream again on the cvStream server
//              Mat hsv = new Mat();
                double m_CameraSwitch;          //Initializes variable stolen from the Rio to switch cameras on the main stream
                double prevSwitch = -1;

                double rectCenterX0;            //Initializes variables to send NetworkTables data to the Rio regarding contour information
                double rectCenterX1;
                double rectCenterX2;
                double rectCenterX3;

                double rectCenterY0;
                double rectCenterY1;
                double rectCenterY2;
                double rectCenterY3;

                double rectWidth0;
                double rectWidth1;
                double rectWidth2;
                double rectWidth3;

                double rectHeight0;
                double rectHeight1;
                double rectHeight2;
                double rectHeight3;
                //ALL IMAGE PROCESSING CODE BELOW HERE
                while (true) {
                        m_CameraSwitch = Pi_RioCom.getNumber("X20",0);  //Gets networkTables information to set the main stream (1180) to whatever camera we want
                        
                        if(m_CameraSwitch == 0 && prevSwitch != m_CameraSwitch){
                                DSStream.setSource(camera0);    //Sets camera to stream the drive cam
                        }
                        
                        if(m_CameraSwitch == 2 && !(prevSwitch == m_CameraSwitch)){
                                DSStream.setSource(camera1);    //Stream Gear Cam
                        }
                        
                        if(m_CameraSwitch == 4 && !(prevSwitch == m_CameraSwitch)){
                                DSStream.setSource(camera2);    //Steam Climber Cam
                        }
                        prevSwitch = m_CameraSwitch;

                        long frameTime = imageSink.grabFrame(inputImage);       //Grabs latest frame from the CV sink and pastes it into the InputImage MAT for processing.
                        
                        if (frameTime == 0){    //Skips whatever below if the framerate = 0
                                continue;
                        }

                        rectCenterX0 = 0;       //Resets all NetworkTables variables for sending vision data
                        rectCenterX1 = 0;
                        rectCenterX2 = 0;
                        rectCenterX3 = 0;

                        rectCenterY0 = 0;
                        rectCenterY1 = 0;
                        rectCenterY2 = 0;
                        rectCenterY3 = 0;

                        rectWidth0 = 0;
                        rectWidth1 = 0;
                        rectWidth2 = 0;
                        rectWidth3 = 0;

                        rectHeight0 = 0;
                        rectHeight1 = 0;
                        rectHeight2 = 0;
                        rectHeight3 = 0;

                        gp.process(inputImage);         //Processes whatever is in the InputImage Mat, even if it is mangled garbage
                        gpArray = gp.filterContoursOutput();    //Grabs from the GripPipeline class the contour information
                        int gpLength = gpArray.size();          //Asks from gpArray how many contours were seen
                        //The following code is ran depending on how many contours were seen
                        if(gpLength > 0){       //1 contour seen
                                Rect r0 = Imgproc.boundingRect(gp.filterContoursOutput().get(0));       //Creates a rectangle based on information from a contour
                                rectCenterX0 = r0.x+(r0.width/2);       //Puts information into a variable that will later be sent to the Rio regarding bounding box center, width, and height of the contour
                                rectCenterY0 = r0.y+(r0.height/2);
                                rectWidth0 = r0.width;
                                rectHeight0 = r0.height;
                                Imgproc.rectangle(inputImage, new Point(r0.x,r0.y), new Point(r0.x+r0.width , r0.y+r0.height), new Scalar(0, 255, 0),5);        //Draws the rectangle on the image itself
                        }else{
                                rectCenterX0 = -9000;   //Sets networkTables variables to an easily recognizable error value
                                rectCenterY0 = -9000;
                                rectWidth0 = -9000;
                                rectHeight0 = -9000;
                        }

                        if(gpLength > 1){       //2 contours seen
                                Rect r1 = Imgproc.boundingRect(gp.filterContoursOutput().get(1));        //Creates a rectangle based on information from a contour
                                rectCenterX1 = r1.x+(r1.width/2);       //Puts information into a variable that will later be sent to the Rio regarding bounding box center, width, and height of the contour
                                rectCenterY1 = r1.y+(r1.height/2);
                                rectWidth1 = r1.width;
                                rectHeight1 = r1.height;
                                Imgproc.rectangle(inputImage, new Point(r1.x,r1.y), new Point(r1.x+r1.width , r1.y+r1.height), new Scalar(0, 255, 0),5);        //Draws the rectangle on the image itself      
                        }else{
                                rectCenterX1 = -9000;   //Sets networkTables variables to an easily recognizable error value
                                rectCenterY1 = -9000;
                                rectWidth1 = -9000;
                                rectHeight1 = -9000;
                        }

                        if(gpLength > 2){       //3 contours seen
                                Rect r2 = Imgproc.boundingRect(gp.filterContoursOutput().get(2));        //Creates a rectangle based on information from a contour
                                rectCenterX2 = r2.x+(r2.width/2);
                                rectCenterY2 = r2.y+(r2.height/2);
                                rectWidth2 = r2.width;
                                rectHeight2 = r2.height;
                                Imgproc.rectangle(inputImage, new Point(r2.x,r2.y), new Point(r2.x+r2.width , r2.y+r2.height), new Scalar(0, 255, 0),5);        //Draws the rectangle on the image itself
                        }else{
                                rectCenterX2 = -9000;   //Sets networkTables variables to an easily recognizable error value
                                rectCenterY2 = -9000;
                                rectWidth2 = -9000;
                                rectHeight2 = -9000;
                        }

                        if(gpLength > 3){       //4 contours seen
                                Rect r3 = Imgproc.boundingRect(gp.filterContoursOutput().get(3));        //Creates a rectangle based on information from a contour
                                rectCenterX3 = r3.x+(r3.width/2);
                                rectCenterY3 = r3.y+(r3.height/2);
                                rectWidth3 = r3.width;
                                rectHeight3 = r3.height;
                                Imgproc.rectangle(inputImage, new Point(r3.x,r3.y), new Point(r3.x+r3.width , r3.y+r3.height), new Scalar(0, 255, 0),5);
                        }else{
                                rectCenterX3 = -9000;   //Sets networkTables variables to an easily recognizable error value
                                rectCenterY3 = -9000;
                                rectWidth3 = -9000;
                                rectHeight3 = -9000;
                        }

                        //Placing data point of center of bounding box
                        Pi_RioCom.putNumber("X0",rectCenterX0);
                        Pi_RioCom.putNumber("Y0",rectCenterY0);
                        Pi_RioCom.putNumber("X1",rectCenterX1);
                        Pi_RioCom.putNumber("Y1",rectCenterY1);
                        Pi_RioCom.putNumber("X2",rectCenterX2);
                        Pi_RioCom.putNumber("Y2",rectCenterY2);
                        Pi_RioCom.putNumber("X3",rectCenterX3);
                        Pi_RioCom.putNumber("Y3",rectCenterY3);

                        //Placing Width
                        Pi_RioCom.putNumber("X4",rectWidth0);
                        Pi_RioCom.putNumber("X5",rectWidth1);
                        Pi_RioCom.putNumber("X6",rectWidth2);
                        Pi_RioCom.putNumber("X7",rectWidth3);
                        //Placing Height
                        Pi_RioCom.putNumber("Y4",rectHeight0);
                        Pi_RioCom.putNumber("Y5",rectHeight1);
                        Pi_RioCom.putNumber("Y6",rectHeight2);
                        Pi_RioCom.putNumber("Y7",rectHeight3);

                        imageSource.putFrame(inputImage);       //Streams the the marked-up MAT on port 1181
                }
        }
}