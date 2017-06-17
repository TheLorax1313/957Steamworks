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
import java.net.*;  

public class Main {
        public static void main(String[] args) throws Exception{

        System.loadLibrary("opencv_java310");
        NetworkTable.setClientMode();
        NetworkTable.setTeam(957);
        NetworkTable.initialize();
        NetworkTable Pi_RioCom = NetworkTable.getTable("datatable");
        
        GripPipeline gp = new GripPipeline();
        GripPipelineGear gpGear = new GripPipelineGear();
        
        UsbCamera driveCam = new UsbCamera("Camera0", 0);        //Starts the front camera and sets resolution 
        driveCam.setResolution(320,280);
        driveCam.setFPS(10);
        
        UsbCamera gearCam = new UsbCamera("Camera1", 1);        //Starts the gear camera, sets resolution, and sets framerate
        gearCam.setResolution(320,280);
        gearCam.setFPS(10);

        ArrayList<MatOfPoint> gpArray = new ArrayList<MatOfPoint>();
        ArrayList<MatOfPoint> gpArrayGear = new ArrayList<MatOfPoint>();
        boolean gear = false;
        
        CvSink imageSink = new CvSink("CV Image Grabber");
        imageSink.setSource(driveCam);

        CvSink imageSinkGear = new CvSink("CV Image Grabber");
        imageSinkGear.setSource(gearCam);

        CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
        MjpegServer cvStream = new MjpegServer("CV Image Stream", 1184);
        cvStream.setSource(imageSource);

        CvSource imageSourceGear = new CvSource("CV Image Source Gear", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
        MjpegServer cvStreamGear = new MjpegServer("CV Image Stream Gear", 1185);
        cvStreamGear.setSource(imageSourceGear);

        //Camera Servers
        MjpegServer dsStream1 = new MjpegServer("CV Image Stream", 1180);
        MjpegServer dsStream2 = new MjpegServer("CV Image Stream", 1181);
        dsStream1.setSource(driveCam);
        dsStream2.setSource(gearCam);

        Mat inputImage = new Mat();
        Mat gearImage = new Mat();
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
        
        Socket clientSocket = null;
        //Runs until socket found
        while(clientSocket == null){
                try{
                        //If the socket is present and can be connected to
                        clientSocket = new Socket("10.9.57.2", 1189);
                        System.out.println("SOCKET FOUND.");
                }catch(Exception e){
                        //Ran if unable to connect to the socket
                        System.out.println("SEARCHING FOR SOCKET 10.9.57.2:1189!!!");
                        clientSocket = null;
                }
        }
        //Creates a data output stream and flushes null values
        DataOutputStream dout = new DataOutputStream(clientSocket.getOutputStream());
        dout.flush();
        int loopCount = 0;
        //  Infinitely process image
        while (true) {
                loopCount++;
                if(loopCount/2 == Math.round(loopCount/2)){
                         // Grab a frame. If it has a frame time of 0, there was an error.
                        // Just skip and continue
                        long frameTime = imageSink.grabFrame(inputImage);
                        long frameTimeGear = imageSinkGear.grabFrame(gearImage);
                        if (frameTime == 0 || frameTimeGear == 0) {
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

                        gp.process(inputImage);     //Processes whatever is in the InputImage Mat, even if it is mangled garbage
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

                        gpGear.process(gearImage);     //Processes whatever is in the InputImage Mat, even if it is mangled garbage
                        gpArrayGear = gpGear.filterContoursOutput();    //Grabs from the GripPipeline class the contour information
                        int gpLengthGear = gpArrayGear.size();          //Asks from gpArray how many contours were seen
                        
                        if(gpLengthGear > 0 && gpLengthGear < 2){
                                gear = true;
                        }
                        imageSource.putFrame(inputImage);    
                        imageSourceGear.putFrame(gearImage);
                        //builds a data string to send to the Rio
                        String data = rectCenterX0 + "," + rectCenterX1 + "," + rectCenterX2 + "," + rectCenterX3 + "," + rectCenterY0 + "," + rectCenterY1 + "," + rectCenterY2 + "," + rectCenterY3 + "," + rectWidth0 + "," + rectWidth1 + "," + rectWidth2 + "," + rectWidth3 + "," + rectHeight0 + "," + rectHeight1 + "," + rectHeight2 + "," + rectHeight3 + "," + gear;
                        System.out.println(data);
                        //Writes the data to the socket and erases the data in the output stream
                        dout.writeUTF(data);  
                        dout.flush();
                }
               
                }
        }
}