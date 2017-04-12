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

public class Main {
  public static void main(String[] args) {

    System.loadLibrary("opencv_java310");
    NetworkTable.setClientMode();
    NetworkTable.setTeam(957);
    NetworkTable.initialize();
        NetworkTable Pi_RioCom = NetworkTable.getTable("datatable");
        GripPipeline gp = new GripPipeline();
        ArrayList<MatOfPoint> gpArray = new ArrayList<MatOfPoint>();
    MjpegServer DSStream = new MjpegServer("DSStream", 1180);

        UsbCamera camera0 = new UsbCamera("Camera0", 0);
    camera0.setResolution(320,240);

    UsbCamera camera1 = new UsbCamera("Camera1", 1);
    camera1.setResolution(320,240);
        camera1.setFPS(10);

        UsbCamera camera2 = new UsbCamera("Camera2", 2);
    camera2.setResolution(320,240);
        camera2.setFPS(10);

    CvSink imageSink = new CvSink("CV Image Grabber");
    imageSink.setSource(camera0);

    CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
    MjpegServer cvStream = new MjpegServer("CV Image Stream", 1181);
    cvStream.setSource(imageSource);

    Mat inputImage = new Mat();
    Mat hsv = new Mat();
        double m_CameraSwitch;

        double rectCenterX0;
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

    while (true) {
                m_CameraSwitch = Pi_RioCom.getNumber("X20",0);
                if(m_CameraSwitch == 0){
                        DSStream.setSource(camera0);
                }
                if(m_CameraSwitch == 2){
                        DSStream.setSource(camera1);
                }
                if(m_CameraSwitch == 4){
                        DSStream.setSource(camera2);
                }

                long frameTime = imageSink.grabFrame(inputImage);
                if (frameTime == 0){
                        continue;
                }

                rectCenterX0 = 0;
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

                gp.process(inputImage);
                gpArray = gp.filterContoursOutput();
                int gpLength = gpArray.size();

                 if(gpLength > 0){

                        Rect r0 = Imgproc.boundingRect(gp.filterContoursOutput().get(0));
                        rectCenterX0 = r0.x+(r0.width/2);
                        rectCenterY0 = r0.y+(r0.height/2);
                        rectWidth0 = r0.width;
                        rectHeight0 = r0.height;
                        Imgproc.rectangle(inputImage, new Point(r0.x,r0.y), new Point(r0.x+r0.width , r0.y+r0.height), new Scalar(0, 255, 0),5);

                }else{
                        rectCenterX0 = -9000;
                        rectCenterY0 = -9000;
                        rectWidth0 = -9000;
                        rectHeight0 = -9000;
                }

                if(gpLength > 1){

                        Rect r1 = Imgproc.boundingRect(gp.filterContoursOutput().get(1));
                        rectCenterX1 = r1.x+(r1.width/2);
                        rectCenterY1 = r1.y+(r1.height/2);
                        rectWidth1 = r1.width;
                        rectHeight1 = r1.height;
                        Imgproc.rectangle(inputImage, new Point(r1.x,r1.y), new Point(r1.x+r1.width , r1.y+r1.height), new Scalar(0, 255, 0),5);

                }else{
                        rectCenterX1 = -9000;
                        rectCenterY1 = -9000;
                        rectWidth1 = -9000;
                        rectHeight1 = -9000;
                }

                if(gpLength > 2){

                        Rect r2 = Imgproc.boundingRect(gp.filterContoursOutput().get(2));
                        rectCenterX2 = r2.x+(r2.width/2);
                        rectCenterY2 = r2.y+(r2.height/2);
                        rectWidth2 = r2.width;
                        rectHeight2 = r2.height;
                        Imgproc.rectangle(inputImage, new Point(r2.x,r2.y), new Point(r2.x+r2.width , r2.y+r2.height), new Scalar(0, 255, 0),5);

                }else{
                        rectCenterX2 = -9000;
                        rectCenterY2 = -9000;
                        rectWidth2 = -9000;
                        rectHeight2 = -9000;
                }

                if(gpLength > 3){

                        Rect r3 = Imgproc.boundingRect(gp.filterContoursOutput().get(3));
                        rectCenterX3 = r3.x+(r3.width/2);
                        rectCenterY3 = r3.y+(r3.height/2);
                        rectWidth3 = r3.width;
                        rectHeight3 = r3.height;
                        Imgproc.rectangle(inputImage, new Point(r3.x,r3.y), new Point(r3.x+r3.width , r3.y+r3.height), new Scalar(0, 255, 0),5);

                }else{
                        rectCenterX3 = -9000;
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

                imageSource.putFrame(inputImage);
    }
  }
}
