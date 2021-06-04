import processing.serial.*;

Serial    myPort;
short     portIndex = 1;
int       lf = 10;             //ASCII linefeed
String    inString;            //String for testing serial communication
int       calibrating;
 
float dt;
float x_gyr;    //Gyroscope data
float y_gyr;
float z_gyr;
float x_acc;    //Accelerometer data
float y_acc;
float z_acc;
float x_fil;    //Filtered data
float y_fil;
float z_fil;

double x = 0;
double y = 1;

float mag = 0;
float speed = 0.05;


void setup()    {
    noStroke();
    colorMode(RGB, 256); 
    
    myPort = new Serial(this, "COM4", 115200);
    myPort.clear();
    myPort.bufferUntil(lf);
    size(600, 600, P3D);
} 

void draw()    { 
    

    background(0);
    lights();
        
    // Tweak the view of the rectangles
    int distance = 50;
    int x_rotation = 90;
    
    //Show combined data
    pushMatrix();
    translate((float)(width/2 + x * mag), (float)(height/2 + y * mag), -50);
    rotateX(radians(-x_rotation));
    rotateY(radians(-y_fil));
    noStroke();
    box(40, 20, 50);    
    popMatrix();
 
    textSize(24);
    String accStr = "(" + (int) x_acc + ", " + (int) y_acc + ")";
    String gyrStr = "(" + (int) x_gyr + ", " + (int) y_gyr + ")";
    String filStr = "(" + (int) x_fil + ", " + (int) y_fil + ")";
 

    fill(249, 250, 50);
    text("Gyroscope", (int) width/6.0 - 60, 25);
    text(gyrStr, (int) (width/6.0) - 40, 50);

    fill(56, 140, 206);
    text("Accelerometer", (int) width/2.0 - 50, 25);
    text(accStr, (int) (width/2.0) - 30, 50); 
    
    fill(83, 175, 93);
    text("Combination", (int) (5.0*width/6.0) - 40, 25);
    text(filStr, (int) (5.0*width/6.0) - 20, 50);

    double x_temp = x * Math.cos(Math.toRadians(-y_fil)) - y * Math.sin(Math.toRadians(-y_fil));
    double y_temp = x * Math.sin(Math.toRadians(-y_fil)) + y * Math.cos(Math.toRadians(-y_fil));

    x = x_temp;
    y = y_temp;
    mag += speed;
} 

void serialEvent(Serial p) {

    inString = (myPort.readString());
    
    try {
        String[] dataStrings = split(inString, '#');
        for (int i = 0; i < dataStrings.length; i++) {
            String type = dataStrings[i].substring(0, 4);
            String dataval = dataStrings[i].substring(4);
        if (type.equals("DEL:")) {
                dt = float(dataval);

                
            } else if (type.equals("ACC:")) {
                String data[] = split(dataval, ',');
                x_acc = float(data[0]);
                y_acc = float(data[1]);
                z_acc = float(data[2]);

            } else if (type.equals("GYR:")) {
                String data[] = split(dataval, ',');
                x_gyr = float(data[0]);
                y_gyr = float(data[1]);
                z_gyr = float(data[2]);
            } else if (type.equals("FIL:")) {
                String data[] = split(dataval, ',');
                x_fil = float(data[0]);
                y_fil = float(data[1]);
                z_fil = float(data[2]);
            }
        }
    } catch (Exception e) {
            println("Caught Exception");
    }
}
