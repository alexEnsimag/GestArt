/* --------------------------------------------------------------------------
 * SimpleOpenNI User3d Test
 * --------------------------------------------------------------------------
 * Processing Wrapper for the OpenNI/Kinect library
 * http://code.google.com/p/simple-openni
 * --------------------------------------------------------------------------
 * prog:  Max Rheiner / Interaction Design / zhdk / http://iad.zhdk.ch/
 * date:  02/16/2011 (m/d/y)
 * ----------------------------------------------------------------------------
 * this demos is at the moment only for 1 user, will be implemented later
 * ----------------------------------------------------------------------------
 */
 
import SimpleOpenNI.*;
import oscP5.*;
import netP5.*;

/* Connexion */
OscP5 oscP5;
NetAddress myRemoteLocation;

/* Instructions */
String text = "Calibre toi!";
int nbUsers;

boolean calibrate;
boolean bDisplayMessage;
int startTime;
int lastVisible = 0;
int user = 0;

final int DISPLAY_DURATION = 5000; // 1s

/* OpenNI */
SimpleOpenNI context;
float        zoomF =0.5f;
float        rotX = radians(180);  // by default rotate the hole scene 180deg around the x-axis, 
                                   // the data from openni comes upside down
float        rotY = radians(0);
boolean      autoCalib=true;

PVector      bodyCenter = new PVector();
PVector      bodyDir = new PVector();

PrintWriter output;

void setup()
{
  //this.frame.hide();
  //size(800,668,P3D);  // strange, get drawing error in the cameraFrustum if i use P3D, in opengl there is no problem
  size(300,100,P3D);
  context = new SimpleOpenNI(this);
   
  // disable mirror
  context.setMirror(false);

  // enable depthMap generation 
  if(context.enableDepth() == false)
  {
     println("Can't open the depthMap, maybe the camera is not connected!"); 
     exit();
     return;
  }

  // enable skeleton generation for all joints
  context.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);

  stroke(255,255,255);
  smooth();  
  perspective(radians(45),
              float(width)/float(height),
              10,150000);
              
               /* start oscP5, listening for incoming messages at port 12000 */
  oscP5 = new OscP5(this,12345);
  myRemoteLocation = new NetAddress("127.0.0.1",12345);
  //remoteLoc = new NetAddress("127.0.0.1",12370);
  
  output = createWriter("positions.txt");
  
  bDisplayMessage = true;
  startTime = millis();
 }

void draw()
{
  // update the cam
  context.update();
// draw the skeleton if it's available
  int[] userList = context.getUsers();
  for(int i=0;i<userList.length;i++)
  {
    if(context.isTrackingSkeleton(userList[i]))
      drawSkeleton(userList[i]);  
  }    
   
  background(0, 0, 0);
   if(bDisplayMessage)
    {
      fill(#FFAA88);
      
      text(text, width/4, height / 2);
      text("      ", width/4, height/2);
      // If the spent time is above the defined duration
      if (millis() - startTime > DISPLAY_DURATION) 
      {
        // Stop displaying the message, thus resume the ball moving
        bDisplayMessage = false;
        //this.frame.hide();
      }
    }
 
}

// draw the skeleton with the selected joints
void drawSkeleton(int userId)
{
  strokeWeight(3);

  // to get the 3d joint data
  drawLimb(userId, SimpleOpenNI.SKEL_HEAD, SimpleOpenNI.SKEL_NECK);

  drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_LEFT_SHOULDER);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_LEFT_ELBOW);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_ELBOW, SimpleOpenNI.SKEL_LEFT_HAND);

  drawLimb(userId, SimpleOpenNI.SKEL_NECK, SimpleOpenNI.SKEL_RIGHT_SHOULDER);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_RIGHT_ELBOW);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_ELBOW, SimpleOpenNI.SKEL_RIGHT_HAND);

  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_SHOULDER, SimpleOpenNI.SKEL_TORSO);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_SHOULDER, SimpleOpenNI.SKEL_TORSO);

  drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_LEFT_HIP);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_HIP, SimpleOpenNI.SKEL_LEFT_KNEE);
  drawLimb(userId, SimpleOpenNI.SKEL_LEFT_KNEE, SimpleOpenNI.SKEL_LEFT_FOOT);

  drawLimb(userId, SimpleOpenNI.SKEL_TORSO, SimpleOpenNI.SKEL_RIGHT_HIP);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_HIP, SimpleOpenNI.SKEL_RIGHT_KNEE);
  drawLimb(userId, SimpleOpenNI.SKEL_RIGHT_KNEE, SimpleOpenNI.SKEL_RIGHT_FOOT);  

  // draw body direction
  getBodyDirection(userId,bodyCenter,bodyDir);
  
  bodyDir.mult(200);  // 200mm length
  bodyDir.add(bodyCenter);
}

void drawLimb(int userId,int jointType1,int jointType2)
{
  PVector jointPos1 = new PVector();
  PVector jointPos2 = new PVector();
  float  confidence;
  
  // draw the joint position
  confidence = context.getJointPositionSkeleton(userId,jointType1,jointPos1);
  confidence = context.getJointPositionSkeleton(userId,jointType2,jointPos2);
  if(jointType1 == SimpleOpenNI.SKEL_HEAD) {
      sendJointMsg(jointType1, jointPos1);
        sendJointMsg(jointType1, jointPos1);
  }
  sendJointMsg(jointType2, jointPos2);
  sendJointMsg(jointType2, jointPos2);
}

// -----------------------------------------------------------------
// SimpleOpenNI user events

void onNewUser(int userId)
{
  nbUsers++;  
  println("onNewUser - userId: " + userId);
  println("  start pose detection");
  
  if(autoCalib)
    context.requestCalibrationSkeleton(userId,true);
  else    
    context.startPoseDetection("Psi",userId);
}

void onLostUser(int userId)
{
  nbUsers--;
  if(nbUsers == 0)  {
    drawLostUser(userId);  
  }
  println("onLostUser - userId: " + userId);
}

void onExitUser(int userId)
{
  drawLostUser(userId);
  println("onExitUser - userId: " + userId);
}

void onReEnterUser(int userId)
{
  drawDetectUser();
  println("onReEnterUser - userId: " + userId);
}


void onStartCalibration(int userId)
{
  println("onStartCalibration - userId: " + userId);
}

void onEndCalibration(int userId, boolean successfull)
{
  println("onEndCalibration - userId: " + userId + ", successfull: " + successfull);
  
  if (successfull) 
  { 
    drawDetectUser();
    user =  userId;
    println("  User calibrated !!!");
    context.startTrackingSkeleton(userId); 
  } 
  else 
  { 
    drawLostUser(userId);
    println("  Failed to calibrate user !!!");
    println("  Start pose detection");
    context.startPoseDetection("Psi",userId);
  }
}

void onStartPose(String pose,int userId)
{
  println("onStartdPose - userId: " + userId + ", pose: " + pose);
  println(" stop pose detection");
  
  context.stopPoseDetection(userId); 
  context.requestCalibrationSkeleton(userId, true);
 
}

void onEndPose(String pose,int userId)
{
  println("onEndPose - userId: " + userId + ", pose: " + pose);
}

// -----------------------------------------------------------------
// Keyboard events

void keyPressed()
{
  switch(key)
  {
  case ' ':
    context.setMirror(!context.mirror());
    break;
  }
    
  switch(keyCode)
  {
    case LEFT:
      rotY += 0.1f;
      break;
    case RIGHT:
      // zoom out
      rotY -= 0.1f;
      break;
    case UP:
      if(keyEvent.isShiftDown())
        zoomF += 0.01f;
      else
        rotX += 0.1f;
      break;
    case DOWN:
      if(keyEvent.isShiftDown())
      {
        zoomF -= 0.01f;
        if(zoomF < 0.01)
          zoomF = 0.01;
      }
      else
        rotX -= 0.1f;
      break;
  }
}

void getBodyDirection(int userId,PVector centerPoint,PVector dir)
{
  PVector jointL = new PVector();
  PVector jointH = new PVector();
  PVector jointR = new PVector();
  float  confidence;
  
  // draw the joint position
  confidence = context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_SHOULDER,jointL);
  confidence = context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_HEAD,jointH);
  confidence = context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_SHOULDER,jointR);
  
  // take the neck as the center point
  confidence = context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_NECK,centerPoint);
  
  /*  // manually calc the centerPoint
  PVector shoulderDist = PVector.sub(jointL,jointR);
  centerPoint.set(PVector.mult(shoulderDist,.5));
  centerPoint.add(jointR);
  */
  
  PVector up = new PVector();
  PVector left = new PVector();
  
  up.set(PVector.sub(jointH,centerPoint));
  left.set(PVector.sub(jointR,centerPoint));
  
  dir.set(up.cross(left));
  dir.normalize();
}

void sendJointMsg(int jointType,PVector pos) 
{
  OscMessage msg;
  if(jointType == SimpleOpenNI.SKEL_HEAD) {
    msg = new OscMessage("/head_pos_body");
  } else if(jointType == SimpleOpenNI.SKEL_LEFT_SHOULDER) {
    msg = new OscMessage("/leftshoulder_pos_body");
  } else if(jointType == SimpleOpenNI.SKEL_LEFT_ELBOW) {
    msg = new OscMessage("/leftelbow_pos_body");
  } else if(jointType == SimpleOpenNI.SKEL_RIGHT_SHOULDER) {
    msg = new OscMessage("/rightshoulder_pos_body");
  } else if(jointType == SimpleOpenNI.SKEL_RIGHT_ELBOW) {
    msg = new OscMessage("/rightelbow_pos_body");
  } else if(jointType == SimpleOpenNI.SKEL_TORSO) {
    msg = new OscMessage("/torso_pos_body");
  } else if(jointType == SimpleOpenNI.SKEL_LEFT_HIP) {
    msg = new OscMessage("/lefthip_pos_body");
  } else if(jointType == SimpleOpenNI.SKEL_LEFT_KNEE) {
    msg = new OscMessage("/leftknee_pos_body");
  } else if(jointType == SimpleOpenNI.SKEL_RIGHT_HIP) {
    msg = new OscMessage("/righthip_pos_body");
  } else if(jointType == SimpleOpenNI.SKEL_RIGHT_KNEE) {
        msg = new OscMessage("/rightknee_pos_body");
  }
  else if(jointType == SimpleOpenNI.SKEL_LEFT_HAND) {
        msg = new OscMessage("/lefthand_pos_body");
  } else if(jointType == SimpleOpenNI.SKEL_RIGHT_HAND) {
        msg = new OscMessage("/righthand_pos_body");
  } else {
    return;
  } 
  msg.add(pos.x);
  msg.add(pos.y);
  msg.add(pos.z);
  oscP5.send(msg, myRemoteLocation); 
  //oscP5.send(msg, remoteLoc);
}

void oscEvent(OscMessage theOscMessage) 
{  
  // get the first value as an integer
  float firstValue = theOscMessage.get(0).floatValue();
 
  // get the second value as a float  
  float secondValue = theOscMessage.get(1).floatValue();
 
  // get the third value as a string
  float thirdValue = theOscMessage.get(2).floatValue();
 
  // print out the message
  /*print("OSC Message Recieved: ");
  print(theOscMessage.addrPattern() + " ");
  println(firstValue + " " + secondValue + " " + thirdValue);*/
  
  // write into the file
  output.print(theOscMessage.addrPattern() + " ");
  output.println(firstValue + " " + secondValue + " " + thirdValue);
  
}

void drawMessage(String msg) {
    text = msg;
    startTime = millis();
    bDisplayMessage = true;
}

void drawDetectUser() {
  if(!calibrate) {
    calibrate = true;
    drawMessage("Fais l'activite !");
  }
}

void drawLostUser(int userId) {
  if(calibrate && userId == user) {
    calibrate = false;
    drawMessage("Calibre toi !");
  }
}
