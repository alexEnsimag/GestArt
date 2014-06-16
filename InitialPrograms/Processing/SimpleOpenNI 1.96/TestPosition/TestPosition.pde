/* --------------------------------------------------------------------------
 * SimpleOpenNI User3d Test
 * --------------------------------------------------------------------------
 * Processing Wrapper for the OpenNI/Kinect 2 library
 * http://code.google.com/p/simple-openni
 * --------------------------------------------------------------------------
 * prog:  Max Rheiner / Interaction Design / Zhdk / http://iad.zhdk.ch/
 * date:  12/12/2012 (m/d/y)
 * ----------------------------------------------------------------------------
 */
 
import SimpleOpenNI.*;

import oscP5.*;
import netP5.*;

OscP5 oscP5;
NetAddress myRemoteLocation;

String text = "Calibre toi!";
int nbUsers;

boolean calibrate;
boolean bDisplayMessage;
int startTime;
int lastVisible = -1;
int cptVisible = 0;

final int DISPLAY_DURATION = 5000; // 1s


SimpleOpenNI context;
float        zoomF =0.5f;
float        rotX = radians(180);  // by default rotate the hole scene 180deg around the x-axis, 
                                   // the data from openni comes upside down
float        rotY = radians(0);
boolean      autoCalib=true;

PVector      bodyCenter = new PVector();
PVector      bodyDir = new PVector();
PVector      com = new PVector();                                   
PVector      com2d = new PVector();   
PrintWriter output;
color[]       userClr = new color[]{ color(255,0,0),
                                     color(0,255,0),
                                     color(0,0,255),
                                     color(255,255,0),
                                     color(255,0,255),
                                     color(0,255,255)
                                   };

void setup()
{
  //size(1024,768,P3D);  // strange, get drawing error in the cameraFrustum if i use P3D, in opengl there is no problem
  size(200, 50, P3D);
  context = new SimpleOpenNI(this);
  if(context.isInit() == false)
  {
     println("Can't init SimpleOpenNI, maybe the camera is not connected!"); 
     exit();
     return;  
  }

  // disable mirror
  context.setMirror(false);

  // enable depthMap generation 
  context.enableDepth();

  // enable skeleton generation for all joints
  context.enableUser();

  stroke(255,255,255);
  smooth();  
  perspective(radians(45),
              float(width)/float(height),
              10,150000);
              
 // create file
 output = createWriter("positions.txt");
 
  /* start oscP5, listening for incoming messages at port 12000 */
  oscP5 = new OscP5(this,12345);
  myRemoteLocation = new NetAddress("127.0.0.1",12345);
  
    bDisplayMessage = true;
  startTime = millis();
  
 }

void draw()
{
  // update the cam
  context.update();

  /*background(0,0,0);
  
  // set the scene pos
  translate(width/2, height/2, 0);
  rotateX(rotX);
  rotateY(rotY);
  scale(zoomF);
  
  int[]   depthMap = context.depthMap();
  int[]   userMap = context.userMap();
  int     steps   = 3;  // to speed up the drawing, draw every third point
  int     index;
  PVector realWorldPoint;
 
  translate(0,0,-1000);  // set the rotation center of the scene 1000 infront of the camera

  // draw the pointcloud
  beginShape(POINTS);
  for(int y=0;y < context.depthHeight();y+=steps)
  {
    for(int x=0;x < context.depthWidth();x+=steps)
    {
      index = x + y * context.depthWidth();
      if(depthMap[index] > 0)
      { 
        // draw the projected point
        realWorldPoint = context.depthMapRealWorld()[index];
        if(userMap[index] == 0)
          stroke(100); 
        else
          stroke(userClr[ (userMap[index] - 1) % userClr.length ]);        
        
        point(realWorldPoint.x,realWorldPoint.y,realWorldPoint.z);
      }
    }
  } 
  endShape();*/
  
  // draw the skeleton if it's available
  int[] userList = context.getUsers();
  for(int i=0;i<userList.length;i++)
  {
    if(context.isTrackingSkeleton(userList[i]))
      drawSkeleton(userList[i]);
    
    // draw the center of mass
    /*if(context.getCoM(userList[i],com))
    {
      stroke(100,255,0);
      strokeWeight(1);
      beginShape(LINES);
        vertex(com.x - 15,com.y,com.z);
        vertex(com.x + 15,com.y,com.z);
        
        vertex(com.x,com.y - 15,com.z);
        vertex(com.x,com.y + 15,com.z);

        vertex(com.x,com.y,com.z - 15);
        vertex(com.x,com.y,com.z + 15);
      endShape();
      
      fill(0,255,100);
      text(Integer.toString(userList[i]),com.x,com.y,com.z);
    }  */    
  }    
 
  // draw the kinect cam
  //context.drawCamFrustum();
  if(lastVisible == cptVisible) {
    text = "Calibre toi !";
    calibrate = false;
  }
  lastVisible = cptVisible;
  
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
  
  /*stroke(255,200,200);
  line(bodyCenter.x,bodyCenter.y,bodyCenter.z,
       bodyDir.x ,bodyDir.y,bodyDir.z);

  strokeWeight(1);*/
 
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
  /*stroke(255,0,0,confidence * 200 + 55);
  line(jointPos1.x,jointPos1.y,jointPos1.z,
       jointPos2.x,jointPos2.y,jointPos2.z);
  
  drawJointOrientation(userId,jointType1,jointPos1,50);*/
}

void drawJointOrientation(int userId,int jointType,PVector pos,float length)
{
  // draw the joint orientation  
  PMatrix3D  orientation = new PMatrix3D();
  float confidence = context.getJointOrientationSkeleton(userId,jointType,orientation);
  if(confidence < 0.001f) 
    // nothing to draw, orientation data is useless
    return;
    
  pushMatrix();
    translate(pos.x,pos.y,pos.z);
    
    // set the local coordsys
    applyMatrix(orientation);
    
    // coordsys lines are 100mm long
    // x - r
    stroke(255,0,0,confidence * 200 + 55);
    line(0,0,0,
         length,0,0);
    // y - g
    stroke(0,255,0,confidence * 200 + 55);
    line(0,0,0,
         0,length,0);
    // z - b    
    stroke(0,0,255,confidence * 200 + 55);
    line(0,0,0,
         0,0,length);
  popMatrix();
}

// -----------------------------------------------------------------
// SimpleOpenNI user events

void onNewUser(SimpleOpenNI curContext,int userId)
{
  if(nbUsers == 0) {
    calibrate = true;
    this.frame.show();
    startTime = millis();
    bDisplayMessage = true;
    text = "Fais l'activité !";  
  }
  nbUsers++;
  
  println("onNewUser - userId: " + userId);
  println("\tstart tracking skeleton");
  
  context.startTrackingSkeleton(userId);
}

void onLostUser(SimpleOpenNI curContext,int userId)
{
  nbUsers--;
  if(nbUsers == 0)  {
    calibrate = false;
    this.frame.show();
    text = "Calibre toi !";
    startTime = millis();
    bDisplayMessage = true;
    // this.frame.hide();
  }
  println("onLostUser - userId: " + userId);
}

void onVisibleUser(SimpleOpenNI curContext,int userId)
{
  if(!calibrate) {
    bDisplayMessage = true;
    startTime = millis();
    text = "Fais l'activité !";
    calibrate = true;
  }
  cptVisible++;
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
  
  PVector up = PVector.sub(jointH,centerPoint);
  PVector left = PVector.sub(jointR,centerPoint);
    
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
  print("OSC Message Recieved: ");
  print(theOscMessage.addrPattern() + " ");
  println(firstValue + " " + secondValue + " " + thirdValue);
  
  // write into the file
  output.print(theOscMessage.addrPattern() + " ");
  output.println(firstValue + " " + secondValue + " " + thirdValue);
  
}
/*
void update (){
  confidence = context.getJointPositionSkeleton(userId,jointType1,jointPos1);
  confidence = context.getJointPositionSkeleton(userId,jointType2,jointPos2);

 if(jointType1 == SimpleOpenNI.SKEL_HEAD) {
      sendJointMsg(jointType1, jointPos1);
    }
    sendJointMsg(jointType2, jointPos2);
  
}*/
