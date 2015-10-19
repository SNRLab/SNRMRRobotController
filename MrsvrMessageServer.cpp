//====================================================================
//
// MRI guided robot control system
//
// Copyright (C) 2003-2005 by The University of Tokyo,
// All Right Reserved.
//
//====================================================================
// $RCSfile: MrsvrMessageServer.cpp,v $
// $Revision: 1.3 $ 
// $Author: junichi $
// $Date: 2006/01/20 03:15:48 $
//====================================================================


//====================================================================
// Description: 
//    Message interface for MRI guided robot control system.
//====================================================================

#include "MrsvrMessageServer.h"

#include <iostream>
#include <math.h>
#include <cstdlib>
#include <cstring>
#include <map>


std::map <int, string>    rowCoor;
std::map <int, string>    colCoor;

const char* MrsvrMessageServer::svrStatusStr[] = {
  "Sleeping",
  "Waiting",
  "Connected",
};

#define MSG_SVR_NUM_MODE  8

const char* MrsvrMessageServer::robotModeStr[] = {
  "STOP",
  "M.CLB",
  "A.CLB",
  "MOVETO",
  "PAUSE",
  "MANUAL",
  "REMOTE",
  "RESET",
};


//-------------------------------------------------- yuting
void GetRandomMatrix(igtl::Matrix4x4& matrix){
  float position[3];
  float orientation[4];
  
  // random position
  static float phi = 0.0;
  position[0] = 50.0 * cos(phi);
  position[1] = 50.0 * sin(phi);
  position[2] = 50.0 * cos(phi);
  phi = phi + 0.2;
  
  // random orientation
  static float theta = 0.0;
  orientation[0] = 0.0;
  orientation[1] = 0.6666666666*cos(theta);
  orientation[2] = 0.577350269189626;
  orientation[3] = 0.6666666666*sin(theta);
  theta = theta + 0.1;
  
  // igtl::Matrix4x4 matrix
  igtl::QuaternionToMatrix(orientation, matrix);
  
  matrix[0][3] = position[0];
  matrix[1][3] = position[1];
  matrix[2][3] = position[2];
  
  igtl::PrintMatrix(matrix);
}

void printTargetMatrix(igtl::Matrix4x4& matrix){
  igtl::PrintMatrix(matrix);
}
//-------------------------------------------------- end, yuting

MrsvrMessageServer::MrsvrMessageServer(int port) : MrsvrThread()
{
  this->port = port;
  this->gapDist = 8.95;
  init();
  pthread_mutex_init(&mtxCommand, NULL);


  //TODO: Change the rowCoor&colCoor according to which template chosen (oldSmartTemplate or newSmartTemplate)  
  /*
  rowCoor[1] = "A";
  rowCoor[2] = "B";
  rowCoor[3] = "C";
  rowCoor[4] = "D";
  rowCoor[5] = "E";
  rowCoor[6] = "F";
  rowCoor[7] = "G";
  rowCoor[8] = "H";
  rowCoor[9] = "I";
  rowCoor[10] = "J";
  rowCoor[11] = "K";
  rowCoor[12] = "L";
  rowCoor[13] = "M";
  rowCoor[14] = "N";


  colCoor[-7] = "-7";
  colCoor[-6] = "-6";
  colCoor[-5] = "-5";
  colCoor[-4] = "-4";
  colCoor[-3] = "-3";
  colCoor[-2] = "-2";
  colCoor[-1] = "-1";
  colCoor[0] = "0";
  colCoor[1] = "1";
  colCoor[2] = "2";
  colCoor[3] = "3";
  colCoor[4] = "4";
  colCoor[5] = "5";
  colCoor[6] = "6";
  colCoor[7] = "7";
  */
}


MrsvrMessageServer::~MrsvrMessageServer()
{
  
}

void MrsvrMessageServer::init()
{
  currentPos  = new MrsvrRASWriter(SHM_RAS_CURRENT);
  setPoint    = new MrsvrRASWriter(SHM_RAS_SETPOINT);
  robotStatus = new MrsvrStatusReader(SHM_STATUS);

  //masterBufferedFD = NULL;
  fSetTargetMatrix      = false;
  fSetCalibrationMatrix = false;
  nextRobotMode    = -1;


  //-------------------------------------------------- yuting
  fZFrameTransform = false;
  fTarget = false;
  fTargetCell = false;
  //-------------------------------------------------- end, yuting

  this->connectionStatus =  SVR_STOP;
  this->fRunServer = 1;
}


void MrsvrMessageServer::process()
{
  this->fRunServer = 1;
  
  igtl::ServerSocket::Pointer serverSocket;
  serverSocket = igtl::ServerSocket::New();
  int r = serverSocket->CreateServer(port);

  // Change the status to "WAIT"
  this->connectionStatus = SVR_WAIT;

  if (r < 0)
    {
#ifdef DEBUG    
    perror("MrsvrMessageServer::process(): ERROR: Cannot create a server socket.");
#endif // DEBUG
    return;
    }

  //igtl::Socket::Pointer socket;

  while (this->fRunServer == 1) {
    //------------------------------------------------------------
    // Waiting for Connection
    socket = serverSocket->WaitForConnection(1000);

    if (socket.IsNotNull()) {// if client connected
#ifdef DEBUG_MRSVR_MESSAGE_SERVER
      printf("Master process connected to the robot module.\n");
      fflush(stdout);
#endif

      // Change the status to "CONNECTED"
      this->connectionStatus = SVR_CONNECTED;
      
      // Create a message buffer to receive header
      igtl::MessageHeader::Pointer headerMsg;
      headerMsg = igtl::MessageHeader::New();
      
      //------------------------------------------------------------
      // loop
      while (this->fRunServer == 1) {       
        // Initialize receive buffer
        headerMsg->InitPack();
        
        // Receive generic header from the socket
        int r = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
        if (r == 0) {
          //------------------------------------------------------------
          // Close connection (The example code never reaches to this section ...)
          break;
        }
        if (r != headerMsg->GetPackSize()) {
          continue;
        }

        // Deserialize the header
        headerMsg->Unpack();

	//-------------------- yuting
	std::cerr << "device type: " << headerMsg->GetDeviceType() << ", device name: " << headerMsg->GetDeviceName() << std::endl;
 
        // Check data type and receive data body
        if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0){
          onRcvMsgMaster(socket, headerMsg);
        } 

	else if (strcmp(headerMsg->GetDeviceType(), "POINT") == 0){
	  onRcvPointMsg(socket, headerMsg);
	  feedBackInfo();
	}

	else if ((strcmp(headerMsg->GetDeviceType(), "STRING") == 0) && (strcmp(headerMsg->GetDeviceName(), "TARGETCell") == 0)){
	  onRcvStringMsgTargetCell(socket, headerMsg);
	}

	else if ((strcmp(headerMsg->GetDeviceType(), "STRING") == 0) && (strcmp(headerMsg->GetDeviceName(), "SelectPath") == 0)){
	  onRcvStringMsgSelectPath(socket, headerMsg);
	}
	
	else{
          // if the data type is unknown, skip reading.
          std::cerr << "Receiving : " << headerMsg->GetDeviceType() << std::endl;
          socket->Skip(headerMsg->GetBodySizeToRead(), 0);
        }
      }      

      // Change the status to "WAIT"
      socket->CloseSocket();
      this->connectionStatus = SVR_WAIT;
    }
  }
  this->connectionStatus = SVR_STOP;
}


void MrsvrMessageServer::stop()
{
  this->fRunServer = 0;

  MrsvrThread::stop();
  /*
  fd_set readfds, testfds;
  for (int i = 0; i < FD_SETSIZE; i ++) {
    if (FD_ISSET(i, &testfds)) {
      close(i);
      FD_CLR(i, &readfds);
      if (i == masterSockFD) {
        masterSockFD = -1;
      }
    }
  }
  */
  init();
}


int MrsvrMessageServer::onRcvMsgMaster(igtl::Socket::Pointer& socket, igtl::MessageHeader::Pointer& header)
{

#ifdef DEBUG_MRSVR_MESSAGE_SERVER
  fprintf(stderr, "MrsvrMessageServer::onRcvMsgMaster():Receiving TRANSFORM data type.\n");
#endif
  
  // Create a message buffer to receive transform data
  igtl::TransformMessage::Pointer transMsg;
  transMsg = igtl::TransformMessage::New();
  transMsg->SetMessageHeader(header);
  transMsg->AllocatePack();
  
  // Receive transform data from the socket
  socket->Receive(transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize());
  
  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = transMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) { // if CRC check is OK
    // Retrive the transform data
    igtl::Matrix4x4 matrix;
    transMsg->GetMatrix(matrix);
    // Check the device name in the OpenIGTLink header
    if (strcmp(transMsg->GetDeviceName(), "TARGET") == 0 ||
        strcmp(transMsg->GetDeviceName(), "ProstateNavRobotTarg") == 0){

      setTargetMatrix(matrix);      
      printTargetMatrix(matrix);
      
      //if (result == TARGET_ACCEPTED) {
      //} else if (result == TARGET_OUT_OF_RANGE) {
      //}

    } else if (strcmp(transMsg->GetDeviceName(), "ZFrameTransform") == 0){
      
      setCalibrationMatrix(matrix);
      //-------------------------------------------------- yuting
      fZFrameTransform = true;
      //-------------------------------------------------- end, yuting
    }
    
    return 1;
  }
  return 0;
}


//-------------------------------------------------- yuting
int MrsvrMessageServer::onRcvPointMsg(igtl::Socket::Pointer& socket, igtl::MessageHeader::Pointer& header)
{
#ifdef DEBUG_MRSVR_MESSAGE_SERVER
  fprintf(stderr, "MrsvrMessageServer::onRcvMarkupMsg():Receiving POINT data type.\n");
#endif
  
  igtl::PointMessage::Pointer pointMsg;
  pointMsg = igtl::PointMessage::New();
  pointMsg->SetMessageHeader(header);
  pointMsg->AllocatePack();
  
  int rcv = socket->Receive(pointMsg->GetPackBodyPointer(), pointMsg->GetPackBodySize());
  
  // Deserialize the transform data
  // If you want to skip CRC check, call Unpack() without argument.
  int c = pointMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) { // if CRC check is OK
    // Check the device name in the OpenIGTLink header
    if (strcmp(pointMsg->GetDeviceName(), "F") == 0){
      int nOfPointElement = pointMsg->GetNumberOfPointElement();
      std::cerr << "number of point element: " << nOfPointElement << std::endl;
      // get the point element
      for (int i = 0; i < nOfPointElement; i++)
	{
	  igtl::PointElement::Pointer pointElement;
	  pointMsg->GetPointElement(i, pointElement);
	
	  igtlUint8 rgba[4];
	  pointElement->GetRGBA(rgba);

	  igtlFloat32 pos[3];
	  pointElement->GetPosition(pos);
	  
	  std::cerr << "Name: " << pointElement->GetName() << std::endl;
          //std::cerr << " Position  : ( " << std::fixed << pos[0] << ", " << pos[1] << ", " << pos[2] << " )" << std::endl;
	  std::cerr << " Position  : ( "  << pos[0] << ", " << pos[1] << ", " << pos[2] << " )" << std::endl;
	}
      fTarget = true;
    }
  }
  return 1;
}



int MrsvrMessageServer::onRcvStringMsgTargetCell(igtl::Socket::Pointer& socket, igtl::MessageHeader::Pointer& header)
{
#ifdef DEBUG_MRSVR_MESSAGE_SERVER
  fprintf(stderr, "MrsvrMessageServer::onRcvStringMsgTargetCell():Receiving STRING data type.\n");
#endif
  
  igtl::StringMessage::Pointer stringMsg;
  stringMsg = igtl::StringMessage::New();
  stringMsg->SetMessageHeader(header);
  stringMsg->AllocatePack();
  
  int rcv = socket->Receive(stringMsg->GetPackBodyPointer(), stringMsg->GetPackBodySize());
  
  int c = stringMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) { 
    if (strcmp(stringMsg->GetDeviceName(), "TARGETCell") == 0){
      //std::cerr << "String: " << stringMsg->GetString() << std::endl;
      
      char *cStringPos = new char[std::strlen(stringMsg->GetString())+1];
      std::strcpy(cStringPos, stringMsg->GetString());
      stringToken = std::strtok(cStringPos, ",");
       
      for (int i = 0; i < 13; i++)
      {
	//std::cerr << stringToken << std::endl;	
	stringPos[i] = std::atof(stringToken);
	//	std::cerr << "stringPos: " << stringPos[i] << ", i: " << i << std::endl;
	stringToken = strtok(NULL,",");
      }

      for (int i = 0; i < 3; i++)
      {
	targetRAS[i] = stringPos[i];
	templatePlane[i] = stringPos[i+3];
	holeRAS[i] = stringPos[i+6];
      }
      
      depthAngulated = stringPos[9];
      degreeAngulated = stringPos[10];
      rowAngulated = int(stringPos[11]);
      colAngulated = int(stringPos[12]);

      std::cerr << "targetRAS: " << targetRAS[0] << ", " << targetRAS[1] << ", " << targetRAS[2] << std::endl;
      std::cerr << "tempplatePlane: " << templatePlane[0] << ", " << templatePlane[1] << ", " << templatePlane[2] << std::endl;
      std::cerr << "holeRAS: " << holeRAS[0] << ", " << holeRAS[1] << ", " << holeRAS[2] << std::endl;
      std::cerr << "depth: " << depthAngulated << std::endl;
      std::cerr << "degree: " << degreeAngulated << std::endl;
      //std::cerr << "(x,y): "<< rowAngulated << ", " << colAngulated << std::endl; 
      std::cerr << "x = " << rowCoor[rowAngulated] << std::endl;
      std::cerr << "y = " << colCoor[colAngulated] << std::endl;

      // insertionLocation = determineInsertionLocation(targetRAS, templatePlane, holeRAS, gapDist);
      //std::cerr << "insertionLocation: " << insertionLocation[0] << ", " << insertionLocation[1] << ", " << insertionLocation[2] << std::endl;
      insertionLocation2 = determineXYZ(targetRAS, holeRAS, gapDist);
      std::cerr << "insertionLocation2: " << insertionLocation2[0] << ", " << insertionLocation2[1] << ", " << insertionLocation2[2] << std::endl;

      std::cerr << "!!!!!! TEST" << std::endl;
      float deltaX = gapDist*tan(stringPos[10] * PI/180.0);
      float theoX =  sqrt(pow((insertionLocation2[0]-holeRAS[0]),2) + pow((insertionLocation2[1]-holeRAS[1]),2));
      float ratioX = (insertionLocation2[0]-holeRAS[0]) / (float)(holeRAS[0]-targetRAS[0]);
      float ratioY = (insertionLocation2[1]-holeRAS[1]) / (float)(holeRAS[1]-targetRAS[1]);
      float ratioZ = (insertionLocation2[2]-holeRAS[2]) / (float)(holeRAS[2]-targetRAS[2]);
      std::cerr << "deltaX: " << deltaX << std::endl;
      std::cerr << "theoX: " << theoX << std::endl;
      std::cerr << "ratioX: " << ratioX << std::endl;
      std::cerr << "ratioY: " << ratioY << std::endl;
      std::cerr << "ratioZ: " << ratioZ << std::endl;

      
      pthread_mutex_lock(&mtxCommand);
        targetMatrix[0][0] = 1.0;
        targetMatrix[0][1] = 0.0;
        targetMatrix[0][2] = 0.0;
        targetMatrix[0][3] = insertionLocation2[0];
        targetMatrix[1][0] = 0.0;
        targetMatrix[1][1] = 1.0;
        targetMatrix[1][2] = 0.0;
        targetMatrix[1][3] = insertionLocation2[1];
        targetMatrix[2][0] = 0.0;
        targetMatrix[2][1] = 0.0;
        targetMatrix[2][2] = 1.0;
        targetMatrix[2][3] = insertionLocation2[2];
        targetMatrix[3][0] = 0.0;
        targetMatrix[3][1] = 0.0;
        targetMatrix[3][2] = 0.0;
        targetMatrix[3][3] = 1.0;
      pthread_mutex_unlock(&mtxCommand);

      // delete[] insertionLocation;
             
      fTargetCell = true;
      
      if (fTargetCell == true){
	feedBackInfoTargetCell(cStringPos);
      }
    } 
    
  }
  return 1;
}



//-------------------------------------------------- sep25, yuting
float* MrsvrMessageServer::determineXYZ(float targetRAS[3], float holeRAS[3], float gapDist)
{
  float *returnLocation = new float[3];
  float x, y, z, con;

  z = holeRAS[2] - gapDist;
  con = (z-holeRAS[2]) / (float)(holeRAS[2]-targetRAS[2]);

  std::cerr << "con: " << con << std::endl;

  x = con*(holeRAS[0]-targetRAS[0]) + holeRAS[0];
  y = con*(holeRAS[1]-targetRAS[1]) + holeRAS[1];

  returnLocation[0] = x;
  returnLocation[1] = y;
  returnLocation[2] = z;
 
  return returnLocation; 
}
//-------------------------------------------------- end, yuting



int MrsvrMessageServer::onRcvStringMsgSelectPath(igtl::Socket::Pointer& socket, igtl::MessageHeader::Pointer& header)
{
#ifdef DEBUG_MRSVR_MESSAGE_SERVER
  fprintf(stderr, "MrsvrMessageServer::onRcvStringMsgSelectPath():Receiving STRING data type.\n");
#endif
  
  igtl::StringMessage::Pointer stringMsg;
  stringMsg = igtl::StringMessage::New();
  stringMsg->SetMessageHeader(header);
  stringMsg->AllocatePack();
  
  int rcv = socket->Receive(stringMsg->GetPackBodyPointer(), stringMsg->GetPackBodySize());
  
  int c = stringMsg->Unpack(1);
  
  if (c & igtl::MessageHeader::UNPACK_BODY) { 
    if (strcmp(stringMsg->GetDeviceName(), "SelectPath") == 0){
      
      char *cStringPath = new char[std::strlen(stringMsg->GetString())+1];
      std::strcpy(cStringPath, stringMsg->GetString());
      stringTokenPath = std::strtok(cStringPath, ",");

      for(int i = 0; i < 2; i++)
      {
	stringPath[i] = std::atof(stringTokenPath);
	std::cerr << "stringPath: " << stringPath[i] << ", i: " << i <<std::endl;
	stringTokenPath = strtok(NULL,",");
      }

    } 
    
  }
  return 1;
}
//-------------------------------------------------- end, yuting


//-------------------------------------------------- yuting
int MrsvrMessageServer::feedBackInfoRegist(char* infoRegistTime){
  std::cerr << "registtime: " << infoRegistTime << std::endl;
  igtl::StringMessage::Pointer feedRegistTimeMsg;
  feedRegistTimeMsg = igtl::StringMessage::New();
  feedRegistTimeMsg->SetDeviceName("feedInfoRegistTime");
  feedRegistTimeMsg->SetString(infoRegistTime);
  feedRegistTimeMsg->Pack();
  socket->Send(feedRegistTimeMsg->GetPackPointer(), feedRegistTimeMsg->GetPackSize());
}


int MrsvrMessageServer::feedBackInfoTargetCell(char* cStringPos){
  std::cerr << "stringPos: " << cStringPos << std::endl;
  igtl::StringMessage::Pointer feedTargetCellMsg;
  feedTargetCellMsg = igtl::StringMessage::New();
  feedTargetCellMsg->SetDeviceName("feedTargetCell");
  feedTargetCellMsg->SetString(cStringPos);
  feedTargetCellMsg->Pack();
  socket->Send(feedTargetCellMsg->GetPackPointer(), feedTargetCellMsg->GetPackSize());
}


int MrsvrMessageServer::feedBackInfo()
{
  // check connect
  if (this->connectionStatus == SVR_CONNECTED) {
    igtl::StringMessage::Pointer feedStatusMsg;
    feedStatusMsg = igtl::StringMessage::New();
    feedStatusMsg->SetDeviceName("feedStatus");
    feedStatusMsg->SetString("Connect");
    feedStatusMsg->Pack();
    socket->Send(feedStatusMsg->GetPackPointer(), feedStatusMsg->GetPackSize());

    if (fZFrameTransform == true){  // feedback ZFrameTransform
      igtl::StringMessage::Pointer feedMsg;
      feedMsg = igtl::StringMessage::New();
      feedMsg->SetDeviceName("feedZFrame");
      feedMsg->SetString("receive ZFrame message!!!!!");
      feedMsg->Pack();
      socket->Send(feedMsg->GetPackPointer(), feedMsg->GetPackSize());
      std::cerr << "feedZFrame" << std::endl;
      fZFrameTransform = false;
    }

    else if (fTarget == true) {  // feedback Target
      igtl::StringMessage::Pointer feedMsg;
      igtl::TimeStamp::Pointer ts;
      ts = igtl::TimeStamp::New();
      feedMsg = igtl::StringMessage::New();
      feedMsg->SetDeviceName("feedTarget");
      feedMsg->SetString("receive Target message!!!!!");
      feedMsg->SetTimeStamp(ts);
      feedMsg->Pack();
      socket->Send(feedMsg->GetPackPointer(), feedMsg->GetPackSize());
      std::cerr << "feedTarget" << std::endl;
      fTarget = false;
    }
      
//      igtl::TransformMessage::Pointer feedMsg;
//      feedMsg = igtl::TransformMessage::New();
//      igtl::TimeStamp::Pointer ts;
//      ts = igtl::TimeStamp::New();
//      igtl::Matrix4x4 rcv;
//      GetRandomMatrix(rcv);
//      feedMsg->SetDeviceName("feedTarget");    
//      feedMsg->SetMatrix(rcv);
//      feedMsg->SetTimeStamp(ts);
//      feedMsg->Pack();  
//      socket->Send(feedMsg->GetPackPointer(), feedMsg->GetPackSize());
//      std::cerr << "feedTarget" << std::endl;
//      fTarget = false;	
//
 
  } 
  else if (this->connectionStatus == SVR_WAIT) {
  }  //end "this" if

}
//-------------------------------------------------- end, yuting


int MrsvrMessageServer::getSvrStatus()
{
  return this->connectionStatus;
}


const char* MrsvrMessageServer::getSvrStatusStr()
{
  return svrStatusStr[this->connectionStatus];
}


bool MrsvrMessageServer::getTargetMatrix(Matrix4x4& matrix)
{
  pthread_mutex_lock(&mtxCommand);  
  if (fSetTargetMatrix == false) {
    pthread_mutex_unlock(&mtxCommand);  
    return false;
  }
  fSetTargetMatrix = false;
  for (int i = 0; i < 4; i ++) {
    for (int j = 0; j < 4; j ++) { 
      matrix[i][j] = targetMatrix[i][j];
    }
  }
  pthread_mutex_unlock(&mtxCommand);  
  return true;
}


bool MrsvrMessageServer::getCalibrationMatrix(Matrix4x4& matrix)
{
  pthread_mutex_lock(&mtxCommand);  
  if (fSetCalibrationMatrix == false) {
    pthread_mutex_unlock(&mtxCommand);  
    return false;
  }
  fSetCalibrationMatrix = false;
  for (int i = 0; i < 4; i ++) {
    for (int j = 0; j < 4; j ++) { 
      matrix[i][j] = calibrationMatrix[i][j];
    }
  }
  pthread_mutex_unlock(&mtxCommand);  
  return true;
}


bool MrsvrMessageServer::getMode(int* next)
{
  bool r;
  *next = -1;
  pthread_mutex_lock(&mtxCommand);  
  if (nextRobotMode < 0) {
    r = false;
  } else {
    r = true;
    *next = nextRobotMode;
  }
  pthread_mutex_unlock(&mtxCommand);  
  return r;
}


int MrsvrMessageServer::sendCurrentPosition(igtl::Matrix4x4& current)
{
  if (this->connectionStatus == SVR_CONNECTED)
    {
    igtl::TransformMessage::Pointer transMsg;
    transMsg = igtl::TransformMessage::New();
    transMsg->SetDeviceName("CURRENT");
    igtl::TimeStamp::Pointer ts;
    ts = igtl::TimeStamp::New();
    transMsg->SetMatrix(current);
    transMsg->SetTimeStamp(ts);
    transMsg->Pack();
    socket->Send(transMsg->GetPackPointer(), transMsg->GetPackSize());
    }
}

int MrsvrMessageServer::setTargetMatrix(igtl::Matrix4x4& matrix)
{
  pthread_mutex_lock(&mtxCommand);  
  for (int j = 0; j < 4; j ++) {
    for (int i = 0; i < 4; i ++) {
      targetMatrix[i][j] = matrix[i][j];
    }
  }
  fSetTargetMatrix = true;
  pthread_mutex_unlock(&mtxCommand);
  //
  // The received target should be validated based on
  // physical condition. This will be implemented in future.
  //
  return TARGET_ACCEPTED;
}


int MrsvrMessageServer::setCalibrationMatrix(igtl::Matrix4x4& matrix)
{
  pthread_mutex_lock(&mtxCommand);  
  for (int i = 0; i < 4; i ++) {
    for (int j = 0; j < 4; j ++) {
      calibrationMatrix[i][j] = matrix[i][j];
    }
  }
  fSetCalibrationMatrix = true;
  pthread_mutex_unlock(&mtxCommand);
  //
  // The received target should be validated based on
  // physical condition. This will be implemented in future.
  //
  return TARGET_ACCEPTED;
}


int MrsvrMessageServer::setMode(const char* param)
{
  int mode = -1;
  pthread_mutex_lock(&mtxCommand);
  for (int i = 0; i < MSG_SVR_NUM_MODE; i ++) {
    //fprintf(stderr, "MrsvrMessageServer::setMode(): compare %s vs %s",
    //MrsvrMessageServer::robotModeStr[i], param);
    if (strncmp(MrsvrMessageServer::robotModeStr[i], param, 
                strlen(MrsvrMessageServer::robotModeStr[i])) == 0) {
      mode = i;
      break;
    }
  }
  if (mode >= 0) {
    nextRobotMode = mode;
  }

  pthread_mutex_unlock(&mtxCommand);  

  return mode;
}


void MrsvrMessageServer::getRobotStatus(int* mode, int* outrange, int* lock)
{
  *mode     = robotStatus->getMode();
  *outrange = 0;
  for (int i = 0; i < NUM_ENCODERS; i ++) {
    if (robotStatus->getOutOfRange(i) != 0) {
      *outrange = 1;
    }
  }
  for (int i = 0; i < NUM_ACTUATORS; i ++) {
    if (robotStatus->getActuatorLockStatus(i)) {
      lock[i] = 1;
    } else {
      lock[i] = 0;
    }
  }
}


// changed by yuting
void MrsvrMessageServer::setOldTemplate(){
  std::cerr << "setOldTemplate()" << std::endl;

  rowCoor[1] = "A";
  rowCoor[2] = "B";
  rowCoor[3] = "C";
  rowCoor[4] = "D";
  rowCoor[5] = "E";
  rowCoor[6] = "F";
  rowCoor[7] = "G";
  rowCoor[8] = "H";
  rowCoor[9] = "I";
  rowCoor[10] = "J";
  rowCoor[11] = "K";
  rowCoor[12] = "L";
  rowCoor[13] = "M";
  rowCoor[14] = "N";

  colCoor[-7] = "-7";
  colCoor[-6] = "-6";
  colCoor[-5] = "-5";
  colCoor[-4] = "-4";
  colCoor[-3] = "-3";
  colCoor[-2] = "-2";
  colCoor[-1] = "-1";
  colCoor[0] = "0";
  colCoor[1] = "1";
  colCoor[2] = "2";
  colCoor[3] = "3";
  colCoor[4] = "4";
  colCoor[5] = "5";
  colCoor[6] = "6";
  colCoor[7] = "7";
}


void MrsvrMessageServer::setNewTemplate(){
  std::cerr << "setNewTemplate()" << std::endl;

  rowCoor[1] = "A";
  rowCoor[2] = "B";
  rowCoor[3] = "C";
  rowCoor[4] = "D";
  rowCoor[5] = "E";
  rowCoor[6] = "F";
  rowCoor[7] = "G";
  rowCoor[8] = "H";
  rowCoor[9] = "I";

  colCoor[1] = "-3";
  colCoor[2] = "-2";
  colCoor[3] = "-1";
  colCoor[4] = "0";
  colCoor[5] = "1";
  colCoor[6] = "2";
  colCoor[7] = "3";
}
