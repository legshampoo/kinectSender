#pragma once

#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxOpenCv.h"
#include "ofxOsc.h"
#include "pbNetwork.h"
#include "ofxXmlSettings.h"
#include "ofxGui.h"

class ofApp : public ofBaseApp{

public:
    void setup();
    void update();
    void draw();

    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);

    //utils
    void buildPointCloud(const ofFloatPixels & depth);
    ofVec3f depthToPointCloudPos(int x, int y, float depthValue);
    string getHostname();
    void tcpCreateConnection();
    void camSetup();

    ofMesh constructPointCloud(const ofFloatPixels & depth);
    ofFbo pclToNetworkFbo(const ofMesh & pcl);
    void sendData(const ofFbo & pointCloudFbo);
    void tcpSendImage(const ofFbo & fbo);
    void saveBackground();
    void loadBackground();
    void loadGui();
    ofMesh removeBackground(const ofMesh & pcl);
    ofMesh removeBackgroundDist(const ofMesh & pcl);
    void saveBackgroundAvg();
    void drawBackgroundPCL();
    void bgCamSetup();
    void createFloorCube();
    ofMesh sphericalSubtraction(const ofMesh & pcl);
    void updateCam();
    ofMesh floorPlaneSubtraction(const ofMesh & pcl);
    void saveSettings();
    void loadSettings();

    //kinect
    ofxKinectV2 kinect;
    int kinectWidth = 512;
    int kinectHeight = 424;
    int sampleSize = 1;
    int depthThreshold = 10000;
    int pointSize = 1;
    ofColor pointColor;
    bool drawPointCloud = false;

    //PointClouds and Mesh
    ofMesh rawPCL;
    ofMesh finalPCL;
    ofMesh backgroundPCL;
    string backgroundMeshName = "background";
    bool backgroundLoaded = false;
    int bgTime = 3000;
    ofMesh floorCube;
    ofSpherePrimitive sphereSubtractor;
    ofVec3f sphereCentroid;

    //cam settings
    ofEasyCam cam;
    ofVec3f camPosition;
    int camX = 0;
    int camY = 0; //1000;
    int camZ = 0;
    int camTilt = 0; //180;
    int camPan = 0;
//    int camRoll = 0; //180;
    int fov = 60;
    float nearClip = 0.01f;
    float farClip = 1000000.0f;
    ofEasyCam bgCam;

    //origin
    ofBoxPrimitive originMarker;
    bool drawOrigin = true;

    //images
    bool drawFbo = true;
    ofFbo pointCloudFbo;
    ofFbo backgroundFbo;

    string HOST = "192.168.2.100";
    int PORT = 12345;
    float oscX = 0.0;
    float oscY = 0.0;


    //tcp
    int imageWidth = 320;
    int imageHeight = 240;
    int imageBytesSize;
    pbNetworkSenderSync tcpSender;
    int _frameId;

//    string tcpAddress = "192.168.2.100";  //linux node0
    string tcpAddress = "192.168.2.202"; // danbaker macbook pro (or other machine if assigned this ip
    string tcpAddress_localhost = "127.0.0.1";
    int tcpPort_0 = 5500;
    int tcpPort_1 = 5501;
    int tcpPort_2 = 5502;
    int tcpPort_3 = 5503;
    int tcpPort_4 = 5504;
    int packetSize = 1024;

    //gui
    ofxPanel gui;
    ofxButton saveBackgroundBtn;
    ofxButton loadBackgroundBtn;
    ofxToggle useBackgroundSubtraction;
    ofxFloatSlider bgDepthRange;
    int bgDepthRangeMax = 1000.0;
    ofxIntSlider sphereX;
    ofxIntSlider sphereY;
    ofxIntSlider sphereZ;
    int spherePosMin = -4500;
    int spherePosMax = 4500;
    ofxIntSlider mTranX;
    ofxIntSlider mTranY;
    ofxIntSlider mTranZ;
    ofxFloatSlider mRotX;
    ofxFloatSlider mRotY;
    ofxFloatSlider mRotZ;
    int rotMax = 360.0;

    int tX = -2205;
    int tY = 1440;
    int tZ = -585;
    float rX = 210.6;
    float rY = 329.4;
    float rZ = 133.2;

    ofxIntSlider floorThreshold;
    int floorMin = 0;
    int floorMax = -6000;
    int floorVal = -6000;

    ofxFloatSlider camRoll;
    float rollMin = 0;
    float rollMax = 360;
    float rollVal = 0;

    ofxButton saveSettingsBtn;
    ofxButton loadSettingsBtn;
};
