//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2015, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.0.0 $Rev: 1772 $
*/
//==============================================================================

/*
	Comment author: Matthias Kreileder
	I used and extended this source file for the purpose of my MSc thesis
	at the chair of telecommunications at King's College London.

	Non of this code is my work expect where I explicitly state it above a
	function declaration as follows:

	// Author: Matthias Kreileder
	// Code developed as part of my MSc thesis at King's College London
	void foo(){ ... }

*/

//------------------------------------------------------------------------------
#include "boost/asio.hpp" 
#include "boost/array.hpp" 
#include "boost/make_shared.hpp"
#include "boost/ref.hpp"
#include "chai3d.h"
#include <queue>

// We want to do a async read on the socket which communicates with the outside world
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <fstream>

//------------------------------------------------------------------------------
using boost::asio::ip::udp;
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

#define PACKETSIZE sizeof(cVector3d) //packet size


// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a few shape primitives that compose our scene
cShapeSphere* sphere0;
cShapeSphere* sphere1;
cShapeLine* line;
cShapeCylinder* cylinder;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// a level to display the position of the cylinder along the line
cLevel* level;

// a small scope to display the interaction force signal
cScope* scope; 

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

const int delayTime = 2;
cVector3d  zeroVector(0.0, 0.0, 0.0);
std::queue<cVector3d> fifo3DPos;
std::queue<cVector3d> fifo3DForce;

ofstream positionFile;
ofstream velocityFile;
ofstream forceFile;

/*
 *	Author: Matthias Kreileder
 *	Code developed as part of my MSc thesis at King's College London
 *
 *	Brief: Container for the force feedback samples received from 172.16.25.125
 *  
 *	This object is shared between the Haptics Thread and the Networking Thread
 *	=> Always use the mutex forceFeedbackSamplesFromNetwork_lock when working 
 *	with this shared container
 */
std::queue<cVector3d> forceFeedbackSamplesFromNetwork;
cVector3d lastForceFeedbackSampleFromNetwork(0.0, 0.0, 0.0);
boost::mutex forceFeedbackSamplesFromNetwork_lock;

/*
 *	Author: Matthias Kreileder
 *	Code developed as part of my MSc thesis at King's College London
 *	
 *	Brief: Thread-safe method to push a force-feedback data sample
 *	into the shared queue forceFeedbackSamplesFromNetwork
 *
 *	returns: true on success and false when an error occurs
 */
bool pushForceFeedbackSampleToQueue(std::string sampleAsString) {
	
	try {
		//
		//	We assume three dimensional, comma separated sensor data here:
		//	"-0.18180, -0.20503, -0.98147"

		//
		// x coordinate
		//
		std::size_t firstFloatingPoint = sampleAsString.find(".");
		std::size_t firstCommaPos = sampleAsString.find(",");
		std::string xString = sampleAsString.substr(firstFloatingPoint -1, firstCommaPos - 1);
		double x = stod(xString);
	

		//
		// y coordinate
		//
		// "-0.20503, -0.98147"
		std::string truncatedPosData = sampleAsString.substr(firstCommaPos + 1, sampleAsString.size() - firstCommaPos - 1);
		std::size_t secondCommaPos = truncatedPosData.find(",");
		std::string yString = truncatedPosData.substr(0, secondCommaPos - 1);
		double y = stod(yString);
	

		//
		// z coordinate
		//
		// "-0.98147"
		std::string zString = truncatedPosData.substr(secondCommaPos + 1, truncatedPosData.size() - secondCommaPos - 1);
		double z = stod(zString);

		cVector3d forceSample(x,y,z);

		// forceFeedbackSamplesFromNetwork is shared between the Haptics Thread and the Networking Thread
		forceFeedbackSamplesFromNetwork_lock.lock();
		forceFeedbackSamplesFromNetwork.push(forceSample);
		
		//std::cout << "RX Queue: " << forceFeedbackSamplesFromNetwork.size() << std::endl;
		/*
		 *	If the queue gets long it might be better
		 *	to skip the outdated samples 
		 */
		if (forceFeedbackSamplesFromNetwork.size() > 3) {
			for (size_t i = 0; i < forceFeedbackSamplesFromNetwork.size() - 1; i++)
				forceFeedbackSamplesFromNetwork.pop();
		}
		
		forceFeedbackSamplesFromNetwork_lock.unlock();

		

		// we made it here => things went fine
		return true;
	}
	catch (exception& e) {
		std::cerr << e.what() << std::endl;
		return false;
	}
}


/*
 *	Author: Matthias Kreileder	
 *	Code developed as part of my MSc thesis at King's College London
 *	
 *	Brief: Thread-safe method to get a force-feedback data sample
 *	from the shared queue forceFeedbackSamplesFromNetwork
 *
 *	returns: the last received force-feedback data sample if
 *	forceFeedbackSamplesFromNetwork is empty otherwise it returns 
 *	the element at the front (and removes it from the queue).
 *	If the queue is empty and no force-feedback data sample
 *	has been received so far it return a cVector3d object with
 *	all fields set to zero
 */
cVector3d popForceFeedbackSampleFromQueue() {

	forceFeedbackSamplesFromNetwork_lock.lock();
	if (forceFeedbackSamplesFromNetwork.empty()) {
		forceFeedbackSamplesFromNetwork_lock.unlock();
		return lastForceFeedbackSampleFromNetwork;
	}

	// we made it here => queue is not empty
	cVector3d vec = forceFeedbackSamplesFromNetwork.front();
	forceFeedbackSamplesFromNetwork.pop();
	forceFeedbackSamplesFromNetwork_lock.unlock();
	lastForceFeedbackSampleFromNetwork = vec;
	return vec;
}

//initialize network connection parameters
// This is the remote IP address and port number we want to send the haptic data to
const std::string& host = "172.16.25.125";
const std::string& port = "1234";

//my address
const std::string& senderIP = "localhost";
const std::string& port2 = "5001";

bool skip = "false";
void forceFeedbackFromNetwork();


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

void sendUDPpos(int delay, boost::shared_ptr<udp::socket> socket, udp::endpoint &receiver_endpoint);
void receiveUDP(int delay, boost::shared_ptr<udp::socket> socket, udp::endpoint &sender_endpoint);
void delayedForceFeedback(int delay);


// main haptics simulation loop
void updateHaptics(void);

/*
 *	Author: Matthias Kreileder
 *	Code developed as part of my MSc thesis at King's College London
 * 
 *	Async receiving
 *  Code and idea based on Boost.Asio C++ Network Programming - Second Edition
 *	by John Torjo, Wisnu Anggoro ISBN: 9781785283079
 */
void WorkerThread(boost::shared_ptr<boost::asio::io_service> iosvc) {
	std::cout << "WorkerThread: About to start asio event processing loop." << "\n";
	iosvc->run();
	std::cout << "End.\n";
}

//==============================================================================
/*
    DEMO:   04-shapes.cpp

    In this example we illustrate how to create some simple scene composed 
    of a few shape primitives that include the sphere, cylinder and line.

    In the haptic threads we compute the interaction between the tool (cursor)
    and the different object composing the scene. 

    A widget is also placed on the front plane to display information relative 
    to the position of the cylinder.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 04-shapes" << endl;
    cout << "Copyright 2003-2015" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Enable/Disable potential field" << endl;
    cout << "[2] - Enable/Disable damping" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;

	//--------------------------------------------------------------------------
	// Initializing boost.asio for udp connection
	//--------------------------------------------------------------------------



   
    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = 0.8 * screenH;
    windowH = 0.5 * screenH;
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY; 

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);

    if (stereoMode == C_STEREO_ACTIVE)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
    // initialize GLEW
    glewInit();
#endif

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d(3.0, 0.0, 0.0),    // camera position (eye)
                 cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);                   

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);         


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // define a radius for the virtual tool (sphere)
    tool->setRadius(0.03);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATING SHAPES
    //--------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////
    // In the following lines create some basic shape primitives such as two
    // spheres, a line and a cylinder. For each primitive we define their
    // dimensions calling their constructor. These values can of course be 
    // modified later in the program by calling the appropriate methods 
    // (setRadius(), setHeight(), etc...). Haptic effects are also created for 
    // each object. In this example we also introduce surface and magnetic
    // effects. Settings for these effects are controlled by adjusting the 
    // different parameters in the m_material properties.
    // We suggest that you explore the different settings.
    ////////////////////////////////////////////////////////////////////////////

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // get properties of haptic device
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    double maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);


    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - SPHERE 0
    ////////////////////////////////////////////////////////////////////////////

    // create a sphere
    sphere0 = new cShapeSphere(0.1);
    world->addChild(sphere0);

    // set position
    sphere0->setLocalPos(0.0,-0.7, 0.0);
    
    // set material color
    sphere0->m_material->setRedFireBrick();

    // create haptic effect and set properties
    sphere0->createEffectSurface();
    
    // set stiffness property
    sphere0->m_material->setStiffness(0.4 * maxStiffness);


    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - SPHERE 1
    ////////////////////////////////////////////////////////////////////////////

    // create a sphere
    sphere1 = new cShapeSphere(0.1);
    world->addChild(sphere1);

    // set position
    sphere1->setLocalPos(0.0, 0.7, 0.0);

    // set material color
    sphere1->m_material->setRedFireBrick();

    // create haptic effect and set properties
    sphere1->createEffectSurface();
    sphere1->m_material->setStiffness(0.4 * maxStiffness);


    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - LINE
    ////////////////////////////////////////////////////////////////////////////

    // create a line
    line = new cShapeLine(sphere0->getLocalPos(), sphere1->getLocalPos());
    world->addChild(line);

    // set color at each point
    line->m_colorPointA.setWhite();
    line->m_colorPointB.setWhite();

    // create haptic effect and set haptic properties
    line->createEffectMagnetic();
    line->m_material->setMagnetMaxDistance(0.05);
    line->m_material->setMagnetMaxForce(0.3 * maxLinearForce);
    line->m_material->setStiffness(0.2 * maxStiffness);


    ////////////////////////////////////////////////////////////////////////////
    // SHAPE - CYLINDER
    ////////////////////////////////////////////////////////////////////////////

    // create a cylinder
    cylinder = new cShapeCylinder(0.25, 0.25, 0.2);
    world->addChild(cylinder);

    // set position and orientation
    cylinder->setLocalPos(0.0, 0.0, 0.0);
    cylinder->rotateAboutGlobalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90);

    // set material color
    cylinder->m_material->setBlueCornflower();

    // create haptic effect and set properties
    cylinder->createEffectSurface();
    cylinder->m_material->setStiffness(0.8 * maxStiffness);
    
    
    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFont *font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    camera->m_frontLayer->addChild(labelHapticRate);

    // create a level to display the relative position of the cylinder
    level = new cLevel();
    camera->m_frontLayer->addChild(level);
    level->rotateWidgetDeg(-90);
    level->setRange(-0.5, 0.6);
    level->setSize(40);
    level->setNumIncrements(100);
    level->setSingleIncrementDisplay(true);
    level->setTransparencyLevel(0.5);

    // create a scope to plot haptic device position data
    scope = new cScope();
    camera->m_frontLayer->addChild(scope);
    scope->setSize(400, 100);
    scope->setRange(0.0, 5.0);
    scope->setSignalEnabled(true, false, false, false);
    scope->setPanelEnabled(false);
    scope->m_colorSignal0.setRedCrimson();


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------



    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);


    // setup callback when application exits
    atexit(close);

    // start the main graphics rendering loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();


    // exit
    return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    // update the size of the viewport
    windowW = w;
    windowH = h;

    // update position of level
    level->setLocalPos((0.5 * (windowW - level->getHeight())), 90);

    // update position of scope
    scope->setLocalPos((0.5 * (windowW - scope->getWidth())), 120);
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
        // exit application
        exit(0);
    }

    // option f: toggle fullscreen
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // display haptic rate data
    labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);

    // update value of level
    level->setValue( cylinder->getLocalPos().y() );

    // update value of scope
    scope->setSignalValues( tool->getDeviceGlobalForce().length() );
    

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------
static int skipCounter = 0;
void sendUDPpos(int delay, boost::shared_ptr<udp::socket> socket, udp::endpoint &receiver_endpoint) {

	
	if (skip) {
		skip = false;
		return;
	}
	else {
		skip = true;
	}
	
	/*
	if (skipCounter < 1000) {
		skipCounter++;
		return;
	}
	else {
		skip = 0;
	}*/
	
	
	
	
	// check if device is available
	if ((hapticDevice == nullptr) || (!tool->getEnabled()))
	{
		cSleepMs(1);
		return;
	}

	//////////////////////////////////////////////////////////////////////
	// retrieve data from haptic device
	//////////////////////////////////////////////////////////////////////

	// temp variables
	
	cVector3d delayedLocalPos,devicePos, deviceLinVel, localLinVel, globalPos, localPos;
	cMatrix3d globalRot;

	double scaleFactor;
	unsigned int userSwitches;

	// init temp variable
	delayedLocalPos.zero();
	zeroVector.zero();
	globalPos.zero();
	localPos.zero();
	devicePos.zero();
	deviceLinVel.zero();
	localLinVel.zero();
	globalRot.identity();

	userSwitches = 0;
	scaleFactor = 0.0;
	// update position, orientation, linear and angular velocities from device
 
	hapticDevice->getPosition(devicePos);
	hapticDevice->getLinearVelocity(deviceLinVel);
	
	
	
	//std::cout << deviceLinVel.str(6) << std::endl;

	hapticDevice->getUserSwitches(userSwitches);
	scaleFactor = tool->getWorkspaceScaleFactor();
	globalPos = tool->getGlobalPos();
	localLinVel = tool->getDeviceLocalLinVel();
	globalRot = tool->getGlobalRot();


	//////////////////////////////////////////////////////////////////////
	// update information inside tool
	//////////////////////////////////////////////////////////////////////
	
	// compute local position - adjust for tool workspace scale factor
	localPos = scaleFactor * devicePos;
	localLinVel = scaleFactor * deviceLinVel;
	velocityFile << localLinVel.str(6) << '\n';
	//cout << localPos << endl;
	//serialize position
	positionFile << localPos.str(6) << '\n';
	string msg = '(' + localPos.str(6) + ')';

	socket->send_to(boost::asio::buffer(msg, msg.size()), receiver_endpoint);
	
	if (fifo3DPos.empty()) {
		for (int i = 0; i < delay; i++) {
			fifo3DPos.push(zeroVector);
			// cout << fifo3DPos.back() << " in position " << i << endl;
		}
	}
	else {
		fifo3DPos.push(localPos);
		// cout << fifo3DPos.back() << endl;
	}
	delayedLocalPos = fifo3DPos.front();
	fifo3DPos.pop();

	tool->setDeviceLocalPos(delayedLocalPos);

	// compute global position in world coordinates
	tool->setDeviceGlobalPos(globalPos + globalRot * delayedLocalPos);

	// compute local rotation
	//tool->setDeviceLocalRot(deviceRot);
	// compute global rotation
	//tool->setDeviceGlobalRot(globalRot * localRot);
	// compute local linear velocity - adjust for tool workspace scale factor
	//tool->setDeviceLocalLinVel( scaleFactor* deviceLinVel);
	// compute global linear velocity
	//tool->setDeviceGlobalLinVel(globalRot * localLinVel);
	// compute local rotational velocity
	//tool->setDeviceLocalAngVel(deviceAngVel);
	// compute global rotational velocity
	//tool->setDeviceGlobalAngVel(globalRot * localAngVel);
	// store gripper angle
	//tool->setGripperAngleRad(gripperAngle);
	// store gripper angular velocity
	//tool->setGripperAngVel(gripperAngVel);
	// store user switch status
	tool->setUserSwitches(userSwitches);

	// update the position and orientation of the tool image
	tool->updateToolImagePosition();
}


/*
 *	Author: Matthias Kreileder
 *	Code developed as part of my MSc thesis at King's College London
 */
void forceFeedbackFromNetwork() {

	cVector3d delayedGlobalForce, deviceGlobalLinVel, deviceGlobalPos, deviceGlobalAngVel;
	cMatrix3d deviceGlobalRot;
	cVector3d globalTorque(0.0, 0.0, 0.0);

	delayedGlobalForce.zero();
	deviceGlobalAngVel.zero();
	deviceGlobalLinVel.zero();
	deviceGlobalPos.zero();
	deviceGlobalRot.identity();


	deviceGlobalPos = tool->getDeviceGlobalPos();
	deviceGlobalLinVel = tool->getDeviceGlobalLinVel();
	deviceGlobalRot = tool->getDeviceGlobalRot();

	deviceGlobalAngVel = tool->getDeviceGlobalAngVel();

	// compute interaction forces at haptic point in global coordinates
	cVector3d globalForce = popForceFeedbackSampleFromQueue();
	// update computed forces to tool
	tool->setDeviceGlobalForce(globalForce);
	tool->setDeviceGlobalTorque(globalTorque);
	tool->setGripperForce(0.0);

	tool->applyToDevice();
}

void delayedForceFeedback(int delay) {
	
	cVector3d delayedGlobalForce, deviceGlobalLinVel, deviceGlobalPos, deviceGlobalAngVel;
	cMatrix3d deviceGlobalRot;
	cVector3d globalTorque(0.0, 0.0, 0.0);

	delayedGlobalForce.zero();
	deviceGlobalAngVel.zero();
	deviceGlobalLinVel.zero();
	deviceGlobalPos.zero();
	deviceGlobalRot.identity();
	

	deviceGlobalPos = tool->getDeviceGlobalPos();
	deviceGlobalLinVel = tool->getDeviceGlobalLinVel();
	deviceGlobalRot = tool->getDeviceGlobalRot();


	//hapticDevice->getPosition(deviceGlobalPos);
	//hapticDevice->getLinearVelocity(deviceGlobalLinVel);
	//hapticDevice->getRotation(deviceGlobalRot);
	deviceGlobalAngVel = tool->getDeviceGlobalAngVel();

	// compute interaction forces at haptic point in global coordinates
	cVector3d globalForce = tool->m_hapticPoint->computeInteractionForces(deviceGlobalPos,
		deviceGlobalRot,
		deviceGlobalLinVel,
		deviceGlobalAngVel);

	if (fifo3DForce.empty()) {
		for (int i = 0; i < delay; i++) {
			fifo3DForce.push(zeroVector);
		}
	}
	else {
		fifo3DForce.push(globalForce);
	}

	delayedGlobalForce = fifo3DForce.front();
	
	forceFile << delayedGlobalForce.str(6) << '\n';
	
	fifo3DForce.pop();
	// update computed forces to tool
	tool->setDeviceGlobalForce(delayedGlobalForce);
	tool->setDeviceGlobalTorque(globalTorque);
	tool->setGripperForce(0.0);

}

void receiveUDP(int delay, boost::shared_ptr<udp::socket> socket, udp::endpoint &sender_endpoint) {

	boost::array<char, 30> recv_buff;
	cVector3d delayedGlobalForce, deviceGlobalLinVel, deviceGlobalPos, deviceGlobalAngVel;
	cMatrix3d deviceGlobalRot;
	cVector3d globalTorque(0.0, 0.0, 0.0);

	delayedGlobalForce.zero();
	deviceGlobalAngVel.zero();
	deviceGlobalLinVel.zero();
	deviceGlobalPos.zero();
	deviceGlobalRot.identity();

	
	deviceGlobalPos = tool->getDeviceGlobalPos();
	deviceGlobalLinVel = tool->getDeviceGlobalLinVel();
	deviceGlobalRot = tool->getDeviceGlobalRot();


	//hapticDevice->getPosition(deviceGlobalPos);
	//hapticDevice->getLinearVelocity(deviceGlobalLinVel);
	//hapticDevice->getRotation(deviceGlobalRot);
	deviceGlobalAngVel = tool->getDeviceGlobalAngVel();


	socket->receive_from(boost::asio::buffer(recv_buff), sender_endpoint);

    cout << recv_buff.data();


//	socket->receive(boost::asio::buffer(recv_buff));

	// compute interaction forces at haptic point in global coordinates
	//cVector3d globalForce = tool->m_hapticPoint->computeInteractionForces(deviceGlobalPos,
	//	deviceGlobalRot,
	//	deviceGlobalLinVel,
	//	deviceGlobalAngVel);

	//if (fifo3DForce.empty()) {
	//	for (int i = 0; i < delay; i++) {
	//		fifo3DForce.push(zeroVector);
	//	}
	//}
	//else {
	//	fifo3DForce.push(globalForce);
	//}

	//delayedGlobalForce = fifo3DForce.front();
	//fifo3DForce.pop();

	//socket->receive_from(boost::asio::buffer(recv_buf))

	// update computed forces to tool
	//tool->setDeviceGlobalForce(delayedGlobalForce);
	//tool->setDeviceGlobalTorque(globalTorque);
	//tool->setGripperForce(0.0);

}

/*
 *	Author: Matthias Kreileder
 *	Code developed as part of my MSc thesis at King's College London
 *
 *	Async receiving
 *	Code and idea based on Boost.Asio C++ Network Programming - Second Edition
 *	by John Torjo, Wisnu Anggoro ISBN: 9781785283079
 */
void KclWorkerThread(boost::shared_ptr<boost::asio::io_service> iosvc, int counter) {
	std::cout << counter << ".\n";
	iosvc->run();
	std::cout << "End.\n";
}


udp::endpoint remote_endpoint_;
boost::array<char, 100> recv_buffer_;

/*
 *	Author: Matthias Kreileder
 *	Code developed as part of my MSc thesis at King's College London
 *
 *	Async receiving
 *	Code and idea based on Boost.Asio C++ Network Programming - Second Edition
 *	by John Torjo, Wisnu Anggoro ISBN: 9781785283079
 *	as well as on
 *  http://www.boost.org/doc/libs/1_38_0/doc/html/boost_asio/reference/basic_datagram_socket/async_receive_from/overload1.html
 */
void handle_receive(const boost::system::error_code& error,
	std::size_t bytes_transferred)
{
	if (!error || error == boost::asio::error::message_size)
	{
		std::string sampleAsString;
		for (size_t i = 0; i < bytes_transferred; i++){
			char c = recv_buffer_[i];
			sampleAsString.push_back(c);
		}


		pushForceFeedbackSampleToQueue(sampleAsString);

	}
	else {
		std::string msg = error.message();
		std::cout << "Error msg from BOOST-ASIO: " << msg << std::endl;
	}
}

void updateHaptics(void)
{
	positionFile.open("position.txt");
	velocityFile.open("velocity.txt");
	forceFile.open("force.txt");

    // precision clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

	boost::shared_ptr<boost::asio::io_service> io_service(
		new boost::asio::io_service
		);
	//boost::asio::io_service io_service;
	boost::shared_ptr<boost::asio::io_service::work> worker(
		new boost::asio::io_service::work(*io_service)
		);


	udp::resolver resolver(*io_service);
	udp::resolver::query query(udp::v4(), host, port);
	udp::resolver::query query2(udp::v4(), senderIP, port2);
	udp::endpoint receiver_endpoint = *resolver.resolve(query);
	udp::endpoint sender_endpoint = *resolver.resolve(query2);

	//udp::socket socket(io_service);

	boost::shared_ptr<udp::socket> data_socket(new udp::socket(*io_service, udp::endpoint(udp::v4(), 1234)));
	

	boost::thread_group threads;
	threads.create_thread(boost::bind(&KclWorkerThread, io_service, 1));
	
	/* Non blocking receive */
	data_socket->async_receive_from(
		boost::asio::buffer(recv_buffer_), remote_endpoint_,
		boost::bind(handle_receive, 
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
	
	
	//boost::shared_ptr<udp::socket> data_socket2(new udp::socket(io_service, udp::endpoint(udp::v4(), 5001)));

	// main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME    
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = clock.getCurrentTimeSeconds();

        // restart the simulation clock
        clock.reset();
        clock.start();

        // update frequency counter
        frequencyCounter.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);


        // update position and orientation of tool
        //tool->updateFromDevice();
		sendUDPpos(delayTime, data_socket, receiver_endpoint);

		data_socket->async_receive_from(
			boost::asio::buffer(recv_buffer_), remote_endpoint_,
			boost::bind(handle_receive,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));

		///////////////////////////receiveUDP(delayTime, data_socket2, sender_endpoint);
		
		//cVector3d forceFeedbackFromNetwork = popForceFeedbackSampleFromQueue();
		//std::cout << forceFeedbackFromNetwork.str() << std::endl;

        // compute interaction forces
        //////////////////////tool->computeInteractionForces();
		delayedForceFeedback(delayTime);
		
		// Matt's Msc thesis
		forceFeedbackFromNetwork();
        
		// Comment out for Matt's Msc thesis
		// send forces to haptic device
        //tool->applyToDevice();


        /////////////////////////////////////////////////////////////////////
        // HAPTIC SIMULATION
        /////////////////////////////////////////////////////////////////////
        
        // check if contact occurred with cylinder
        if(tool->isInContact(cylinder))
        {
            // get force applied on the y axis
            double force = tool->getDeviceGlobalForce().y();

            // move cylinder along the line according to force
            const double K = 0.5;
            double pos = cylinder->getLocalPos().y();
            pos = cClamp(pos - K * timeInterval * force,-0.5, 0.6);
            cylinder->setLocalPos(0.0, pos, 0.0);
        }
    }
    
    // exit haptics thread
    simulationFinished = true;
	
	/*
	 *	Tear down the work object so that the background worker thread can exit
	 */
	worker.reset();

	/*
	 *	Wait for the background worker thread to finish before we return
	 */
	threads.join_all();

	positionFile.close();
	velocityFile.close();
	forceFile.close();
	
}

//------------------------------------------------------------------------------
