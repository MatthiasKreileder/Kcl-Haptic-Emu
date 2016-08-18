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

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "../../../src/chai3d.h"
#include <ostream>
//------------------------------------------------------------------------------
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

// a small sphere (cursor) representing the haptic device
cShapeSphere* cursor;

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

// main haptics simulation loop
void updateHaptics(void);



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

    cout << handler->getNumDevices() << endl;
    //derived_ptr = static_pointer_cast<Derived>(base_ptr);
    cMyCustomDevicePtr myCustomDev = static_pointer_cast<cMyCustomDevice>(hapticDevice);
    //cout << "Des = " << myCustomDev->m_des << endl;
    // open a connection to haptic device
    hapticDevice->open();

    std::string fifo1 = std::string(argv[1]);
    std::string fifo2 = std::string(argv[2]);
    myCustomDev->openFifos(fifo1,fifo2);

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

    // create a sphere (cursor) to represent the haptic device
    cursor = new cShapeSphere(0.01);
    // insert cursor inside world
    world->addChild(cursor);

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

void updateHaptics(void)
{
    // precision clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

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

        // read position
        cVector3d position;
        hapticDevice->getPosition(position);



        /*
         * READ FROM NS3
         */

        cMyCustomDevicePtr myCustomDev = static_pointer_cast<cMyCustomDevice>(hapticDevice);
        std::string dataSampleAsString;
        //myCustomDev->SafeRead(dataSampleAsString);

        std::string sharedMemFromNs3;
        myCustomDev->KclSafeRead(sharedMemFromNs3);


		  /*
		   * Extract message
		   */
		  std::size_t firstHashTag = sharedMemFromNs3.find_first_of('#');
		  if(firstHashTag == 0){
			  // no haptic sample from ns3 available => nothing for us to do
			  dataSampleAsString = "0.000, 0.000, 0.000";
		  }
		  else{
			  dataSampleAsString = sharedMemFromNs3.substr(0,firstHashTag);

		  }


    	//
    	// x coordinate
    	//
    	std::size_t firstCommaPos = dataSampleAsString.find(",");
    	std::string xString = dataSampleAsString.substr(0,firstCommaPos - 1);
    	double x = stod(xString);

    	//
    	// y coordinate
    	//
    	// "-0.20503, -0.98147"
    	std::string truncatedPosData = dataSampleAsString.substr(firstCommaPos + 1,dataSampleAsString.size() - firstCommaPos - 1);
    	std::size_t secondCommaPos = truncatedPosData.find(",");
    	std::string yString = truncatedPosData.substr(0,secondCommaPos -1);
    	double y = stod(yString);


    	//
    	// z coordinate
    	//
    	// "-0.98147"
    	std::string zString = truncatedPosData.substr(secondCommaPos + 1, truncatedPosData.size() - secondCommaPos -1);
    	double z = stod(zString);

    	cVector3d positionFromNs3(x,y,z);

        // update position and orientation of cursor
        cursor->setLocalPos(positionFromNs3);
        //tool->setLocalPos(positionFromNs3);
        //tool->setV
        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);


        double scaleFactor = 0.0;
        scaleFactor = tool->getWorkspaceScaleFactor();

        // update position and orientation of tool
        tool->updateFromDevice();
        tool->setDeviceLocalPos(positionFromNs3);

        cMatrix3d globalRot;
        globalRot.identity();

        tool->setDeviceGlobalPos(globalRot*positionFromNs3);


        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();

        /*
         * Get the global force value which needs to be sent back to the real PhantomOmni
         */
        cVector3d deviceGlobalPos;
        deviceGlobalPos.zero();
        deviceGlobalPos = tool->getDeviceGlobalPos();

        cMatrix3d deviceGlobalRot;
        deviceGlobalRot.identity();
        deviceGlobalRot = tool->getDeviceGlobalRot();

        cVector3d deviceGlobalLinVel;
        deviceGlobalLinVel.zero();
        deviceGlobalLinVel = tool->getDeviceGlobalLinVel();

        cVector3d deviceGlobalAngVel;
        deviceGlobalAngVel.zero();
        deviceGlobalAngVel = tool->getDeviceGlobalAngVel();

        cVector3d globalForce = tool->m_hapticPoint->computeInteractionForces(	deviceGlobalPos,
        																		deviceGlobalRot,
																				deviceGlobalLinVel,
																				deviceGlobalAngVel);

        /*
         * WRITE TO NS3
         */
        std::ostringstream stringForNs3;
        stringForNs3 << scaleFactor;
        //myCustomDev->SafeWrite(globalForce.str(6));
        myCustomDev->KclSafeWrite(globalForce.str(6));
        /*
         * WRITE TO NS3
         */


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
}

//------------------------------------------------------------------------------
