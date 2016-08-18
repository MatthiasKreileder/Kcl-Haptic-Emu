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
    \author    Your name, institution, or company name.
    \version   3.0.0 $Rev: 1639 $
*/
//==============================================================================
#include "../system/CGlobals.h"
//------------------------------------------------------------------------------
#ifndef CMyCustomDeviceH
#define CMyCustomDeviceH
//------------------------------------------------------------------------------
#define C_ENABLE_CUSTOM_DEVICE_SUPPORT
#if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------
#include "devices/CGenericHapticDevice.h"
//------------------------------------------------------------------------------
#include <string>
#include <cstdlib>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////////////
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <cstdlib>
#include <string>
#include <vector>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <cstdlib>
#include <zconf.h>
#include <fcntl.h>
#include <semaphore.h>
#include <iostream>

#define SHMSZ     27
////////////////////////////////////////////////////////////////////////////////

//------------------------------------------------------------------------------
namespace chai3d {
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \file       CMyCustomDevice.h

    \brief
    Implements support for custom haptic device.
*/
//==============================================================================

//------------------------------------------------------------------------------
class cMyCustomDevice;
typedef std::shared_ptr<cMyCustomDevice> cMyCustomDevicePtr;
//------------------------------------------------------------------------------

//==============================================================================
/*!
    \class      cMyCustomDevice
    \ingroup    devices  

    \brief
    This class is a interface to support custom haptic devices (template).

    \details
    This class provides the basics to easily interface CHAI3D to your 
    own custom haptic device. \n\n

    Simply follow the 11 commented step in file CMyCustomDevice.cpp 
    and complete the code accordingly.
    Depending of the numbers of degrees of freedom of your device, not
    all methods need to be implemented. For instance, if your device
    does not provide any rotation degrees of freedom, simply ignore
    the getRotation() method. Default values will be returned correctly
    if these are not implemented on your device. In the case of rotations
    for instance, the identity matrix is returned.\n\n

    You may also rename this class in which case you will also want to
    customize the haptic device handler to automatically detect your device.
    Please consult method update() of the cHapticDeviceHandler class
    that is located in file CHapticDeviceHandler.cpp .
    Simply see how the haptic device handler already looks for
    device of type cMyCustomDevice.\n\n

    If you are encountering any problems with your implementation, check 
    for instance file cDeltaDevices.cpp which implement supports for the 
    Force Dimension series of haptic devices. In order to verify the implementation
    use the 01-device example to get started. Example 11-effects is a great
    demo to verify how basic haptic effects may behave with you haptic
    devices. If you do encounter vibrations or instabilities, try reducing
    the maximum stiffness and/or damping values supported by your device. 
    (see STEP-1 in file CMyCustomDevice.cpp).\n
    
    Make  sure that your device is also communicating fast enough with 
    your computer. Ideally the communication period should take less 
    than 1 millisecond in order to reach a desired update rate of at least 1000Hz.
    Problems can typically occur when using a slow serial port (RS232) for
    instance.\n
*/
//==============================================================================
class cMyCustomDevice : public cGenericHapticDevice
{
    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    //! Constructor of cMyCustomDevice.
    cMyCustomDevice(unsigned int a_deviceNumber = 0);

    //! Destructor of cMyCustomDevice.
    virtual ~cMyCustomDevice();

    //! Shared cMyCustomDevice allocator.
    static cMyCustomDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<cMyCustomDevice>(a_deviceNumber)); }


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    //! This method opens a connection to the haptic device.
    virtual bool open();

    //! This method closes the connection to the haptic device.
    virtual bool close();

    //! This method calibrates the haptic device.
    virtual bool calibrate(bool a_forceCalibration = false);

    //! This method returns the position of the device.
    virtual bool getPosition(cVector3d& a_position);

    //! This method returns the orientation frame of the device end-effector.
    virtual bool getRotation(cMatrix3d& a_rotation);

    //! This method returns the gripper angle in radian [rad].
    virtual bool getGripperAngleRad(double& a_angle);

    //! This method sends a force [N] and a torque [N*m] and gripper force [N] to the haptic device.
    virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);

    //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
    virtual bool getUserSwitches(unsigned int& a_userSwitches); 


    //--------------------------------------------------------------------------
    // PUBLIC STATIC METHODS:
    //--------------------------------------------------------------------------

public: 

    //! This method returns the number of devices available from this class of device.
    static unsigned int getNumDevices();


    void openFifos(std::string fifo1, std::string fifo2);

    /*
     * \brief Will read a message of up to 100 bytes from the pipe
     *
     * Blocks until one message arrives.
     *
     * \param s puts the string that was read from the named pipe into s
     */
    void SafeRead(std::string& s);

    /*
     * \brief write a string of up to 100 bytes to the named pipe
     *
     * Does nothing if the string contains more than 100 bytes
     *
     * \param s the string that will be written to the pipe
     */
    void SafeWrite(std::string s);

    void KclSafeRead(std::string& msg);
    void KclSafeWrite(std::string msg);
    void KclInitSharedMem();

    //--------------------------------------------------------------------------
    // PROTECTED MEMBERS:
    //--------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////
    /*
        INTERNAL VARIABLES:

        If you need to declare any local variables or methods for your device,
        you may do it here below. 
    */
    ////////////////////////////////////////////////////////////////////////////

protected:

    //! A short description of my variable
    int m_MyVariable;

protected:

    //! Device ID number among the Phantom devices connected to the computer.
    int m_deviceID;

    double m_x_pos;
    int m_rx_pipe;  // Pipe to receive from peer
    int m_tx_pipe;  // Pipe to send to peer

private:
    char* m_startOfSharedMemoryPage;
    sem_t* m_semaphore;

public:
    int m_des;
};

//------------------------------------------------------------------------------
}       // namespace chai3d
//------------------------------------------------------------------------------
#endif  // C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
