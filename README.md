# Kcl-Haptic-Emu

Prerequisites
=============

This repo is an extension to use the live functionality of this repo https://github.com/MatthiasKreileder/Kcl-Haptic-Sim and can not do anyhing useful as a stand-alone repo.

In order to learn how the different source files form a meaningful system that's runnable please start at the README from the repo in Kcl-Haptic-Sim or in the best case read my master thesis (:

Disclaimer: My thesis goal was to develop a protoype, I am aware that a real software product should come with some kind of installation script. But I am happy to do my best to assist if you want to make it work on your system, please write me a mail if something doesn't work/compile or run (matthias.kreileder@gmail.com).

What it contains
================

This source code contains the files to run the live examples of the Kcl-Haptic-Sim repo and is assumed you have the following setup available.

Geomagic Touch --- Windows PC with chai3d --- ethernet cable --- Linux machine with ns-3 and chai3d

FOR MORE INFO
Geomagic Touch: http://www.geomagic.com/en/products/phantom-omni/overview
chai3d: http://www.chai3d.org/
ns-3: https://www.nsnam.org/

Setup the Windows side
======================

PLEASE DO THE FOLLOWING
(1) Please follow these instructions to create a chai3d example with Visual Studio: http://www.chai3d.org/download/doc/html/chapter4-creation.html

(2) Use the source code found in 04-UDP-Local.cpp as the source of your example. 

WHY DOING THIS?
This adds the sending and receiving functionality to chai3d.

Setup the Linux side
====================
I assume you have chai3d installed in ~/Dev/chai3d.
I assume you have put the source of this repo into ~/Dev/Kcl-Haptic-Emu

PLEASE DO THE FOLLOWING
(1) user@~/Dev/Kcl-Haptic-Emu$ cp cMyCustomDeviceHandler.h ~/Dev/chai3d/src/devices/
(2) user@~/Dev/Kcl-Haptic-Emu$ cp cMyCustomDeviceHandler.cpp ~/Dev/chai3d/src/devices/
(3) user@~/Dev/Kcl-Haptic-Emu$ cp 04-shapes.cpp ~/Dev/chai3d/examples/GLUT/04-shapes
(4) Build chai3d

WHY DOING THIS?
Step (1) and (2) add the functionality to chai3d to talk to the ns-3 module Kcl-Haptic-Sim via a custom device implementation
Step (3) provides one example which uses this custom device (if I had more time, there would be more than once example :)
You know what step (4) does!

Configure both ethernet interfaces
==================================
Please assign your windows ethernet by assigning it the IP: 172.16.25.126 with the Subnet mask: 255.255.0.0
Please assign your linux eth0: IP 172.16.25.125 (same Subnet mask) and set it into promiscuous mode

Configure the Linux ns-3
========================

PLEASE NOTE
Network emulations are resource hungry programs, so please make sure you do a release (optimized) build to speed things up.

Idea and commands taken from https://www2.nsnam.org/doxygen/fd-emu-onoff_8cc_source.html
your-ns3-root-folder-here$ sudo chown root.root build/src/fd-net-device/ns3-dev-raw-sock-creator-optimized
your-ns3-root-folder-here$ sudo chmod 4755 build/src/fd-net-device/ns3-dev-raw-sock-creator-optimized

Try it and cross your fingers while doing it
============================================

ON THE LINUX MACHINE
sudo ./waf --run src/Kcl/Haptic-Sim/examples/kcl-wifi-udp-example
This should start a second process which runs chai3d (a window should pop up).

If you stop ns-3 please make sure that also the chai3d process (called 04-shapes) is also terminated, and yes that is not a brilliant way to start/end programs, please read the disclaimer for more info.

ON THE WINDOWS MACHINE
Start the 04-UDP-Local example

WHAT YOU SHOULD SEE
You control the virtual scence on the Linux machine
What you should feel (it's about haptic here :)
The data has to travel through an emulated network, you have successfully placed a wifi network between your haptic device and your remote chai3d environmet.

THINGS TO TRY OUT
Play around with the delay and channel quality inside ns-3 and observe how it changes the quality of experience!

Many thanks for actually using this code!!!
