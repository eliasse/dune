# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/parallels/clion-2020.1.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/parallels/clion-2020.1.1/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/parallels/CLionProjects/dune

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/parallels/CLionProjects/dune/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/dune.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dune.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dune.dir/flags.make

CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.o: CMakeFiles/dune.dir/flags.make
CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.o: DUNEGeneratedFiles/src/Main/StaticTasks.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.o -c /home/parallels/CLionProjects/dune/cmake-build-debug/DUNEGeneratedFiles/src/Main/StaticTasks.cpp

CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/parallels/CLionProjects/dune/cmake-build-debug/DUNEGeneratedFiles/src/Main/StaticTasks.cpp > CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.i

CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/parallels/CLionProjects/dune/cmake-build-debug/DUNEGeneratedFiles/src/Main/StaticTasks.cpp -o CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.s

CMakeFiles/dune.dir/src/Main/Daemon.cpp.o: CMakeFiles/dune.dir/flags.make
CMakeFiles/dune.dir/src/Main/Daemon.cpp.o: ../src/Main/Daemon.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/dune.dir/src/Main/Daemon.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS)  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -o CMakeFiles/dune.dir/src/Main/Daemon.cpp.o -c /home/parallels/CLionProjects/dune/src/Main/Daemon.cpp

CMakeFiles/dune.dir/src/Main/Daemon.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dune.dir/src/Main/Daemon.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS)  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -E /home/parallels/CLionProjects/dune/src/Main/Daemon.cpp > CMakeFiles/dune.dir/src/Main/Daemon.cpp.i

CMakeFiles/dune.dir/src/Main/Daemon.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dune.dir/src/Main/Daemon.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS)  -fexceptions -Wno-long-long -Wextra -Wformat -Wformat-security -Wno-missing-field-initializers -fdiagnostics-show-option -S /home/parallels/CLionProjects/dune/src/Main/Daemon.cpp -o CMakeFiles/dune.dir/src/Main/Daemon.cpp.s

# Object files for target dune
dune_OBJECTS = \
"CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.o" \
"CMakeFiles/dune.dir/src/Main/Daemon.cpp.o"

# External object files for target dune
dune_EXTERNAL_OBJECTS =

dune: CMakeFiles/dune.dir/DUNEGeneratedFiles/src/Main/StaticTasks.cpp.o
dune: CMakeFiles/dune.dir/src/Main/Daemon.cpp.o
dune: CMakeFiles/dune.dir/build.make
dune: libdune-core.a
dune: libActuators.AMC.a
dune: libActuators.Broom.a
dune: libActuators.FLIRPTU.a
dune: libActuators.LED4R.a
dune: libActuators.MCD4R.a
dune: libActuators.MicroCamD.a
dune: libActuators.PWM.a
dune: libActuators.SCRTv4.a
dune: libActuators.SIMCT01.a
dune: libActuators.SingleSIMCT01.a
dune: libAutonomy.OnEvent.a
dune: libAutonomy.TREX.a
dune: libAutonomy.TextActions.a
dune: libControl.ASV.HeadingAndSpeed.a
dune: libControl.ASV.RemoteOperation.a
dune: libControl.AUV.Allocator.a
dune: libControl.AUV.Attitude.a
dune: libControl.AUV.Diving.a
dune: libControl.AUV.LMI.a
dune: libControl.AUV.RemoteOperation.a
dune: libControl.AUV.Speed.a
dune: libControl.AntennaTracker.a
dune: libControl.Path.Height.a
dune: libControl.Path.ILOS.a
dune: libControl.Path.LOSnSMC.a
dune: libControl.Path.PurePursuit.a
dune: libControl.Path.VectorField.a
dune: libControl.ROV.Depth.a
dune: libControl.ROV.HorizontalPlane.a
dune: libControl.ROV.RemoteOperation.a
dune: libControl.UAV.Ardupilot.a
dune: libControl.UAV.LOS.a
dune: libControl.UAV.PX4.a
dune: libControl.UAV.RemoteOperation.a
dune: libKTH.Ardupilot.a
dune: libKTH.CMPS10.a
dune: libKTH.Fish.a
dune: libKTH.ISBInterface.a
dune: libKTH.TBR700.a
dune: libManeuver.CommsRelay.a
dune: libManeuver.CompassCalibration.a
dune: libManeuver.CoverArea.a
dune: libManeuver.FollowReference.AUV.a
dune: libManeuver.FollowReference.UAV.a
dune: libManeuver.FollowSystem.a
dune: libManeuver.FollowTrajectory.a
dune: libManeuver.Multiplexer.a
dune: libManeuver.RowsCoverage.a
dune: libManeuver.Teleoperation.a
dune: libManeuver.VehicleFormation.SMC.a
dune: libMonitors.Clock.a
dune: libMonitors.Collisions.a
dune: libMonitors.Emergency.a
dune: libMonitors.Entities.a
dune: libMonitors.FuelLevel.a
dune: libMonitors.Medium.a
dune: libMonitors.OperationalLimits.a
dune: libMonitors.Servos.a
dune: libNavigation.AUV.Navigation.a
dune: libNavigation.AUV.Ranger.a
dune: libNavigation.General.GPSNavigation.a
dune: libNavigation.General.LBL.a
dune: libNavigation.General.ROV.a
dune: libPlan.DB.a
dune: libPlan.Engine.a
dune: libPlan.Generator.a
dune: libPower.APD.a
dune: libPower.BATMANv2.a
dune: libPower.CPMB.a
dune: libPower.CPMBv2.a
dune: libPower.DOAMv1.a
dune: libPower.DOAMv2.a
dune: libPower.LUEMB.a
dune: libPower.MCBv2.a
dune: libPower.OPCON.a
dune: libPower.PCTLv2.a
dune: libPower.PSIMAR.a
dune: libSensors.AIM104MultiIO.a
dune: libSensors.AIS.a
dune: libSensors.CyclopsC7.a
dune: libSensors.DMS.a
dune: libSensors.DataStore.a
dune: libSensors.Edgetech2205.a
dune: libSensors.EmulatedGPS.a
dune: libSensors.GPS.a
dune: libSensors.Genesys.a
dune: libSensors.GillWindObserverII.a
dune: libSensors.IFOG.a
dune: libSensors.Imagenex837B.a
dune: libSensors.Imagenex852.a
dune: libSensors.Imagenex872.a
dune: libSensors.Imagenex881A.a
dune: libSensors.Keller.a
dune: libSensors.LIMU.a
dune: libSensors.MLBL.a
dune: libSensors.MLBLTracker.a
dune: libSensors.MTi.a
dune: libSensors.MetrecX.a
dune: libSensors.Microstrain3DMGX1.a
dune: libSensors.Microstrain3DMGX3.a
dune: libSensors.MiniSVS.a
dune: libSensors.OEMX.a
dune: libSensors.OS4000.a
dune: libSensors.SADC.a
dune: libSensors.SBG_ELLIPSE2D.a
dune: libSensors.SCH311X.a
dune: libSensors.SW100.a
dune: libSensors.SonTekArgonaut.a
dune: libSensors.ThermalZone.a
dune: libSensors.WifiRSSI.a
dune: libSensors.XR620CTD.a
dune: libSensors.XchangeSV.a
dune: libSimulators.AcousticModem.a
dune: libSimulators.CDC3.a
dune: libSimulators.CTD.a
dune: libSimulators.DVL.a
dune: libSimulators.DepthSensor.a
dune: libSimulators.Docking.a
dune: libSimulators.Environment.a
dune: libSimulators.GPS.a
dune: libSimulators.Gaussian.a
dune: libSimulators.IMU.a
dune: libSimulators.Iridium.a
dune: libSimulators.LBL.a
dune: libSimulators.Leaks.a
dune: libSimulators.Motor.a
dune: libSimulators.Servos.a
dune: libSimulators.StreamVelocity.a
dune: libSimulators.Target.a
dune: libSimulators.UAV.a
dune: libSimulators.UAVAutopilot.a
dune: libSimulators.USBL.a
dune: libSimulators.VSIM.a
dune: libSupervisors.AUV.Assist.a
dune: libSupervisors.AUV.LostComms.a
dune: libSupervisors.Delegator.a
dune: libSupervisors.Entities.a
dune: libSupervisors.Power.a
dune: libSupervisors.PowerManager.a
dune: libSupervisors.Reporter.a
dune: libSupervisors.SlaveCPU.a
dune: libSupervisors.UAV.LostComms.a
dune: libSupervisors.Vehicle.a
dune: libTransports.Announce.a
dune: libTransports.Cache.a
dune: libTransports.CommManager.a
dune: libTransports.DataStore.a
dune: libTransports.Discovery.a
dune: libTransports.Evologics.a
dune: libTransports.FTP.a
dune: libTransports.Fragments.a
dune: libTransports.GSM.a
dune: libTransports.HTTP.a
dune: libTransports.Iridium.a
dune: libTransports.IridiumSBD.a
dune: libTransports.LogBook.a
dune: libTransports.Logging.a
dune: libTransports.LoggingDigest.a
dune: libTransports.MobileInternet.a
dune: libTransports.Radio.a
dune: libTransports.Replay.a
dune: libTransports.Seatrac.a
dune: libTransports.Serial.a
dune: libTransports.SerialOverTCP.a
dune: libTransports.TCP.Client.a
dune: libTransports.TCP.Server.a
dune: libTransports.TCPOnDemand.a
dune: libTransports.UAN.a
dune: libTransports.UDP.a
dune: libUserInterfaces.Buttons.a
dune: libUserInterfaces.LCD.a
dune: libUserInterfaces.LEDs.a
dune: libUserInterfaces.MantaPanel.a
dune: libVision.DFK51BG02H.a
dune: libVision.FrameGrabber.a
dune: libVision.Lumenera.a
dune: libVision.PhotoTrigger.a
dune: libVision.UAVCamera.a
dune: libdune-core.a
dune: CMakeFiles/dune.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable dune"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dune.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dune.dir/build: dune

.PHONY : CMakeFiles/dune.dir/build

CMakeFiles/dune.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dune.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dune.dir/clean

CMakeFiles/dune.dir/depend:
	cd /home/parallels/CLionProjects/dune/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug /home/parallels/CLionProjects/dune/cmake-build-debug/CMakeFiles/dune.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dune.dir/depend

