TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
#!USR_CXXFLAGS += -DDEBUG

#=============================
# Build the IOC application

PROD_IOC = appliedMotion
# appliedMotion.dbd will be created and installed
DBD += appliedMotion.dbd

# appliedMotion.dbd will be made up from these files:
appliedMotion_DBD += base.dbd

# Include dbd files from all support applications:
#ifdef ASYN
appliedMotion_DBD += asyn.dbd
appliedMotion_DBD += drvAsynSerialPort.dbd
appliedMotion_DBD += drvAsynIPPort.dbd
#endif
appliedMotion_DBD += motorSupport.dbd
appliedMotion_DBD += appliedMotionMotor.dbd

# Add all the support libraries needed by this IOC
appliedMotion_LIBS += appliedMotion
appliedMotion_LIBS += motor
#ifdef ASYN
appliedMotion_LIBS += asyn
#endif
#ifdef SNCSEQ
appliedMotion_LIBS += seq pv
#endif

# appliedMotion_registerRecordDeviceDriver.cpp derives from appliedMotion.dbd
appliedMotion_SRCS += appliedMotion_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
appliedMotion_SRCS_DEFAULT += appliedMotionMain.cpp
appliedMotion_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#appliedMotion_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
appliedMotion_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

