TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The following are used for debugging messages.
USR_CXXFLAGS += -DDEBUG

#==================================================
# build a support library

LIBRARY_IOC += motorAppliedMotion

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install motorAppliedMotion.dbd into <top>/dbd
DBD += motorAppliedMotion.dbd

# specify all source files to be compiled and added to the library
motorAppliedMotion_SRCS += appliedMotionDriver.cpp

motorAppliedMotion_LIBS += motor asyn
motorAppliedMotion_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

