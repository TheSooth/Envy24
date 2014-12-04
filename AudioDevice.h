#include "DriverData.h"

#ifndef _Envy24AudioDevice_H
#define _Envy24AudioDevice_H

#include <IOKit/audio/IOAudioDevice.h>

#ifdef DEBUG
	#define DBGPRINT(msg,...)    IOLog(msg, ##__VA_ARGS__)
#else
	#define DBGPRINT(msg,...)
#endif

class IOPCIDevice;
class IOMemoryMap;

#define Envy24AudioDevice com_audio_evolution_driver_Envy24


class Envy24AudioDevice : public IOAudioDevice
{
    friend class Envy24AudioEngine;
    
    OSDeclareDefaultStructors(Envy24AudioDevice)
    
	struct CardData *card;

    virtual bool	initHardware(IOService *provider);
    virtual bool	createAudioEngine();
    virtual void	free();
	virtual IOReturn performPowerStateChange(IOAudioDevicePowerState oldPowerState, 
											 IOAudioDevicePowerState newPowerState, 
											 UInt32 *microsecondsUntilComplete);
	
	void dumpRegisters();
    
	static IOReturn volumeChangeHandler(IOService *target, IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn volumeChanged(IOAudioControl *volumeControl, SInt32 oldValue, SInt32 newValue);
    
    static IOReturn outputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn outputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);

    static IOReturn gainChangeHandler(IOService *target, IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn gainChanged(IOAudioControl *gainControl, SInt32 oldValue, SInt32 newValue);
    
    static IOReturn inputMuteChangeHandler(IOService *target, IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
    virtual IOReturn inputMuteChanged(IOAudioControl *muteControl, SInt32 oldValue, SInt32 newValue);
};

#endif /* _Envy24AudioDevice_H */
