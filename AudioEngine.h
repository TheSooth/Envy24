#ifndef _Envy24AudioEngine_H
#define _Envy24AudioEngine_H

#include <IOKit/audio/IOAudioEngine.h>

#include "AudioDevice.h"

#define Envy24AudioEngine com_Envy24AudioEngine

class IOFilterInterruptEventSource;
class IOInterruptEventSource;

class Envy24AudioEngine : public IOAudioEngine
{
    OSDeclareDefaultStructors(Envy24AudioEngine)
    
public:

    virtual bool	init(struct CardData* i_card);
    virtual void	free();
    
    virtual bool	initHardware(IOService *provider);
    virtual void	stop(IOService *provider);
	
	UInt32 lookUpFrequencyBits(UInt32 Frequency, const UInt32* FreqList, const UInt32* FreqBitList, UInt32 ListSize, UInt32 Default);
    virtual void	dumpRegisters();

	virtual IOAudioStream *createNewAudioStream(IOAudioStreamDirection direction, void *sampleBuffer, UInt32 sampleBufferSize, UInt32 channel, UInt32 channels);

    virtual IOReturn performAudioEngineStart();
    virtual IOReturn performAudioEngineStop();
    
    virtual UInt32 getCurrentSampleFrame();
    
    virtual IOReturn performFormatChange(IOAudioStream *audioStream, const IOAudioStreamFormat *newFormat, const IOAudioSampleRate *newSampleRate);

    virtual IOReturn clipOutputSamples(const void *mixBuf, void *sampleBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    virtual IOReturn convertInputSamples(const void *sampleBuf, void *destBuf, UInt32 firstSampleFrame, UInt32 numSampleFrames, const IOAudioStreamFormat *streamFormat, IOAudioStream *audioStream);
    
    static void interruptHandler(OSObject *owner, IOInterruptEventSource *source, int count);
    static bool interruptFilter(OSObject *owner, IOFilterInterruptEventSource *source);
    virtual void filterInterrupt(int index);
	
private:
	struct CardData				   *card;
	UInt32							currentSampleRate;
    
	SInt32							*inputBuffer;
    SInt32							*outputBuffer;
    
	IOPhysicalAddress               physicalAddressInput;
	IOPhysicalAddress               physicalAddressOutput;
    
    IOFilterInterruptEventSource	*interruptEventSource;
	UInt8 lastirq;
	
	void * IOMallocContiguousWithLimit(vm_size_t size,
									   mach_vm_address_t          physicalMask,
									   IOPhysicalAddress * physicalAddress);
};

#endif /* _Envy24AudioEngine_H */
