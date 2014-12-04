#ifndef AHI_Drivers_misc_h
#define AHI_Drivers_h

#include <DriverData.h>

void SaveGPIOStatus(struct CardData *card);
void RestoreGPIOStatus(struct CardData *card);


unsigned char GetGPIOData(struct CardData *card);
void SetGPIOData(struct CardData *card, unsigned char data);
void WritePartialMask8(IOPCIDevice *dev, IOMemoryMap *base, unsigned char reg, unsigned char shift, unsigned char mask, unsigned char val);
void ClearMask8(IOPCIDevice *dev, IOMemoryMap *base, unsigned char reg, unsigned char mask);
void WriteMask8(IOPCIDevice *dev, IOMemoryMap *base, unsigned char reg, unsigned char mask);


void WritePartialMask(IOPCIDevice *dev, IOMemoryMap *base, unsigned char reg, unsigned long shift, unsigned long mask, unsigned long val);
void ClearMask(IOPCIDevice *dev, IOMemoryMap *base, unsigned long reg, unsigned long mask);
void WriteMask(IOPCIDevice *dev, IOMemoryMap *base, unsigned long reg, unsigned long mask);
void MicroDelay(unsigned int val);

void codec_write(struct CardData *card, unsigned short reg, unsigned short val);
unsigned short codec_read(struct CardData *card, unsigned short reg);

void wm_put(struct CardData *card, IOMemoryMap *base, unsigned short reg, unsigned short val);
void update_spdif_bits(struct CardData *card, unsigned short val);
void update_spdif_rate(struct CardData *card, unsigned short rate);

void WriteI2C(IOPCIDevice *dev, struct CardData *card, unsigned chip_address, unsigned char reg, unsigned char data);

void SaveGPIO(IOPCIDevice *dev, struct CardData* card);
void RestoreGPIO(IOPCIDevice *dev, struct CardData* card);
void SetGPIODir(IOPCIDevice *dev, struct CardData* card, unsigned long data);

unsigned char ReadCCI(struct CardData *card, unsigned char address);
void WriteCCI(struct CardData *card, unsigned char address, unsigned char data);
void spdif_write(struct CardData *card, unsigned char bits);

int card_init(struct CardData *card);

struct CardData*
AllocDriverData( IOPCIDevice*    dev, struct CardData *card );

void
FreeDriverData( struct CardData* card );

void
SaveMixerState( struct CardData* card );

void
RestoreMixerState( struct CardData* card );

void
UpdateMonitorMixer( struct CardData* card );

ULONG
SamplerateToLinearPitch( ULONG samplingrate );

void *pci_alloc_consistent(size_t size, void *NonAlignedAddress);

void pci_free_consistent(void* addr);

#endif /* AHI_Drivers_misc_h */
