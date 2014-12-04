
#include "misc.h"
#include "DriverData.h"
#include "regs.h"
#include "ak_codec.h"
#include "Phase88.h"
#include <IOKit/IOLib.h>

#define SND_CS8404


static void Phase88_Start(struct CardData *card)
{

	SaveGPIOStatus(card);

   WriteCCI(card, CCI_GPIO_DIR, card->gpio_dir | PHASE88_RW); // prepare for write
   WriteCCI(card, CCI_GPIO_MASK, ~PHASE88_RW); // prevent RW from switching
}


static void Phase88_Stop(struct CardData *card)
{
	RestoreGPIOStatus(card);
}



// Set the direction of the CLK and SDA lines:
// For sending, use 1, 1
static void  Phase88_SetDir_CLK_SDA(struct CardData *card, int clock, int data)
{
	unsigned char mask = 0;

	if (clock)
		mask |= PHASE88_CLOCK; /* write SCL */
	if (data)
		mask |= PHASE88_DATA; /* write SDA */
   
   WriteCCI(card, CCI_GPIO_DIR, (ReadCCI(card, CCI_GPIO_DIR) & ~(PHASE88_CLOCK | PHASE88_DATA)) | mask); // tbd: mag weg?
   WriteCCI(card, CCI_GPIO_MASK, ~mask);
}



// Write data to the SDA line and clock to the SCL
static void Phase88_Write_CLK_SDA(struct CardData *card, int clk, int data)
{
	unsigned char tmp = 0;

	if (clk)
		tmp |= PHASE88_CLOCK;
	if (data)
		tmp |= PHASE88_DATA;
	
   WriteCCI(card, CCI_GPIO_DATA, tmp);
	
   MicroDelay(5);
}


static int Phase88_GetClock(struct CardData *card)
{
	unsigned char tmp;
   
   tmp = ReadCCI(card, CCI_GPIO_DATA);
   
   if (tmp & PHASE88_CLOCK)
      return 1;
   else
      return 0;
}


static int Phase88_GetDataBit(struct CardData *card, int ack)
{
	int bit;

	//set RW to read mode
   WriteCCI(card, CCI_GPIO_MASK, ~PHASE88_RW); // clear rw mask first
   WriteCCI(card, CCI_GPIO_DATA, 0); // set to read

	if (ack)
		MicroDelay(5);

   // get data bit from the GPIO pines
	bit = ReadCCI(card, CCI_GPIO_DATA) & PHASE88_DATA ? 1 : 0;
   
	/* set RW pin to high */
   WriteCCI(card, CCI_GPIO_DATA, PHASE88_RW); // set to write
	
   /* reset write mask */
   WriteCCI(card, CCI_GPIO_MASK, ~PHASE88_CLOCK);

	return bit;
}



// ---------------------------------------


/*
 * AK4524 access
 */

/* AK4524 chip select; address 0x48 bit 0-3 */
static int Phase88_CS(struct CardData *card, unsigned char chip_mask)
{
	unsigned char data, ndata;

	if (chip_mask > 0x0f)
      return 0;
   
	if (ReadBytesI2C(card, card->i2c_out_addr, &data, 1) != 1)
		goto __error;

	ndata = (data & 0xf0) | chip_mask;
	if (ndata != data)
		if (WriteBytesI2C(card, card->i2c_out_addr, &ndata, 1) != 1)
			goto __error;
   
	return 0;

     __error:
	return -1;
}


/* start callback for PHASE88, needs to select a certain chip mask */
void Phase88_ak4524_lock(struct CardData *card, int chip)
{
	unsigned char tmp;
   
	if (Phase88_CS(card, ~(1 << chip) & 0x0f) < 0)
		IOLog("Can't select chip\n");
	
   SaveGPIOStatus(card);
   
	tmp = PHASE88_DATA | PHASE88_CLOCK | PHASE88_RW;
   
	WriteCCI(card, CCI_GPIO_DIR, card->gpio_dir | tmp);
	WriteCCI(card, CCI_GPIO_MASK, ~tmp);
}


/* stop callback for PHASE88, needs to deselect chip mask */
void Phase88_ak4524_unlock(struct CardData *card, int chip)
{
	RestoreGPIOStatus(card);
   MicroDelay(1);

	Phase88_CS(card, 0x0f);
}


static struct I2C_bit_ops Phase88_bit_ops = {
	Phase88_Start,
	Phase88_Stop,
	Phase88_SetDir_CLK_SDA,
	Phase88_Write_CLK_SDA,
	Phase88_GetClock,
	Phase88_GetDataBit,
};


void Phase88_akm4xxx_write(struct CardData *card, int chip, unsigned char addr, unsigned char data)
{
	unsigned int tmp;
	int idx;
	unsigned int addrdata;
    
	if (!(chip >= 0 && chip < 4))
        return;
    
    Phase88_ak4524_lock(card, chip);
    
	tmp = ReadCCI(card, CCI_GPIO_DATA);
	tmp |= PHASE88_RW;
    
    
	/* build I2C address + data byte */
	addrdata = (2 << 6) | 0x20 | (addr & 0x1f);
	addrdata = (addrdata << 8) | data;
    
	for (idx = 15; idx >= 0; idx--) {
        
		/* drop clock */
		tmp &= ~PHASE88_CLOCK;
		WriteCCI(card, CCI_GPIO_DATA, tmp);
        MicroDelay(1);
        
		/* set data */
		if (addrdata & (1 << idx))
			tmp |= PHASE88_DATA;
		else
			tmp &= ~PHASE88_DATA;
		WriteCCI(card, CCI_GPIO_DATA, tmp);
		MicroDelay(1);
        
		/* raise clock */
		tmp |= PHASE88_CLOCK;
		WriteCCI(card, CCI_GPIO_DATA, tmp);
		MicroDelay(1);
	}
    
	/* assert a cs pulse to trigger */
	WriteCCI(card, CCI_GPIO_DATA, tmp);
	MicroDelay(1);
    
    Phase88_ak4524_unlock(card, chip);
}


int Phase88_Init(struct CardData *card)
{
    int err = 0;
    
	/* create i2c devices */
   card->i2c_cs8404 = AllocI2C(0x40 >> 1);
   card->i2c_in_addr = AllocI2C(0x46 >> 1);
   card->i2c_out_addr = AllocI2C(0x48 >> 1);
   
   card->bit_ops = &Phase88_bit_ops;
   
   /* Check if the front module is connected */
	if ((err = Phase88_CS(card, 0x0f)) < 0) {
	   IOLog("Front module not connected?\n");
      return err;
      }
    
    unsigned char *ptr, reg, data, *inits;
    
    static unsigned char inits_phase88_ak4524[] = {
		0x00, 0x07, /* 0: all power up */
		0x01, 0x00, /* 1: ADC/DAC reset */
		0x02, 0x60, /* 2: 24bit I2S */
		0x03, 0x19, /* 3: deemphasis off */
		0x01, 0x03, /* 1: ADC/DAC enable */
		
		0x04, 0x80, /* 4: ADC IPGA gain 0dB */
		0x05, 0x80, /* 5: ADC IPGA gain 0dB */
		0x06, 0x7E, /* 6: DAC left muted */
		0x07, 0x7E, /* 7: DAC right muted */
		0xff, 0xff
	};
    
    inits = inits_phase88_ak4524;
    
    for (int chip = 0; chip < 4; chip++)
    {
        ptr = inits;
        while (*ptr != 0xff)
        {
            reg = *ptr++;
            data = *ptr++;
            Phase88_akm4xxx_write(card, chip, reg, data);
        }
    }
    

   #if 0
	/* set up SPDIF interface */
		/* set up CS8404 */
		ice->spdif.ops.open = ews88_open_spdif;
		ice->spdif.ops.setup_rate = ews88_setup_spdif;
		ice->spdif.ops.default_get = ews88_spdif_default_get;
		ice->spdif.ops.default_put = ews88_spdif_default_put;
		ice->spdif.ops.stream_get = ews88_spdif_stream_get;
		ice->spdif.ops.stream_put = ews88_spdif_stream_put;
		/* Set spdif defaults */
		snd_ice1712_ews_cs8404_spdif_write(ice, ice->spdif.cs8403_bits);
   #endif

	return 0;
}
