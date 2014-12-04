
#include "regs.h"
#include "misc.h"
#include "DriverData.h"

#include <IOKit/IOLib.h>
#include <IOKit/audio/IOAudioDevice.h>
#include <IOKit/audio/IOAudioDefines.h>

#include "Phase88.h"
#include "cs8427.h"

#define MAUDIO_2496_ID 0x121434D6
#define MAUDIO_1010_ID 0x121430d6
#define MAUDIO_1010LT_ID 0x12143bd6
#define MAUDIO_DELTA44_ID 0x121433d6
#define MAUDIO_DELTA66_ID 0x121432d6
#define MAUDIO_DELTA_410_ID 0x121438d6
#define PHASE_88 0x3b155111
#define EWS88MT 0x3b151511
#define EWS88MT_2 0x3b152511
#define TS88PCI 0x3b157c11
#define EWX2496_ID 0x3b153011


/* Public functions in main.c */
void card_cleanup(struct CardData *card);
static void CreateParmsFor2496(struct CardData *card);
static void CreateParmsForDelta4466(struct CardData *card);
static void CreateParmsForDelta1010LT(struct CardData *card);
static void CreateParmsForEWX2496(struct CardData *card);
static void CreateParmsForDelta410(struct CardData *card);
void InitDigitalOut(struct CardData *card);

#define BIT_DEPTH			32




unsigned char ReadCCI(struct CardData *card, unsigned char address)
{
   card->pci_dev->ioWrite8(CCS_ENVY_INDEX, address, card->iobase);
   return card->pci_dev->ioRead8(CCS_ENVY_DATA, card->iobase);
}


void WriteCCI(struct CardData *card, unsigned char address, unsigned char data)
{
   card->pci_dev->ioWrite8(CCS_ENVY_INDEX, address, card->iobase);
   card->pci_dev->ioWrite8(CCS_ENVY_DATA, data, card->iobase);
}


void WritePartialMask8(IOPCIDevice *dev, IOMemoryMap *map, unsigned char reg, unsigned char shift, unsigned char mask, unsigned char val)
{
    UInt8 tmp;
    
    tmp = dev->ioRead8(reg, map);
    tmp &= ~(mask << shift);
    tmp |= val << shift;
    dev->ioWrite8(reg, tmp, map);
}


void ClearMask8(IOPCIDevice *dev, IOMemoryMap *map, unsigned char reg, unsigned char mask)
{
    UBYTE tmp;
    
    tmp = dev->ioRead8(reg, map);
    tmp &= ~mask;
    dev->ioWrite8(reg, tmp, map);
}


void WriteMask8(IOPCIDevice *dev, IOMemoryMap *map, unsigned char reg, unsigned char mask)
{
    UBYTE tmp;
    
    tmp = dev->ioRead8(reg, map);
    tmp |= mask;
    dev->ioWrite8(reg, tmp, map);
}


void WritePartialMask(IOPCIDevice *dev, IOMemoryMap *map, unsigned char reg, unsigned long shift, unsigned long mask, unsigned long val)
{
    ULONG tmp;
    
    tmp = dev->ioRead32(reg, map);
    tmp &= ~(mask << shift);
    tmp |= val << shift;
    dev->ioWrite32(reg, tmp, map);
}


void ClearMask(IOPCIDevice *dev, IOMemoryMap *map, unsigned long reg, unsigned long mask)
{
    ULONG tmp;
    
    tmp = dev->ioRead32(reg, map);
    tmp &= ~mask;
    dev->ioWrite32(reg, tmp, map);
}


void WriteMask(IOPCIDevice *dev, IOMemoryMap *map, unsigned long reg, unsigned long mask)
{
    ULONG tmp;
    
    tmp = dev->ioRead32(reg, map);
    tmp |= mask;
    dev->ioWrite32(reg, tmp, map);
}

void SetGPIOData(struct CardData *card, unsigned char data)
{
    WriteCCI(card, CCI_GPIO_DATA, data);
}


void SaveGPIOStatus(struct CardData *card)
{
   card->gpio_dir = ReadCCI(card, CCI_GPIO_DIR);
   card->gpio_data = ReadCCI(card, CCI_GPIO_DATA);
}


void RestoreGPIOStatus(struct CardData *card)
{
   WriteCCI(card, CCI_GPIO_DIR, card->gpio_dir);
   WriteCCI(card, CCI_GPIO_DATA, card->gpio_data);
}


unsigned char GetGPIOData(struct CardData *card)
{
    return ReadCCI(card, CCI_GPIO_DATA);
}


void WaitForI2C(IOPCIDevice *dev, struct CardData *card)
{
    int Counter = 0;
    
	for (Counter = 0; Counter < 1000; Counter++)
	{
		UInt8 status = dev->ioRead8(CCS_I2C_STATUS, card->iobase);
	    
		if ((status & CCS_I2C_BUSY) == 0)
        {
		    //IOLog("Counter was %d\n", Counter);
            return;
	    }
		
		MicroDelay(32);
	}

	IOLog("WaitForI2C() failed!\n");
	IOSleep(1000);
}


unsigned char ReadI2C(IOPCIDevice *dev, struct CardData *card, unsigned char addr)
{
    UInt8 val;

	WaitForI2C(dev, card);
    dev->ioWrite8(CCS_I2C_ADDR, addr, card->iobase);
	dev->ioWrite8(CCS_I2C_DEV_ADDRESS, 0xA0, card->iobase);
	WaitForI2C(dev, card);
    val = dev->ioRead8(CCS_I2C_DATA, card->iobase);
	
    return val;
}


/*
 * CS8427 via SPI mode (for Audiophile), emulated I2C
 */

/* send 8 bits */
static void ap_cs8427_write_byte(struct CardData *card, unsigned char data, unsigned char tmp)
{
	int idx;
    
	for (idx = 7; idx >= 0; idx--) {
		tmp &= ~(ICE1712_DELTA_AP_DOUT|ICE1712_DELTA_AP_CCLK);
		if (data & (1 << idx))
			tmp |= ICE1712_DELTA_AP_DOUT;
		WriteCCI(card, CCI_GPIO_DATA, tmp);
		MicroDelay(5);
		tmp |= ICE1712_DELTA_AP_CCLK;
		WriteCCI(card, CCI_GPIO_DATA, tmp);
		MicroDelay(5);
	}
}


/* read 8 bits */
static unsigned char ap_cs8427_read_byte(struct CardData *card, unsigned char tmp)
{
	unsigned char data = 0;
	int idx;
	
	for (idx = 7; idx >= 0; idx--) {
		tmp &= ~ICE1712_DELTA_AP_CCLK;
		WriteCCI(card, CCI_GPIO_DATA, tmp);
		MicroDelay(5);
		if (ReadCCI(card, CCI_GPIO_DATA) & ICE1712_DELTA_AP_DIN)
			data |= 1 << idx;
		tmp |= ICE1712_DELTA_AP_CCLK;
		WriteCCI(card, CCI_GPIO_DATA, tmp);
		MicroDelay(5);
	}
	return data;
}


/* assert chip select */
static unsigned char ap_cs8427_codec_select(struct CardData *card)
{
	unsigned char tmp;
	tmp = ReadCCI(card, CCI_GPIO_DATA);
	/*switch (ice->eeprom.subvendor) {
        case ICE1712_SUBDEVICE_DELTA1010E:
        case ICE1712_SUBDEVICE_DELTA1010LT:
            tmp &= ~ICE1712_DELTA_1010LT_CS;
            tmp |= ICE1712_DELTA_1010LT_CCLK | ICE1712_DELTA_1010LT_CS_CS8427;
            break;
        case ICE1712_SUBDEVICE_AUDIOPHILE:
        case ICE1712_SUBDEVICE_DELTA410:*/
            tmp |= ICE1712_DELTA_AP_CCLK | ICE1712_DELTA_AP_CS_CODEC;
            tmp &= ~ICE1712_DELTA_AP_CS_DIGITAL;
            /*break;
        case ICE1712_SUBDEVICE_VX442:
            tmp |= ICE1712_VX442_CCLK | ICE1712_VX442_CODEC_CHIP_A | ICE1712_VX442_CODEC_CHIP_B;
            tmp &= ~ICE1712_VX442_CS_DIGITAL;
            break;
	}*/
	WriteCCI(card, CCI_GPIO_DATA, tmp);
	MicroDelay(5);
    
    IOLog("ap_cs8427_codec_select done!\n");
	return tmp;
}

/* deassert chip select */
static void ap_cs8427_codec_deassert(struct CardData *card, unsigned char tmp)
{
	/*switch (ice->eeprom.subvendor) {
        case ICE1712_SUBDEVICE_DELTA1010E:
        case ICE1712_SUBDEVICE_DELTA1010LT:
            tmp &= ~ICE1712_DELTA_1010LT_CS;
            tmp |= ICE1712_DELTA_1010LT_CS_NONE;
            break;
        case ICE1712_SUBDEVICE_AUDIOPHILE:
        case ICE1712_SUBDEVICE_DELTA410:*/
            tmp |= ICE1712_DELTA_AP_CS_DIGITAL;
            /*break;
        case ICE1712_SUBDEVICE_VX442:
            tmp |= ICE1712_VX442_CS_DIGITAL;
            break;
	}*/
	WriteCCI(card, CCI_GPIO_DATA, tmp);
}

/* sequential write */
static int ap_cs8427_sendbytes(struct CardData *card, unsigned char *bytes, int count)
{
	int res = count;
	unsigned char tmp;
    
	//mutex_lock(&ice->gpio_mutex);
	tmp = ap_cs8427_codec_select(card);
	ap_cs8427_write_byte(card, (0x10 << 1) | 0, tmp); /* address + write mode */
	while (count-- > 0)
		ap_cs8427_write_byte(card, *bytes++, tmp);
	ap_cs8427_codec_deassert(card, tmp);
	//mutex_unlock(&ice->gpio_mutex);
	return res;
}


/* sequential read */
static int ap_cs8427_readbytes(struct CardData *card, unsigned char *bytes, int count)
{
	int res = count;
	unsigned char tmp;
	
	//mutex_lock(&ice->gpio_mutex);
	tmp = ap_cs8427_codec_select(card);
	ap_cs8427_write_byte(card, (0x10 << 1) | 1, tmp); /* address + read mode */
	while (count-- > 0)
		*bytes++ = ap_cs8427_read_byte(card, tmp);
	ap_cs8427_codec_deassert(card, tmp);
	//mutex_unlock(&ice->gpio_mutex);
	return res;
}

#if 0
static int ap_cs8427_probeaddr(struct CardData *card, unsigned short addr)
{
	if (addr == 0x10)
		return 1;
	return -1;
}
#endif


int snd_cs8427_reg_write(struct CardData *card, unsigned char reg,
						 unsigned char val)
{
	int err;
	unsigned char buf[2];
	
	buf[0] = reg & 0x7f;
	buf[1] = val;
	if ((err = ap_cs8427_sendbytes(card, buf, 2)) != 2) {
		IOLog("unable to send bytes 0x%02x:0x%02x to CS8427 (%i)\n", buf[0], buf[1], err);
		return err < 0 ? err : -1;
	}
	return 0;
}


static int snd_cs8427_reg_read(struct CardData *card, unsigned char reg)
{
	int err;
	unsigned char buf;
	
	if ((err = ap_cs8427_sendbytes(card, &reg, 1)) != 1) {
		IOLog("unable to send register 0x%x byte "
				   "to CS8427\n", reg);
		return err < 0 ? err : -1;
	}
	if ((err = ap_cs8427_readbytes(card, &buf, 1)) != 1) {
		IOLog("unable to read register 0x%x byte "
				   "from CS8427\n", reg);
		return err < 0 ? err : -1;
	}
	return buf;
}



/*
 * Reset the chip using run bit, also lock PLL using ILRCK and
 * put back AES3INPUT. This workaround is described in latest
 * CS8427 datasheet, otherwise TXDSERIAL will not work.
 */
#if 0
static void snd_cs8427_reset(struct CardData *card)
{
	//struct cs8427 *chip;
	//unsigned long end_time;
	int aes3input = 0;
	unsigned char val;
	
	//if (snd_BUG_ON(!cs8427))
	//	return;
	//chip = cs8427->private_data;
	//snd_i2c_lock(cs8427->bus);
	//if ((chip->regmap[CS8427_REG_CLOCKSOURCE] & CS8427_RXDAES3INPUT) ==
	  //  CS8427_RXDAES3INPUT)  /* AES3 bit is set */
		//aes3input = 1;
	val = 0;
	snd_cs8427_reg_write(card, CS8427_REG_CLOCKSOURCE, val);
	MicroDelay(200);
	val |= CS8427_RUN | CS8427_RXDILRCK;
	snd_cs8427_reg_write(card, CS8427_REG_CLOCKSOURCE, val);
	MicroDelay(200);
	//snd_i2c_unlock(cs8427->bus);
	/*end_time = jiffies + chip->reset_timeout;
	while (time_after_eq(end_time, jiffies)) {
		//snd_i2c_lock(cs8427->bus);
		data = snd_cs8427_reg_read(card, CS8427_REG_RECVERRORS);
		//snd_i2c_unlock(cs8427->bus);
		if (!(data & CS8427_UNLOCK))
			break;
		schedule_timeout_uninterruptible(1);
	}*/
	//snd_i2c_lock(cs8427->bus);
	val &= ~CS8427_RXDMASK;
	//if (aes3input)
	//	chip->regmap[CS8427_REG_CLOCKSOURCE] |= CS8427_RXDAES3INPUT;
	snd_cs8427_reg_write(card, CS8427_REG_CLOCKSOURCE, val);
	//snd_i2c_unlock(cs8427->bus);
}
#endif


/******************************************************************************
** DriverData allocation ******************************************************
******************************************************************************/

// This code used to be in _AHIsub_AllocAudio(), but since we're now
// handling CAMD support too, it needs to be done at driver loading
// time.

struct CardData*
AllocDriverData( IOPCIDevice *    dev, struct CardData *card )
{
  card->SavedMask = 0;
  card->SPDIF_RateSupported = false;

  /* Initialize chip */
  if( card_init( card ) < 0 )
  {
    return NULL;
  }

  card->card_initialized = TRUE;

  card->input          = 0;
  card->output         = 0;
 
  return card;
}


/******************************************************************************
** DriverData deallocation ****************************************************
******************************************************************************/

// And this code used to be in _AHIsub_FreeAudio().

void
FreeDriverData( struct CardData* card )
{
  if( card != NULL )
  {


  }
}


static unsigned char initvals1[] = {
    CS8427_REG_CONTROL1 | CS8427_REG_AUTOINC,
    /* CS8427_REG_CONTROL1: RMCK to OMCK, valid PCM audio, disable mutes,
     TCBL=output */
    CS8427_SWCLK | CS8427_TCBLDIR,
    /* CS8427_REG_CONTROL2: hold last valid audio sample, RMCK=256*Fs,
     normal stereo operation */
    0x00,
    /* CS8427_REG_DATAFLOW: output drivers normal operation, Tx<=serial,
     Rx=>serial */
    CS8427_TXDSERIAL | CS8427_SPDAES3RECEIVER,
    /* CS8427_REG_CLOCKSOURCE: Run off, CMCK=256*Fs,
     output time base = OMCK, input time base = recovered input clock,
     recovered input clock source is ILRCK changed to AES3INPUT
     (workaround, see snd_cs8427_reset) */
    CS8427_RXDILRCK,
    /* CS8427_REG_SERIALINPUT: Serial audio input port data format = I2S,
     24-bit, 64*Fsi */
    CS8427_SIDEL | CS8427_SILRPOL,
    /* CS8427_REG_SERIALOUTPUT: Serial audio output port data format
     = I2S, 24-bit, 64*Fsi */
    CS8427_SODEL | CS8427_SOLRPOL,
};

static unsigned char initvals2[] = {
	CS8427_REG_RECVERRMASK | CS8427_REG_AUTOINC,
	/* CS8427_REG_RECVERRMASK: unmask the input PLL clock, V, confidence,
	 biphase, parity status bits */
	/* CS8427_UNLOCK | CS8427_V | CS8427_CONF | CS8427_BIP | CS8427_PAR,*/
	0xff, /* set everything */
	/* CS8427_REG_CSDATABUF:
	 Registers 32-55 window to CS buffer
	 Inhibit D->E transfers from overwriting first 5 bytes of CS data.
	 Inhibit D->E transfers (all) of CS data.
	 Allow E->F transfer of CS data.
	 One byte mode; both A/B channels get same written CB data.
	 A channel info is output to chip's EMPH* pin. */
	CS8427_CBMR | CS8427_DETCI,
	/* CS8427_REG_UDATABUF:
	 Use internal buffer to transmit User (U) data.
	 Chip's U pin is an output.
	 Transmit all O's for user data.
	 Inhibit D->E transfers.
	 Inhibit E->F transfers. */
	CS8427_UD | CS8427_EFTUI | CS8427_DETUI,
};



#define SNDRV_PCM_DEFAULT_CON_SPDIF	(IEC958_AES0_CON_EMPHASIS_NONE|\
(IEC958_AES1_CON_ORIGINAL<<8)|\
(IEC958_AES1_CON_PCM_CODER<<8)|\
(IEC958_AES3_CON_FS_48000<<24))


// ----------- OSS

#define BIT0		0x01
#define BIT1		0x02
#define BIT2		0x04
#define BIT3		0x08
#define BIT4		0x10
#define BIT5		0x20
#define BIT6		0x40
#define BIT7		0x80

#define CDC_CLK 1		/* Clock input to the CODEC's, rising edge clocks data. */
#define CDC_DIN 2		/* Data input to Envy from the CODEC. */
#define CDC_DOUT 3		/* Data output from Envy to the CODEC. */
#define DIG_CS 4		/* Chip select (0=select) for the SPDIF tx/rx. */
#define CDC_CS 5		/* Chip select (0=select) for the CODEC. */
#define D410_MUTE	7	/* Delta 410 codec mute */
#define CS_ASSERT 0		/* Asserted chip select (selects are inverted). */
#define CS_RELEASE 1		/* Idle chip select (selects are inverted). */


#define CS8_CLK CDC_CLK
#define CS8_DIN CDC_DIN
#define CS8_DOUT CDC_DOUT
#define CS8_CS DIG_CS
#define CS_1 CS_ASSERT
#define CS_0 CS_RELEASE
#define CS8_ADDR 0x20		/* Chip SPI/I2C address */
#define CS8_RD 0x01
#define CS8_WR 0x00

#define EWX_DIG_CS				0
#define EWX_CDC_CLK				5
#define EWX_CDC_DOUT			4
#define EWX_CDC_DIN 			EWX_CDC_DOUT
#define EWX_IIC_WRITE			3
#define CX_ASSERT 0		/* Asserted chip select (selects are inverted). */
#define CX_RELEASE 1		/* Idle chip select (selects are inverted). */

void
WriteGPIObit (struct CardData *card, int sel, int what)
{
	unsigned char gpio;
	
	gpio = ReadCCI(card, 0x20);
	gpio &= ~(1 << sel);
	gpio |= (what << sel);
	WriteCCI(card, 0x20, gpio);
}

int
ReadGPIObit (struct CardData *card, int sel)
{
	unsigned char gpio;
	
	gpio = ReadCCI(card, 0x20);
	return !!(gpio & (1 << sel));
}

#define write_cs8427_spdif_reg write_ap_spdif_reg


static void
write_ap_spdif_reg (struct CardData *card, int bRegister, int bData)
/*
 *****************************************************************************
 *  Writes a byte to a specific register of the Delta-AP S/PDIF chip.
 *  Register must be (0..55).
 ****************************************************************************/
{
	unsigned char bMask;
	unsigned char bSPI;
	
	/* Assert the CODEC chip select and wait at least 150 nS. */
	/* */
	WriteGPIObit (card, CDC_CS, CS_0);
	WriteGPIObit (card, CS8_CS, CS_1);
	
	/* Write the SPI address/cmd byte. */
	/* */
	bSPI = CS8_ADDR | CS8_WR;
	/* */
	for (bMask = 0x80; bMask; bMask = (bMask >> 1) & 0x7F)
    {
		/* Drop SPI clock low. */
		WriteGPIObit (card, CS8_CLK, 0);
		
		/* Write current data bit. */
		if (bMask & bSPI)
			WriteGPIObit (card, CS8_DOUT, 1);
		else
			WriteGPIObit (card, CS8_DOUT, 0);
		
		/* Raise SPI clock to "clock data in". */
		WriteGPIObit (card, CS8_CLK, 1);
    }
	
	/* Write the address (MAP) byte. */
	/* */
	for (bMask = 0x80; bMask; bMask = (bMask >> 1) & 0x7F)
    {
		/* Drop SPI clock low. */
		WriteGPIObit (card, CS8_CLK, 0);
		
		/* Write current data bit. */
		if (bMask & bRegister)
			WriteGPIObit (card, CS8_DOUT, 1);
		else
			WriteGPIObit (card, CS8_DOUT, 0);
		
		/* Raise SPI clock to "clock data in". */
		WriteGPIObit (card, CS8_CLK, 1);
    }
	
	
	/* Write the data byte. */
	/* */
	for (bMask = 0x80; bMask; bMask = (bMask >> 1) & 0x7F)
    {
		/* Drop SPI clock low. */
		WriteGPIObit (card, CS8_CLK, 0);
		
		/* Write current data bit. */
		if (bMask & bData)
			WriteGPIObit (card, CS8_DOUT, 1);
		else
			WriteGPIObit (card, CS8_DOUT, 0);
		
		/* Raise SPI clock to "clock data in". */
		WriteGPIObit (card, CS8_CLK, 1);
    }
	
	/* De-assert chip select. */
	/* */
	WriteGPIObit (card, CS8_CS, CS_0);
}


static int
read_cs8427_spdif_reg (struct CardData *card, int bRegister)
/*
 *****************************************************************************
 *  Reads a byte from a specific CS8427 register.
 ****************************************************************************/
{
	unsigned char bMask;
	unsigned char bRet = 0;
	unsigned char bSPI;
	
	/****** WRITE MAP ADDRESS FIRST ******/
	
	/* Drop the chip select low. */
	/* Wait at least 150 nS. */
	/* */
	WriteGPIObit (card, DIG_CS, CS_ASSERT);
	
	/* Write the SPI address/cmd byte. */
	/* */
	bSPI = CS8_ADDR + CS8_WR;	/* SPI address field plus WRITE operation. */
	/* */
	for (bMask = 0x80; bMask; bMask = (bMask >> 1) & 0x7F)
    {
		/* Drop clock (GPIO5) low. */
		WriteGPIObit (card, CDC_CLK, 0);
		
		/* Write current data bit. */
		if (bMask & bSPI)
			WriteGPIObit (card, CDC_DOUT, 1);
		else
			WriteGPIObit (card, CDC_DOUT, 0);
		
		/* Raise clock (GPIO5). */
		WriteGPIObit (card, CDC_CLK, 1);
    }
	
	
	/* Write the address (MAP) byte. */
	/* */
	for (bMask = 0x80; bMask; bMask = (bMask >> 1) & 0x7F)
    {
		/* Drop clock (GPIO5) low. */
		WriteGPIObit (card, CDC_CLK, 0);
		
		/* Write current data bit. */
		if (bMask & bRegister)
			WriteGPIObit (card, CDC_DOUT, 1);
		else
			WriteGPIObit (card, CDC_DOUT, 0);
		
		/* Raise clock (GPIO5). */
		WriteGPIObit (card, CDC_CLK, 1);
    }
	
	/* De-assert chip select(s). */
	/* */
	WriteGPIObit (card, DIG_CS, CS_RELEASE);
	
	
	/****** NOW READ THE DATA ******/
	
	/* Drop the chip select low. */
	/* Wait at least 150 nS. */
	/* */
	WriteGPIObit (card, DIG_CS, CS_ASSERT);
	
	
	/* Write the SPI address/cmd byte. */
	/* */
	bSPI = CS8_ADDR + CS8_RD;	/* SPI address field plus READ operation. */
	/* */
	for (bMask = 0x80; bMask; bMask = (bMask >> 1) & 0x7F)
    {
		/* Drop clock (GPIO5) low. */
		WriteGPIObit (card, CDC_CLK, 0);
		
		/* Write current data bit. */
		if (bMask & bSPI)
			WriteGPIObit (card, CDC_DOUT, 1);
		else
			WriteGPIObit (card, CDC_DOUT, 0);
		
		/* Raise clock (GPIO5). */
		WriteGPIObit (card, CDC_CLK, 1);
    }
	
	
	/* Read the data byte. */
	/* */
	bRet = 0;
	/* */
	for (bMask = 0x80; bMask; bMask = (bMask >> 1) & 0x7F)
    {
		/* Drop clock low. */
		WriteGPIObit (card, CDC_CLK, 0);
		
		/* Read current data bit. */
		if (ReadGPIObit (card, CDC_DIN))
			bRet |= bMask;
		
		/* Raise clock. */
		WriteGPIObit (card, CDC_CLK, 1);
    }
	
	
	/* De-assert chip selects. */
	/* */
	WriteGPIObit (card, DIG_CS, CS_RELEASE);
	
	/* Return value. */
	
	return bRet;
}



static void
lock_cs8427_spdif (struct CardData *card)
{
	write_cs8427_spdif_reg (card, 18, read_cs8427_spdif_reg (card, 18) & ~BIT5);
	write_cs8427_spdif_reg (card, 18, read_cs8427_spdif_reg (card, 18) | BIT2);
}

static void
unlock_cs8427_spdif (struct CardData *card)
{
	write_cs8427_spdif_reg (card, 18, read_cs8427_spdif_reg (card, 18) & ~BIT2);
}

static unsigned char
bitswap (unsigned char bIn)
/*
 *****************************************************************************
 *  Endian reversing routine.
 ****************************************************************************/
{
	unsigned char bOut = 0;
	unsigned char bImask = 0x01;
	unsigned char bOmask = 0x80;
	
	while (bImask)
    {
		if (bIn & bImask)
			bOut |= bOmask;
		bImask = bImask << 1;
		bOmask = (bOmask >> 1) & 0x7F;
    }
	
	return bOut;
}


static unsigned char
ReadCsByte (struct CardData *card, unsigned char bByteNum)
/*
 *****************************************************************************
 *  Reads a byte from Channel Status block buffer in CS8427.
 *
 *  bByteNum is in the range (0..23)
 *
 *  This routine assumes that CS8427 register 18 bit 5 is cleared so that the
 *  CS buffer is windowed, and that register 18 bit 2 is set so that CS output
 *  transfers are currently disabled.
 ****************************************************************************/
{
	unsigned char bTemp;
	
	/* CS block window starts at reg #32... */
	bTemp = read_cs8427_spdif_reg (card, bByteNum + 32);
	
	/* CS block access is reverse endian. */
	return bitswap (bTemp);
}

static void
WriteCsByte (struct CardData *card, unsigned char bByteNum, unsigned char bData)
/*
 *****************************************************************************
 *  Writes a byte to Channel Status block buffer in CS8427.
 *
 *  bByteNum is in the range (0..23)
 *
 *  This routine assumes that CS8427 register 18 bit 5 is cleared so that the
 *  CS buffer is windowed, and that register 18 bit 2 is set so that CS output
 *  transfers are currently disabled.
 ****************************************************************************/
{
	/* CS block access is reverse endian. */
	bData = bitswap (bData);
	
	/* CS Window starts at reg #32... */
	write_cs8427_spdif_reg (card, bByteNum + 32, bData);
}

void
InitConsumerModeCS (struct CardData *card)
{
	int i;
	
	/* Set CS8427 registers 32-55 to window CS block, and disable CS output. */
	lock_cs8427_spdif (card);
	
	/* Zero all the general CS bits. */
	/* */
	for (i = 0; i < 24; i++)
		WriteCsByte (card, i, 0x00);
	/* */
	/* Consumer (usually SPDIF or AC3) mode static bit settings. */
	/* */
	WriteCsByte (card, 0, 0x00);	/* Consumer format (bit0 = 0). */
	WriteCsByte (card, 1, 0x02);	/* Category = PCM encoder/decoder. */
	
	unlock_cs8427_spdif (card);
}

void
init_cs8427_spdif (struct CardData *card)
{
	int tmp;
	
	/* Select iunternal sync */
	write_cs8427_spdif_reg (card, 4, read_cs8427_spdif_reg (card, 4) & (~BIT0));
	/*
	 *****************************************************************************
	 *  Initializes core (mainly static) registers of the CS8427.
	 *  Returns 1 if initialization OK, otherwise 0.
	 ****************************************************************************/
	/* Assumes Envy24 GPIO's have been initialized.  They should be just fine */
	/* in the Windows driver as they are initialized from EEPROM info. */
	
	/* Verify device ID register.  Must be 0x71. */
	if ((tmp = read_cs8427_spdif_reg (card, 127)) != 0x71 && tmp != 0)
    {
		IOLog("Envy24: Unrecognized S/PDIF chip ID %02x\n",
				 read_cs8427_spdif_reg (card, 127));
		IOLog("        Hardware stalled. Please reboot and try again.\n");
		return;
    }
	
	/* Turn off RUN bit while making changes to configuration. */
	write_cs8427_spdif_reg (card, 4, read_cs8427_spdif_reg (card, 4) & (~BIT6));
	
	/* RMCK default function, set Validity, disable mutes, TCBL=output. */
	write_cs8427_spdif_reg (card, 1, 0x01);	/* validity* is BIT6. */
	
	/* Hold last valid audio sample, RMCK=256*Fs, normal stereo operation. */
	write_cs8427_spdif_reg (card, 2, 0x00);
	/* Output drivers normal operation, Tx <== serial audio port, */
	/* Rx ==> serial audio port. */
	write_cs8427_spdif_reg (card, 3, 0x0C);
	
	/* RUN off, OMCK=256xFs, output time base = OMCK, input time base = */
	/* recovered input clock, recovered input clock source is Envy24. */
	write_cs8427_spdif_reg (card, 4, 0x00);
	
	/* Serial audio input port data format = I2S. */
	write_cs8427_spdif_reg (card, 5, BIT2 | BIT0);	/* SIDEL=1, SILRPOL=1. */
	
	/* Serial audio output port data format = I2S. */
	write_cs8427_spdif_reg (card, 6, BIT2 | BIT0);	/* SODEL=1, SOLRPOL=1. */
	
	/* Turn off CS8427 interrupt stuff that we don't implement in our hardware. */
	write_cs8427_spdif_reg (card, 9, 0x00);
	write_cs8427_spdif_reg (card, 10, 0x00);
	write_cs8427_spdif_reg (card, 11, 0x00);
	write_cs8427_spdif_reg (card, 12, 0x00);
	write_cs8427_spdif_reg (card, 13, 0x00);
	write_cs8427_spdif_reg (card, 14, 0x00);
	
	/* Unmask the input PLL lock, V, confidence, biphase, parity status bits. */
	write_cs8427_spdif_reg (card, 17,
							(unsigned char) BIT4 | BIT3 | BIT2 | BIT1 | BIT0);
	
	/* Registers 32-55 window to CS buffer. */
	/* Inhibit D->E transfers from overwriting first 5 bytes of CS data. */
	/* Inhibit D->E transfers (all) of CS data. */
	/* Allow E->F transfers of CS data. */
	/* One-byte mode: both A/B channels get same written CS data. */
	/* A channel info is output to chip's EMPH* pin. */
	/* */
	write_cs8427_spdif_reg (card, 18, 0x18);
	
	/* Use internal buffer to transmit User (U) data.    */
	/* Chip's U pin is an output. */
	/* Transmit all 0's for user data. */
	/* */
	write_cs8427_spdif_reg (card, 19, 0x10);
	
	IOLog("going to init consumer mode!\n");
	
	/* Turn on chip's RUN bit, rock and roll! */
	/* */
	write_cs8427_spdif_reg (card, 4, read_cs8427_spdif_reg (card, 4) | BIT6);
	InitConsumerModeCS (card);
}


static __inline__ void
WriteCsField (struct CardData *card, unsigned char bByteNum,
			  unsigned short bMask, unsigned short bBits)
{
	/* Get current reg value. */
	unsigned char bTemp = ReadCsByte (card, bByteNum);
	
	/* Clear field to be written. */
	bTemp &= ~(bMask);
	
	/* Set new values. */
	WriteCsByte (card, bByteNum, (unsigned char) (bTemp | (bBits & bMask)));
}


static void
envy24_setup_consumer_speed (struct CardData *card, int speed)
{
	
	/*
	 * Set the sampling rate indication
	 */
	//if (card->ac3_mode)
	//	WriteCsField (card, 0, 0x02, 0x02);	/* 1:1 = 1 */
	//else
		WriteCsField (card, 0, 0x02, 0x00);	/* 1:1 = 0 */
	
	switch (speed)
    {
		case 22050L:
			WriteCsField (card, 0, 0xC0, 0x00);	/* 7:6 = 00 */
			WriteCsField (card, 3, 0x0F, 0x00);	/* 3:0 = 0000 */
			WriteCsField (card, 4, 0x0F, 0x09);	/* 3:0 = 1001 */
			break;
		case 32000L:
			WriteCsField (card, 0, 0xC0, 0xC0);	/* 7:6 = 11 */
			WriteCsField (card, 3, 0x0F, 0x03);	/* 3:0 = 0011 */
			WriteCsField (card, 4, 0x0F, 0x00);	/* 3:0 = 0000 */
			break;
		case 44100L:
			WriteCsField (card, 0, 0xC0, 0x40);	/* 7:6 = 01 */
			WriteCsField (card, 3, 0x0F, 0x00);	/* 3:0 = 0000 */
			WriteCsField (card, 4, 0x0F, 0x00);	/* 3:0 = 0000 */
			break;
		case 48000L:
			WriteCsField (card, 0, 0xC0, 0x80);	/* 7:6 = 10 */
			WriteCsField (card, 3, 0x0F, 0x02);	/* 3:0 = 0010 */
			WriteCsField (card, 4, 0x0F, 0x00);	/* 3:0 = 0000 */
			break;
		case 88200L:
			WriteCsField (card, 0, 0xC0, 0x00);	/* 7:6 = 00 */
			WriteCsField (card, 3, 0x0F, 0x00);	/* 3:0 = 0000 */
			WriteCsField (card, 4, 0x0F, 0x05);	/* 3:0 = 0101 */
			break;
		case 96000L:
			WriteCsField (card, 0, 0xC0, 0x00);	/* 7:6 = 00 */
			WriteCsField (card, 3, 0x0F, 0x00);	/* 3:0 = 0000 */
			WriteCsField (card, 4, 0x0F, 0x04);	/* 3:0 = 0100 */
			break;
		default:
			WriteCsField (card, 0, 0xC0, 0x00);	/* 7:6 = 00 */
			WriteCsField (card, 3, 0x0F, 0x00);	/* 3:0 = 0000 */
			WriteCsField (card, 4, 0x0F, 0x00);	/* 3:0 = 0000 */
			break;
    }
}


static void
setup_consumer_mode (struct CardData *card)
{
	WriteCsByte (card, 0, ReadCsByte (card, 0) & ~(0x02));	/* Set audio mode */
	WriteCsByte (card, 0, ReadCsByte (card, 0) & ~(0x38));	/* Set no emphasis */
	
	WriteCsByte (card, 0, ReadCsByte (card, 0) & ~(0x04));	/* Set "original" */
	WriteCsByte (card, 1, ReadCsByte (card, 1) | (0x80));	/* Set "original" */
	
	envy24_setup_consumer_speed (card, 44100);
}



// ------------- OSS


int card_init(struct CardData *card)
{
    IOPCIDevice *dev = (IOPCIDevice *) card->pci_dev;
    int i;
    unsigned char eeprom[128];
    
    dev->ioWrite8(CCS_ENVY_INDEX, CCI_PRO_POWER_DOWN, card->iobase);
    dev->ioWrite8(CCS_ENVY_DATA, 0xFF, card->iobase);
    dev->ioRead8(CCS_ENVY_DATA, card->iobase); // dummy read
    MicroDelay(300);
    dev->ioWrite8(CCS_ENVY_INDEX, CCI_PRO_POWER_DOWN, card->iobase);
    dev->ioWrite8(CCS_ENVY_DATA, 0x00, card->iobase);
    dev->ioRead8(CCS_ENVY_DATA, card->iobase); // dummy read
    MicroDelay(300);

    // set up CCS registers
    // reset
    dev->ioWrite8(CCS_CTRL, CCS_RESET_ALL | CCS_NATIVE_MODE, card->iobase);
    MicroDelay(300);
    dev->ioWrite8(CCS_CTRL, CCS_NATIVE_MODE, card->iobase);
    MicroDelay(300);

    dev->ioWrite8(CCS_INTR_MASK, 0xEF, card->iobase);
    dev->ioWrite8(CCS_INTR_STATUS, 0xFF, card->iobase); // clear all
	WriteMask8(card->pci_dev, card->mtbase, MT_INTR_MASK_STATUS, MT_PLAY_MASK | MT_REC_MASK);

	
    if (dev->ioRead8(CCS_I2C_STATUS, card->iobase) & 0x80)
    {
        int version, size;
        unsigned long subvendor = 0;
        
        for (i = 0; i < 4; i++)
        {
            ReadI2C(dev, card, i);
        }
        
        subvendor = ReadI2C(dev, card, 0) |
                    (ReadI2C(dev, card, 1) << 8) |
                    (ReadI2C(dev, card, 2) << 16) |
                    (ReadI2C(dev, card, 3) << 24);
        
        switch (subvendor)
        {
            case MAUDIO_2496_ID: card->SubType = MAUDIO_2496;
								 card->Specific.NumChannels = 2;
								 card->Specific.HasSPDIF = true;
                                 IOLog("M-Audio Audiophile 2496 detected!\n");
                                 break;
			
			case MAUDIO_1010_ID: card->SubType = MAUDIO_1010;
				card->Specific.NumChannels = 8;
				card->Specific.HasSPDIF = true;
				IOLog("M-Audio Audiophile 1010 detected!\n");
				break;
                                 
            case MAUDIO_1010LT_ID: card->SubType = MAUDIO_1010LT;
								   card->Specific.NumChannels = 8;
								   card->Specific.HasSPDIF = true;
                                   IOLog("M-Audio Audiophile 1010LT detected!\n");
                                   break;

            case MAUDIO_DELTA44_ID: card->SubType = MAUDIO_DELTA44;
			                        card->Specific.NumChannels = 4;
								    card->Specific.HasSPDIF = false;
                                    IOLog("M-Audio Delta 44 detected!\n");
                                    break;
                                                             
            case MAUDIO_DELTA66_ID: card->SubType = MAUDIO_DELTA66;
								    card->Specific.NumChannels = 4;
								    card->Specific.HasSPDIF = true;
                                   IOLog("M-Audio Delta 66 detected!\n");
                                   break;
			
			case PHASE_88:
			case EWS88MT:
			case EWS88MT_2: 
			case TS88PCI:   card->SubType = PHASE88;
							card->Specific.NumChannels = 8;
							card->Specific.HasSPDIF = true;
							IOLog("Terratec Phase88/EWS88MT/TS88PCI detected!\n");
							break; 
            
            case EWX2496_ID:
            {
                card->SubType = EWX2496;
                card->Specific.NumChannels = 2;
                card->Specific.HasSPDIF = true;
                IOLog("Terratec EWX2496 detected!\n");
                break;
            }
                
            case MAUDIO_DELTA_410_ID:
            {
                card->SubType = DELTA_410;
                card->Specific.NumChannels = 8; // well, 2 input channels and 8 outputs
                card->Specific.HasSPDIF = true;
                IOLog("M-Audio Delta 410 detected!\n");
                break;                
            }

            default: 
            {
                card->SubType = PHASE88;
                card->Specific.NumChannels = 8;
                card->Specific.HasSPDIF = true;
                IOLog("Generic Envy24 detected!\n");
                IOLog("This specific Envy24 card with subvendor id %lx is not really supported! Only digital out might work.\n", subvendor);
			    break;
            }
        }
        
		card->Specific.BufferSize = NUM_SAMPLE_FRAMES * 10 * (BIT_DEPTH / 8);
        card->Specific.BufferSizeRec = NUM_SAMPLE_FRAMES * 12 * (BIT_DEPTH / 8);
		
        size = ReadI2C(dev, card, 4);
        version = ReadI2C(dev, card, 5);
        
        //IOLog("EEPROM size = %d, version = %d\n", size, version);
        size -= 6; // including bytes 0 - 5
        
        for (i = 0; i < size; i++)
        {
            eeprom[i] = ReadI2C(dev, card, i + 6);
        }
        
		dev->configWrite8(0x60, eeprom[0]); // Codecs
        dev->configWrite8(0x61, eeprom[1]); // AC-link
        dev->configWrite8(0x62, eeprom[2]); // I2S
        dev->configWrite8(0x63, eeprom[3]); // S/PDIF
        
        //IOLog("read config = %x\n", dev->configRead8(0x63));
        //IOLog("eeprom %x %x %x %x\n", eeprom[0], eeprom[1], eeprom[2], eeprom[3]);
        
		WriteCCI(card, CCI_GPIO_MASK, eeprom[4]); // GPIO MASK
        WriteCCI(card, CCI_GPIO_DATA, eeprom[5]); // GPIO STATE
        WriteCCI(card, CCI_GPIO_DIR, eeprom[6]);  // GPIO DIR
    }
    
    if (card->SubType == PHASE88)
    {
        if (Phase88_Init(card) != 0)
        {
            IOLog("Error initing Phase88!\n");
        }
        else
        {
            IOLog("Phase88 Initialized!\n");
        }
        
        card->ParmList = NULL;
    }
    else if (card->SubType == MAUDIO_2496)
    {
        card->akm_type = AKM4528;
        
        card->codec[0].caddr = 2;
        card->codec[0].cif = 0;
        card->codec[0].datamask = ICE1712_DELTA_AP_DOUT;
        card->codec[0].clockmask = ICE1712_DELTA_AP_CCLK;
        
        card->codec[0].csmask = ICE1712_DELTA_AP_CS_CODEC;
        card->codec[0].csaddr = ICE1712_DELTA_AP_CS_CODEC;
        card->codec[0].csnone = 0;
        
        card->codec[0].addflags = ICE1712_DELTA_AP_CS_DIGITAL;
		card->codec[0].totalmask = 0;
        card->codec[0].type = AKM4528;
		card->codec[0].newflag = 1;
        
        Init_akm4xxx(card, &card->codec[0]);
        
        IOLog("Envy24: Going to init CS8427 (3)!\n");
		init_cs8427_spdif(card);
		setup_consumer_mode(card);
		
#if 0
		unsigned char buf[24];
		snd_cs8427_reg_write(card, CS8427_REG_CLOCKSOURCE, 0x00);
        ap_cs8427_sendbytes(card, initvals1, 1);
		
		IOLog("Sent initvals1\n");
		memset(buf, 0, 7);
		buf[0] = 9;	/* register */
		ap_cs8427_sendbytes(card, buf, 7);
		ap_cs8427_sendbytes(card, initvals2, 4);
		IOLog("Sent initvals2\n");
		
		
		snd_cs8427_reset(card);
		IOLog("Reset cs8427\n");
#endif
		
        CreateParmsFor2496(card);
    }
    
    else if (card->SubType == MAUDIO_1010LT)
    {
        int chip;
        card->akm_type = AKM4524;

        for (chip = 0; chip < 4; chip++)
        {
            card->codec[chip].caddr = 2;
            card->codec[chip].cif = 0;
            card->codec[chip].datamask = ICE1712_DELTA_1010LT_DOUT;
            card->codec[chip].clockmask = ICE1712_DELTA_1010LT_CCLK;
            
            card->codec[chip].csmask = ICE1712_DELTA_1010LT_CS;

            if (chip == 0)
                card->codec[chip].csaddr = ICE1712_DELTA_1010LT_CS_CHIP_A;
            if (chip == 1)
                card->codec[chip].csaddr = ICE1712_DELTA_1010LT_CS_CHIP_B;
            if (chip == 2)
                card->codec[chip].csaddr = ICE1712_DELTA_1010LT_CS_CHIP_C;
            if (chip == 3)
                card->codec[chip].csaddr = ICE1712_DELTA_1010LT_CS_CHIP_D;

            card->codec[chip].csnone = ICE1712_DELTA_1010LT_CS_NONE;
            card->codec[chip].addflags = 0;
            card->codec[chip].totalmask = 0;
		    card->codec[chip].newflag = 1;
            
            card->codec[chip].type = AKM4524;
			Init_akm4xxx(card, &card->codec[chip]);
        }
        CreateParmsForDelta1010LT(card);
    }
    else if (card->SubType == MAUDIO_DELTA44 || card->SubType == MAUDIO_DELTA66)
    {
        card->akm_type = AKM4524;

        card->codec[0].caddr = 2;
        card->codec[0].cif = 0;
        card->codec[0].datamask = 0x10;
        card->codec[0].clockmask = 0x20;

        card->codec[0].csmask = 0x40; // 1st codec
        card->codec[0].csaddr = 0x40;
        card->codec[0].csnone = 0;
        card->codec[0].addflags = 0;
        card->codec[0].totalmask = 0;

        card->codec[0].type = AKM4524;
		card->codec[0].newflag = 1; 

        Init_akm4xxx(card, &card->codec[0]);
		
		card->codec[1].caddr = 2;
        card->codec[1].cif = 0;
        card->codec[1].datamask = 0x10;
        card->codec[1].clockmask = 0x20;

        card->codec[1].csmask = 0x80; // 2nd codec
        card->codec[1].csaddr = 0x80;
        card->codec[1].csnone = 0;
        card->codec[1].addflags = 0;
        card->codec[1].totalmask = 0;

        card->codec[1].type = AKM4524;
		card->codec[1].newflag = 1; 

        Init_akm4xxx(card, &card->codec[1]);
        CreateParmsForDelta4466(card);
        
        if (card->SubType == MAUDIO_DELTA66)
        {
            InitDigitalOut(card);
        }
    }
    else if (card->SubType == EWX2496)
    {
        card->akm_type = AKM4524;
        
        card->codec[0].caddr = 2;
        card->codec[0].cif = 1;
        card->codec[0].datamask = 0x10;
        card->codec[0].clockmask = 0x20;
        
        card->codec[0].csmask = 0x1;
        card->codec[0].csaddr = 0x1;
        card->codec[0].csnone = 0;
        card->codec[0].addflags = 8;
        card->codec[0].totalmask = 0;
        
        card->codec[0].type = AKM4524;
		card->codec[0].newflag = 1; 
        
        Init_akm4xxx(card, &card->codec[0]);
        
        CreateParmsForEWX2496(card);
    }
    else if (card->SubType == DELTA_410)
    {
        card->akm_type = AKM4529;
        
        card->codec[0].caddr = 0;
        card->codec[0].cif = 0;
        card->codec[0].datamask = 0x8;
        card->codec[0].clockmask = 0x2;
        
        card->codec[0].csmask = 0x20;
        card->codec[0].csaddr = 0x20;
        card->codec[0].csnone = 0;
        card->codec[0].addflags = 0x10;
        card->codec[0].totalmask = 0;
        
        card->codec[0].type = AKM4529;
		card->codec[0].newflag = 1; 
        
        Init_akm4xxx(card, &card->codec[0]);
        CreateParmsForDelta410(card);
    }
    
    // turn on monitoring:
    // MT30-31 select digital mixer is input instead of DMA channel 10-11
    dev->ioWrite16(0x30, 0x0101, card->mtbase);
    
    // loopback psdin[0] to psdout[0]
    dev->ioWrite32(0x34, 0x00000010, card->mtbase);
	
	IOLog("Envy24: Turned on monitoring\n");

    return 0;

}


void card_cleanup(struct CardData *card)
{
}


void InitDigitalOut(struct CardData *card)
{
    unsigned char bits = 0x01 /* consumer */ | 0x10 /* no emphasis */ | 0x20 /* PCM */;
    spdif_write(card, bits);
}


void spdif_write(struct CardData *card, unsigned char bits)
{
    int i;
    unsigned char tmp;
    
    tmp = ReadCCI(card, CCI_GPIO_DATA);
    for (i = 7; i >= 0; i--)
    {
        tmp &= ~(0x04 | 0x08); // stat clock | data
        if (bits & (1 << i))
        {
            tmp |= 0x08;
        }
        WriteCCI(card, CCI_GPIO_DATA, tmp);
        MicroDelay(100);
        tmp |= 0x04;
        WriteCCI(card, CCI_GPIO_DATA, tmp);
        MicroDelay(100);
    }
    
    tmp &= ~0x04;
    WriteCCI(card, CCI_GPIO_DATA, tmp);
}

/******************************************************************************
** Misc. **********************************************************************
******************************************************************************/

void
SaveMixerState( struct CardData* card )
{
}


void
RestoreMixerState( struct CardData* card )
{
}

void
UpdateMonitorMixer( struct CardData* card )
{
}


ULONG
SamplerateToLinearPitch( ULONG samplingrate )
{
  samplingrate = (samplingrate << 8) / 375;
  return (samplingrate >> 1) + (samplingrate & 1);
}


void MicroDelay(unsigned int micros)
{
  IODelay(micros);
}

// ----- parms ----
static void CreateParmsFor2496(struct CardData *card)
{
    Parm* p = new Parm;
    Parm *prev = NULL;
    card->ParmList = p;
    
    
    // left output
    p->InitialValue = 0x7F;
    p->MinValue = 14;
    p->MaxValue = 0x7F;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = kIOAudioControlChannelIDDefaultLeft; 
    p->Name = kIOAudioControlChannelNameLeft;
    p->ControlID = 0;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x4;
    p->reverse = false;  
    p->codec = 0;
    
    
    // right output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7F;
    p->MinValue = 14;
    p->MaxValue = 0x7F;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = kIOAudioControlChannelIDDefaultRight; 
    p->Name = kIOAudioControlChannelNameRight;
    p->ControlID = 1;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x5;
    p->reverse = false;  
    p->codec = 0;
    p->Next = NULL;
}


static void CreateParmsForDelta4466(struct CardData *card)
{
    Parm* p = new Parm;
    Parm *prev = NULL;
    card->ParmList = p;
    
    
    // left output
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = kIOAudioControlChannelIDDefaultLeft; 
    p->Name = kIOAudioControlChannelNameLeft;
    p->ControlID = 0;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x6;
    p->reverse = false;  
    p->codec = 0;
    p->Next = prev;
    
    
    // right output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = kIOAudioControlChannelIDDefaultRight; 
    p->Name = kIOAudioControlChannelNameRight;
    p->ControlID = 1;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x7;
    p->reverse = false;  
    p->codec = 0;
    p->Next = NULL;

    // third output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 3;//kIOAudioControlChannelIDDefaultLeftRear; 
    p->Name = "Output 3";
    p->ControlID = 2;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x6;
    p->reverse = false;  
    p->codec = 1;
    p->Next = prev;
    
    
    // fourth output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 4; //kIOAudioControlChannelIDDefaultRightRear; 
    p->Name = "Output 4";
    p->ControlID = 3;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x7;
    p->reverse = false;  
    p->codec = 1;
    
    // gains
    // input 1
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 1; 
    p->Name = "Input 1";
    p->ControlID = 4;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x4;
    p->reverse = false;  
    p->codec = 0;
    
    
    // input 2
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 2; 
    p->Name = "Input 2";
    p->ControlID = 5;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x5;
    p->reverse = false;  
    p->codec = 0;
    
    // input 3
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 3; 
    p->Name = "Input 3";
    p->ControlID = 6;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x4;
    p->reverse = false;  
    p->codec = 1;
    
    
    // input 4
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 4; 
    p->Name = "Input 4";
    p->ControlID = 7;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x5;
    p->reverse = false;  
    p->codec = 1;
    p->Next = NULL;
}


static void CreateParmsForDelta1010LT(struct CardData *card)
{
    Parm* p = new Parm;
    Parm *prev = NULL;
    card->ParmList = p;
    
    
    // left output
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = kIOAudioControlChannelIDDefaultLeft; 
    p->Name = kIOAudioControlChannelNameLeft;
    p->ControlID = 0;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x6;
    p->reverse = false;  
    p->codec = 0;
    
    // right output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = kIOAudioControlChannelIDDefaultRight; 
    p->Name = kIOAudioControlChannelNameRight;
    p->ControlID = 1;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x7;
    p->reverse = false;  
    p->codec = 0;
    
    // third output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 3;//kIOAudioControlChannelIDDefaultLeftRear; 
    p->Name = "Output 3";
    p->ControlID = 2;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x6;
    p->reverse = false;  
    p->codec = 1;
    
    
    // fourth output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 4; //kIOAudioControlChannelIDDefaultRightRear; 
    p->Name = "Output 4";
    p->ControlID = 3;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x7;
    p->reverse = false;  
    p->codec = 1;
    
    // fifth output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 5;//kIOAudioControlChannelIDDefaultLeftRear; 
    p->Name = "Output 5";
    p->ControlID = 4;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x6;
    p->reverse = false;  
    p->codec = 2;
    
    
    // sixth output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 6; //kIOAudioControlChannelIDDefaultRightRear; 
    p->Name = "Output 6";
    p->ControlID = 5;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x7;
    p->reverse = false;  
    p->codec = 2;
    
    
    // seventh output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 7;//kIOAudioControlChannelIDDefaultLeftRear; 
    p->Name = "Output 7";
    p->ControlID = 6;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x6;
    p->reverse = false;  
    p->codec = 3;
    
    
    // eigth output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 8; //kIOAudioControlChannelIDDefaultRightRear; 
    p->Name = "Output 8";
    p->ControlID = 7;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x7;
    p->reverse = false;  
    p->codec = 3;
    
    
    // gains
    // input 1
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 1; 
    p->Name = "Input 1";
    p->ControlID = 8;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x4;
    p->reverse = false;  
    p->codec = 0;
    
    
    // input 2
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = kIOAudioControlChannelIDDefaultRight; 
    p->Name = "Input 2";
    p->ControlID = 9;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x5;
    p->reverse = false;  
    p->codec = 0;
    
    // input 3
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 3; 
    p->Name = "Input 3";
    p->ControlID = 10;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x4;
    p->reverse = false;  
    p->codec = 1;
    
    
    // input 4
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 4; 
    p->Name = "Input 4";
    p->ControlID = 11;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x5;
    p->reverse = false;  
    p->codec = 1;
    
    // input 5
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 5; 
    p->Name = "Input 5";
    p->ControlID = 12;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x4;
    p->reverse = false;  
    p->codec = 2;
    
    
    // input 6
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 6; 
    p->Name = "Input 6";
    p->ControlID = 13;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x5;
    p->reverse = false;  
    p->codec = 2;
    
    // input 7
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 7; 
    p->Name = "Input 7";
    p->ControlID = 14;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x4;
    p->reverse = false;  
    p->codec = 3;
    
    
    // input 8
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = 8; 
    p->Name = "Input 8";
    p->ControlID = 15;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x5;
    p->reverse = false;  
    p->codec = 3;
    p->Next = NULL;
}


static void CreateParmsForEWX2496(struct CardData *card)
{
    Parm* p = new Parm;
    Parm *prev = NULL;
    card->ParmList = p;
    
    
    // left output
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = kIOAudioControlChannelIDDefaultLeft; 
    p->Name = kIOAudioControlChannelNameLeft;
    p->ControlID = 0;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x6;
    p->reverse = false;  
    p->codec = 0;
    
    
    // right output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 0x7E;
    p->MinValue = 14;
    p->MaxValue = 0x7E;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = kIOAudioControlChannelIDDefaultRight; 
    p->Name = kIOAudioControlChannelNameRight;
    p->ControlID = 1;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x7;
    p->reverse = false;  
    p->codec = 0;
    
    // gains
    // left input
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = kIOAudioControlChannelIDDefaultLeft; 
    p->Name = kIOAudioControlChannelNameLeft;
    p->ControlID = 2;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x4;
    p->reverse = false;  
    p->codec = 0;
    
    
    // right input
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 128;
    p->MinValue = 128;
    p->MaxValue = 164;
    p->MindB = (0 << 16) + 32768;
    p->MaxdB = (18 << 16) + 32768;
    p->ChannelID = kIOAudioControlChannelIDDefaultRight; 
    p->Name = kIOAudioControlChannelNameRight;
    p->ControlID = 3;
    p->Usage = kIOAudioControlUsageInput;
    p->reg = 0x5;
    p->reverse = false;  
    p->codec = 0;
    p->Next = NULL;
}


void CreateParmsForDelta410(struct CardData *card)
{
    Parm* p = new Parm;
    Parm *prev = NULL;
    card->ParmList = p;
    
    
    // left output
    p->InitialValue = 98;
    p->MinValue = 0;
    p->MaxValue = 98;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = kIOAudioControlChannelIDDefaultLeft; 
    p->Name = kIOAudioControlChannelNameLeft;
    p->ControlID = 0;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x2;
    p->reverse = true;  
    p->codec = 0;
    
    // right output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 98;
    p->MinValue = 0;
    p->MaxValue = 98;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = kIOAudioControlChannelIDDefaultRight; 
    p->Name = kIOAudioControlChannelNameRight;
    p->ControlID = 1;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x3;
    p->reverse = true;  
    p->codec = 0;
    
    // third output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 98;
    p->MinValue = 0;
    p->MaxValue = 98;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 3;//kIOAudioControlChannelIDDefaultLeftRear; 
    p->Name = "Output 3";
    p->ControlID = 2;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x4;
    p->reverse = true;  
    p->codec = 0;
    
    
    // fourth output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 98;
    p->MinValue = 0;
    p->MaxValue = 98;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 4; //kIOAudioControlChannelIDDefaultRightRear; 
    p->Name = "Output 4";
    p->ControlID = 3;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x5;
    p->reverse = true;  
    p->codec = 0;
    
    // fifth output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 98;
    p->MinValue = 0;
    p->MaxValue = 98;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 5;//kIOAudioControlChannelIDDefaultLeftRear; 
    p->Name = "Output 5";
    p->ControlID = 4;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x6;
    p->reverse = true;  
    p->codec = 0;
    
    
    // sixth output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 98;
    p->MinValue = 0;
    p->MaxValue = 98;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 6; //kIOAudioControlChannelIDDefaultRightRear; 
    p->Name = "Output 6";
    p->ControlID = 5;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0x7;
    p->reverse = true;  
    p->codec = 0;
    
    
    // seventh output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 98;
    p->MinValue = 0;
    p->MaxValue = 98;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 7;//kIOAudioControlChannelIDDefaultLeftRear; 
    p->Name = "Output 7";
    p->ControlID = 6;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0xB;
    p->reverse = true;  
    p->codec = 0;
    
    
    // eigth output
    prev = p;
    p = new Parm;
    prev->Next = p;
    p->InitialValue = 98;
    p->MinValue = 0;
    p->MaxValue = 98;
    p->MindB = (-49 << 16) + 32768;
    p->MaxdB = 0;
    p->ChannelID = 8; //kIOAudioControlChannelIDDefaultRightRear; 
    p->Name = "Output 8";
    p->ControlID = 7;
    p->Usage = kIOAudioControlUsageOutput;
    p->reg = 0xC;
    p->reverse = true;  
    p->codec = 0;
    p->Next = NULL;
}


/*
 p->InitialValue = 
 p->MinValue = 
 p->MaxValue = 
 p->MindB = 
 p->MaxdB = 
 p->ChannelID = 
 p->Name = 
 p->ControlID = 
 p->usage = 
 p->reg = 
 p->reverse = 
 p->codec = 
 p->Next = 
*/