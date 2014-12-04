#ifndef _REGS_H
#define _REGS_H

#define CCS_CTRL                0x0000
    #define CCS_RESET_ALL           0x80
    #define CCS_NATIVE_MODE         0x01

#define CCS_INTR_MASK           0x0001
    #define CCS_ENABLE_MIDI1        0x80
    #define CCS_ENABLE_TIMER        0x40
    #define CCS_ENABLE_MIDI2        0x20
    #define CCS_ENABLE_PRO_MACRO    0x10
    #define CCS_ENABLE_FM           0x08
    #define CCS_ENABLE_PLAY_DS      0x04
    #define CCS_ENABLE_CONS_REC     0x02
    #define CCS_ENABLE_CONS_PLAY    0x01

#define CCS_INTR_STATUS         0x0002
    #define CCS_INTR_MIDI1          0x80
    #define CCS_INTR_TIMER          0x40
    #define CCS_INTR_MIDI2          0x20
    #define CCS_INTR_PRO_MACRO      0x10
    #define CCS_INTR_FM             0x08
    #define CCS_INTR_PLAY_DS        0x04
    #define CCS_INTR_CONS_REC       0x02
    #define CCS_INTR_CONS_PLAY      0x01

#define CCS_ENVY_INDEX          0x0003 // CCI registers
#define CCS_ENVY_DATA           0x0004

#define CCS_MIDI1_DATA          0x000C
#define CCS_MIDI1_CMD_STATUS    0x000D

#define CCS_I2C_DEV_ADDRESS     0x0010   // check CCS13 before accessing
    #define CCS_ADDRESS_MASK        0xFE // I2C device address (bit 7:1)
    #define CCS_ADDRESS_SHIFT       1
    #define CCS_ADDRESS_WRITE       0x01 // 0: read, 1:write


#define CCS_I2C_ADDR            0x0011   // byte address to read/write
#define CCS_I2C_DATA            0x0012
#define CCS_I2C_STATUS          0X0013
    #define CCS_I2C_EPROM           0x80 // 1: E2PROM connected
    #define CCS_I2C_BUSY            0x01 // 0: idle, 1: busy


#define CCS_CONS_REC_ADDRESS    0x0014
#define CCS_CONS_REC_LENGTH     0x0018


#define CCS_MIDI2_DATA          0x001C
#define CCS_MIDI2_CMD_STATUS    0x001D

#define CCS_TIMER               0x001E // word
    #define CCS_TIMER_ENABLE        0x8000
    #define CCS_TIMER_MASK          0x7FFF // bits 0 - 14: write to set up the period for the internal 15 bits timer to generate an interrupt


#define CCI_GPIO_DATA           0x20
#define CCI_GPIO_MASK           0x21 // 0 in mask means CCI_GPIO_DATA register bit can be written
#define CCI_GPIO_DIR            0x22 // 1 = output

#define CCI_CONS_POWER_DOWN     0x30
#define CCI_PRO_POWER_DOWN      0x31




// ----------MT regs-------------------

#define MT_INTR_MASK_STATUS     0x0000
    #define MT_REC_MASK             0x80
    #define MT_PLAY_MASK            0x40
    #define MT_REC_STATUS           0x02
    #define MT_PLAY_STATUS          0x01

#define MT_SAMPLERATE           0x0001   // in slave mode (SPDIF is master): 256X master clock alone selects rate
    #define MT_SPDIF_MASTER         0x10 // 
    #define MT_RATE_MASK            0x0F // ignored if MT_SPDIF_MASTER = 

#define MT_I2S_FORMAT           0x0002
    #define MT_CLOCK_128x           0x08 // 0: 256x (default), 1: 128x
    #define MT_CLOCK_48bpf          0x04 // 0: 256x (default), 1: 128x

#define MT_AC97_REG             0x0004 // AC'97 register index
#define MT_AC97_CMD_STATUS      0x0005 // valid when CCS_ACLINK_TYPE == 0
    #define MT_AC97_RESET           0x80 // cold reset (alone will put it into master mode)
    #define MT_AC97_WARM_RESET      0X40 // when used together with MT_AC97_RESET, will set external VIA AC'97 to slave mode
    #define MT_AC97_WRITE           0X20 // write 1 for write mode, reading a 1 is WIP
    #define MT_AC97_READ            0x10 // write 1 for read mode, reading a 1 is RIP
    #define MT_AC97_READY           0x08 // codec ready status
    #define MT_AC97_ID_MASK         0x03 // bit 0:1 is ID mode when is split mode.
#define MT_AC97_DATA            0x0006
    
    
#define MT_DMA_PB_ADDRESS       0x0010 // long: start address of interleaved playback buffer (long boundary), in long units
#define MT_DMA_PB_LENGTH        0x0014 // word: DMA size - 1, read: counter
#define MT_DMA_PB_INTLEN        0x0016 

#define MT_DMA_CONTROL          0x0018 // start/stop (use read-modify-write)
    #define MT_REC_START             0x04
    #define MT_PAUSE                 0x02
    #define MT_PLAY_START            0x01


// record pair registers
#define MT_DMA_REC_ADDRESS      0x0020
#define MT_DMA_REC_LENGTH       0x0024
#define MT_DMA_REC_INTLEN       0x0026

#define MT_VOLUME_L 0x38
#define MT_VOLUME_R 0x39
#define MT_VOLUME_INDEX 0x3A


#define ICE1712_DELTA_DFS 0x01		/* fast/slow sample rate mode */
#define ICE1712_DELTA_AP_CCLK	0x02	/* SPI clock */
					/* (clocking on rising edge - 0->1) */
#define ICE1712_DELTA_AP_DIN	0x04	/* data input */
#define ICE1712_DELTA_AP_DOUT	0x08	/* data output */
#define ICE1712_DELTA_AP_CS_DIGITAL 0x10 /* CS8427 chip select */
					/* low signal = select */
#define ICE1712_DELTA_AP_CS_CODEC 0x20	/* AK4528 (audiophile), AK4529 (Delta410) chip select */
					/* low signal = select */


/* 0x01 = DFS */
#define ICE1712_DELTA_1010LT_CCLK	0x02	/* SPI clock (AK4524 + CS8427) */
#define ICE1712_DELTA_1010LT_DIN	0x04	/* data input (CS8427) */
#define ICE1712_DELTA_1010LT_DOUT	0x08	/* data output (AK4524 + CS8427) */
#define ICE1712_DELTA_1010LT_CS		0x70	/* mask for CS address */
#define ICE1712_DELTA_1010LT_CS_CHIP_A	0x00	/* AK4524 #0 */
#define ICE1712_DELTA_1010LT_CS_CHIP_B	0x10	/* AK4524 #1 */
#define ICE1712_DELTA_1010LT_CS_CHIP_C	0x20	/* AK4524 #2 */
#define ICE1712_DELTA_1010LT_CS_CHIP_D	0x30	/* AK4524 #3 */
#define ICE1712_DELTA_1010LT_CS_CS8427	0x40	/* CS8427 */
#define ICE1712_DELTA_1010LT_CS_NONE	0x50	/* nothing */
#define ICE1712_DELTA_1010LT_WORDCLOCK 0x80	/* sample clock source: 0 = Word Clock Input, 1 = S/PDIF Input ??? */


#define IEC958_AES0_PROFESSIONAL	(1<<0)	/* 0 = consumer, 1 = professional */
#define IEC958_AES0_NONAUDIO		(1<<1)	/* 0 = audio, 1 = non-audio */
#define IEC958_AES0_PRO_EMPHASIS	(7<<2)	/* mask - emphasis */
#define IEC958_AES0_PRO_EMPHASIS_NOTID	(0<<2)	/* emphasis not indicated */
#define IEC958_AES0_PRO_EMPHASIS_NONE	(1<<2)	/* none emphasis */
#define IEC958_AES0_PRO_EMPHASIS_5015	(3<<2)	/* 50/15us emphasis */
#define IEC958_AES0_PRO_EMPHASIS_CCITT	(7<<2)	/* CCITT J.17 emphasis */
#define IEC958_AES0_PRO_FREQ_UNLOCKED	(1<<5)	/* source sample frequency: 0 = locked, 1 = unlocked */
#define IEC958_AES0_PRO_FS		(3<<6)	/* mask - sample frequency */
#define IEC958_AES0_PRO_FS_NOTID	(0<<6)	/* fs not indicated */
#define IEC958_AES0_PRO_FS_44100	(1<<6)	/* 44.1kHz */
#define IEC958_AES0_PRO_FS_48000	(2<<6)	/* 48kHz */
#define IEC958_AES0_PRO_FS_32000	(3<<6)	/* 32kHz */
#define IEC958_AES0_CON_NOT_COPYRIGHT	(1<<2)	/* 0 = copyright, 1 = not copyright */
#define IEC958_AES0_CON_EMPHASIS	(7<<3)	/* mask - emphasis */
#define IEC958_AES0_CON_EMPHASIS_NONE	(0<<3)	/* none emphasis */
#define IEC958_AES0_CON_EMPHASIS_5015	(1<<3)	/* 50/15us emphasis */
#define IEC958_AES0_CON_MODE		(3<<6)	/* mask - mode */
#define IEC958_AES1_PRO_MODE		(15<<0)	/* mask - channel mode */
#define IEC958_AES1_PRO_MODE_NOTID	(0<<0)	/* not indicated */
#define IEC958_AES1_PRO_MODE_STEREOPHONIC (2<<0) /* stereophonic - ch A is left */
#define IEC958_AES1_PRO_MODE_SINGLE	(4<<0)	/* single channel */
#define IEC958_AES1_PRO_MODE_TWO	(8<<0)	/* two channels */
#define IEC958_AES1_PRO_MODE_PRIMARY	(12<<0)	/* primary/secondary */
#define IEC958_AES1_PRO_MODE_BYTE3	(15<<0)	/* vector to byte 3 */
#define IEC958_AES1_PRO_USERBITS	(15<<4)	/* mask - user bits */
#define IEC958_AES1_PRO_USERBITS_NOTID	(0<<4)	/* not indicated */
#define IEC958_AES1_PRO_USERBITS_192	(8<<4)	/* 192-bit structure */
#define IEC958_AES1_PRO_USERBITS_UDEF	(12<<4)	/* user defined application */
#define IEC958_AES1_CON_CATEGORY	0x7f
#define IEC958_AES1_CON_GENERAL		0x00
#define IEC958_AES1_CON_LASEROPT_MASK	0x07
#define IEC958_AES1_CON_LASEROPT_ID	0x01
#define IEC958_AES1_CON_IEC908_CD	(IEC958_AES1_CON_LASEROPT_ID|0x00)
#define IEC958_AES1_CON_NON_IEC908_CD	(IEC958_AES1_CON_LASEROPT_ID|0x08)
#define IEC958_AES1_CON_MINI_DISC	(IEC958_AES1_CON_LASEROPT_ID|0x48)
#define IEC958_AES1_CON_DVD		(IEC958_AES1_CON_LASEROPT_ID|0x18)
#define IEC958_AES1_CON_LASTEROPT_OTHER	(IEC958_AES1_CON_LASEROPT_ID|0x78)
#define IEC958_AES1_CON_DIGDIGCONV_MASK 0x07
#define IEC958_AES1_CON_DIGDIGCONV_ID	0x02
#define IEC958_AES1_CON_PCM_CODER	(IEC958_AES1_CON_DIGDIGCONV_ID|0x00)
#define IEC958_AES1_CON_MIXER		(IEC958_AES1_CON_DIGDIGCONV_ID|0x10)
#define IEC958_AES1_CON_RATE_CONVERTER	(IEC958_AES1_CON_DIGDIGCONV_ID|0x18)
#define IEC958_AES1_CON_SAMPLER		(IEC958_AES1_CON_DIGDIGCONV_ID|0x20)
#define IEC958_AES1_CON_DSP		(IEC958_AES1_CON_DIGDIGCONV_ID|0x28)
#define IEC958_AES1_CON_DIGDIGCONV_OTHER (IEC958_AES1_CON_DIGDIGCONV_ID|0x78)
#define IEC958_AES1_CON_MAGNETIC_MASK	0x07
#define IEC958_AES1_CON_MAGNETIC_ID	0x03
#define IEC958_AES1_CON_DAT		(IEC958_AES1_CON_MAGNETIC_ID|0x00)
#define IEC958_AES1_CON_VCR		(IEC958_AES1_CON_MAGNETIC_ID|0x08)
#define IEC958_AES1_CON_DCC		(IEC958_AES1_CON_MAGNETIC_ID|0x40)
#define IEC958_AES1_CON_MAGNETIC_DISC	(IEC958_AES1_CON_MAGNETIC_ID|0x18)
#define IEC958_AES1_CON_MAGNETIC_OTHER	(IEC958_AES1_CON_MAGNETIC_ID|0x78)
#define IEC958_AES1_CON_BROADCAST1_MASK 0x07
#define IEC958_AES1_CON_BROADCAST1_ID	0x04
#define IEC958_AES1_CON_DAB_JAPAN	(IEC958_AES1_CON_BROADCAST1_ID|0x00)
#define IEC958_AES1_CON_DAB_EUROPE	(IEC958_AES1_CON_BROADCAST1_ID|0x08)
#define IEC958_AES1_CON_DAB_USA		(IEC958_AES1_CON_BROADCAST1_ID|0x60)
#define IEC958_AES1_CON_SOFTWARE	(IEC958_AES1_CON_BROADCAST1_ID|0x40)
#define IEC958_AES1_CON_IEC62105	(IEC958_AES1_CON_BROADCAST1_ID|0x20)
#define IEC958_AES1_CON_BROADCAST1_OTHER (IEC958_AES1_CON_BROADCAST1_ID|0x78)
#define IEC958_AES1_CON_BROADCAST2_MASK 0x0f
#define IEC958_AES1_CON_BROADCAST2_ID	0x0e
#define IEC958_AES1_CON_MUSICAL_MASK	0x07
#define IEC958_AES1_CON_MUSICAL_ID	0x05
#define IEC958_AES1_CON_SYNTHESIZER	(IEC958_AES1_CON_MUSICAL_ID|0x00)
#define IEC958_AES1_CON_MICROPHONE	(IEC958_AES1_CON_MUSICAL_ID|0x08)
#define IEC958_AES1_CON_MUSICAL_OTHER	(IEC958_AES1_CON_MUSICAL_ID|0x78)
#define IEC958_AES1_CON_ADC_MASK	0x1f
#define IEC958_AES1_CON_ADC_ID		0x06
#define IEC958_AES1_CON_ADC		(IEC958_AES1_CON_ADC_ID|0x00)
#define IEC958_AES1_CON_ADC_OTHER	(IEC958_AES1_CON_ADC_ID|0x60)
#define IEC958_AES1_CON_ADC_COPYRIGHT_MASK 0x1f
#define IEC958_AES1_CON_ADC_COPYRIGHT_ID 0x16
#define IEC958_AES1_CON_ADC_COPYRIGHT	(IEC958_AES1_CON_ADC_COPYRIGHT_ID|0x00)
#define IEC958_AES1_CON_ADC_COPYRIGHT_OTHER (IEC958_AES1_CON_ADC_COPYRIGHT_ID|0x60)
#define IEC958_AES1_CON_SOLIDMEM_MASK	0x0f
#define IEC958_AES1_CON_SOLIDMEM_ID	0x08
#define IEC958_AES1_CON_SOLIDMEM_DIGITAL_RECORDER_PLAYER (IEC958_AES1_CON_SOLIDMEM_ID|0x00)
#define IEC958_AES1_CON_SOLIDMEM_OTHER	(IEC958_AES1_CON_SOLIDMEM_ID|0x70)
#define IEC958_AES1_CON_EXPERIMENTAL	0x40
#define IEC958_AES1_CON_ORIGINAL	(1<<7)	/* this bits depends on the category code */
#define IEC958_AES2_PRO_SBITS		(7<<0)	/* mask - sample bits */
#define IEC958_AES2_PRO_SBITS_20	(2<<0)	/* 20-bit - coordination */
#define IEC958_AES2_PRO_SBITS_24	(4<<0)	/* 24-bit - main audio */
#define IEC958_AES2_PRO_SBITS_UDEF	(6<<0)	/* user defined application */
#define IEC958_AES2_PRO_WORDLEN		(7<<3)	/* mask - source word length */
#define IEC958_AES2_PRO_WORDLEN_NOTID	(0<<3)	/* not indicated */
#define IEC958_AES2_PRO_WORDLEN_22_18	(2<<3)	/* 22-bit or 18-bit */
#define IEC958_AES2_PRO_WORDLEN_23_19	(4<<3)	/* 23-bit or 19-bit */
#define IEC958_AES2_PRO_WORDLEN_24_20	(5<<3)	/* 24-bit or 20-bit */
#define IEC958_AES2_PRO_WORDLEN_20_16	(6<<3)	/* 20-bit or 16-bit */
#define IEC958_AES2_CON_SOURCE		(15<<0)	/* mask - source number */
#define IEC958_AES2_CON_SOURCE_UNSPEC	(0<<0)	/* unspecified */
#define IEC958_AES2_CON_CHANNEL		(15<<4)	/* mask - channel number */
#define IEC958_AES2_CON_CHANNEL_UNSPEC	(0<<4)	/* unspecified */
#define IEC958_AES3_CON_FS		(15<<0)	/* mask - sample frequency */
#define IEC958_AES3_CON_FS_44100	(0<<0)	/* 44.1kHz */
#define IEC958_AES3_CON_FS_NOTID	(1<<0)	/* non indicated */
#define IEC958_AES3_CON_FS_48000	(2<<0)	/* 48kHz */
#define IEC958_AES3_CON_FS_32000	(3<<0)	/* 32kHz */
#define IEC958_AES3_CON_FS_22050	(4<<0)	/* 22.05kHz */
#define IEC958_AES3_CON_FS_24000	(6<<0)	/* 24kHz */
#define IEC958_AES3_CON_FS_88200	(8<<0)	/* 88.2kHz */
#define IEC958_AES3_CON_FS_768000	(9<<0)	/* 768kHz */
#define IEC958_AES3_CON_FS_96000	(10<<0)	/* 96kHz */
#define IEC958_AES3_CON_FS_176400	(12<<0)	/* 176.4kHz */
#define IEC958_AES3_CON_FS_192000	(14<<0)	/* 192kHz */
#define IEC958_AES3_CON_CLOCK		(3<<4)	/* mask - clock accuracy */
#define IEC958_AES3_CON_CLOCK_1000PPM	(0<<4)	/* 1000 ppm */
#define IEC958_AES3_CON_CLOCK_50PPM	(1<<4)	/* 50 ppm */
#define IEC958_AES3_CON_CLOCK_VARIABLE	(2<<4)	/* variable pitch */
#define IEC958_AES4_CON_MAX_WORDLEN_24	(1<<0)	/* 0 = 20-bit, 1 = 24-bit */
#define IEC958_AES4_CON_WORDLEN		(7<<1)	/* mask - sample word length */
#define IEC958_AES4_CON_WORDLEN_NOTID	(0<<1)	/* not indicated */
#define IEC958_AES4_CON_WORDLEN_20_16	(1<<1)	/* 20-bit or 16-bit */
#define IEC958_AES4_CON_WORDLEN_22_18	(2<<1)	/* 22-bit or 18-bit */
#define IEC958_AES4_CON_WORDLEN_23_19	(4<<1)	/* 23-bit or 19-bit */
#define IEC958_AES4_CON_WORDLEN_24_20	(5<<1)	/* 24-bit or 20-bit */
#define IEC958_AES4_CON_WORDLEN_21_17	(6<<1)	/* 21-bit or 17-bit */
#define IEC958_AES4_CON_ORIGFS		(15<<4)	/* mask - original sample frequency */
#define IEC958_AES4_CON_ORIGFS_NOTID	(0<<4)	/* not indicated */
#define IEC958_AES4_CON_ORIGFS_192000	(1<<4)	/* 192kHz */
#define IEC958_AES4_CON_ORIGFS_12000	(2<<4)	/* 12kHz */
#define IEC958_AES4_CON_ORIGFS_176400	(3<<4)	/* 176.4kHz */
#define IEC958_AES4_CON_ORIGFS_96000	(5<<4)	/* 96kHz */
#define IEC958_AES4_CON_ORIGFS_8000	(6<<4)	/* 8kHz */
#define IEC958_AES4_CON_ORIGFS_88200	(7<<4)	/* 88.2kHz */
#define IEC958_AES4_CON_ORIGFS_16000	(8<<4)	/* 16kHz */
#define IEC958_AES4_CON_ORIGFS_24000	(9<<4)	/* 24kHz */
#define IEC958_AES4_CON_ORIGFS_11025	(10<<4)	/* 11.025kHz */
#define IEC958_AES4_CON_ORIGFS_22050	(11<<4)	/* 22.05kHz */
#define IEC958_AES4_CON_ORIGFS_32000	(12<<4)	/* 32kHz */
#define IEC958_AES4_CON_ORIGFS_48000	(13<<4)	/* 48kHz */
#define IEC958_AES4_CON_ORIGFS_44100	(15<<4)	/* 44.1kHz */
#define IEC958_AES5_CON_CGMSA		(3<<0)	/* mask - CGMS-A */
#define IEC958_AES5_CON_CGMSA_COPYFREELY (0<<0)	/* copying is permitted without restriction */
#define IEC958_AES5_CON_CGMSA_COPYONCE	(1<<0)	/* one generation of copies may be made */
#define IEC958_AES5_CON_CGMSA_COPYNOMORE (2<<0)	/* condition not be used */
#define IEC958_AES5_CON_CGMSA_COPYNEVER	(3<<0)	/* no copying is permitted */




#endif /* _REGS_H */
