#define pr_fmt(fmt) "MFRC522: " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/delay.h>

#define RST 10
/*Debug*/
#define DEBUG 1 

/*command overview : pg 70 :PDC(Proximity Coupling Device)*/
enum PDC_CMD {
	PCD_IDLE       = 0x00,       //	No action, cancels current command execution
  	PCD_AUTHENT    = 0x0E,       //	Performs the MIFARE standard authentication as a reader
  	PCD_RECEIVE    = 0x08,       //	Activates the receiver circuits
  	PCD_TRANSMIT   = 0x04,       //	Transmits data from FIFO buffer 
  	PCD_TRANSCEIVE = 0x0C,       //	Transmits data from FIFO buffer to antenna 
  	PCD_RESETPHASE = 0x0F,       //	Rsets the MFRC522
	PCD_CALCCRC    = 0x03,	    //	activates the CRC coprocessor or performs a self test
};

/* Commands sent to the PICC (Proximity Integrated Circuit Card).*/
/*The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4 pg 12)*/
enum PICC_CMD {
	PICC_REQIDL    = 0x26,
  	PICC_REQALL    = 0x52,
  	PICC_ANTICOLL  = 0x93,
  	PICC_SElECTTAG = 0x93,
  	PICC_AUTHENT1A = 0x60,
  	PICC_AUTHENT1B = 0x61,
 	PICC_READ      = 0x30,
 	PICC_WRITE     = 0xA0,
  	PICC_DECREMENT = 0xC0,
  	PICC_INCREMENT = 0xC1,
  	PICC_RESTORE   = 0xC2,
  	PICC_TRANSFER  = 0xB0,
	PICC_HALT      = 0x50,	
};
/*MFRC522 registers : pg 35*/
enum PCD_Reg {
	  /*Page 0:Command and status */
	  Reserved00         = 0x00,
	  CommandReg         = 0x01,
	  CommIEnReg         = 0x02,
	  DivlEnReg          = 0x03,
	  CommIrqReg         = 0x04,
	  DivIrqReg          = 0x05,
	  ErrorReg           = 0x06,
	  Status1Reg         = 0x07,
	  Status2Reg         = 0x08,
	  FIFODataReg        = 0x09,
	  FIFOLevelReg       = 0x0A,
	  WaterLevelReg      = 0x0B,
	  ControlReg         = 0x0C,
	  BitFramingReg      = 0x0D,
	  CollReg            = 0x0E,
	  Reserved01         = 0x0F,
	 
	  /*Page 1: Command*/ 
	  Reserved10         = 0x10,
	  ModeReg            = 0x11,
	  TxModeReg          = 0x12,
	  RxModeReg          = 0x13,
	  TxControlReg       = 0x14,
	  TxAutoReg          = 0x15,
	  TxSelReg           = 0x16,
	  RxSelReg           = 0x17,
	  RxThresholdReg     = 0x18,
	  DemodReg           = 0x19,
	  Reserved11         = 0x1A,
	  Reserved12         = 0x1B,
	  MifareReg          = 0x1C,
	  Reserved13         = 0x1D,
	  Reserved14         = 0x1E,
	  SerialSpeedReg     = 0x1F,
	  
	  /*Page 2: Configuration*/
	  Reserved20         = 0x20,  
	  CRCResultRegM      = 0x21,
	  CRCResultRegL      = 0x22,
	  Reserved21         = 0x23,
	  ModWidthReg        = 0x24,
	  Reserved22         = 0x25,
	  RFCfgReg           = 0x26,
	  GsNReg             = 0x27,
	  CWGsPReg           = 0x28,
	  ModGsPReg          = 0x29,
	  TModeReg           = 0x2A,
	  TPrescalerReg      = 0x2B,
	  TReloadRegH        = 0x2C,
	  TReloadRegL        = 0x2D,
	  TCounterValueRegH  = 0x2E,
	  TCounterValueRegL  = 0x2F,
	 
	  /*Page 3: Test register*/ 
	  Reserved30         = 0x30,
	  TestSel1Reg        = 0x31,
	  TestSel2Reg        = 0x32,
	  TestPinEnReg       = 0x33,
	  TestPinValueReg    = 0x34,
	  TestBusReg         = 0x35,
	  AutoTestReg        = 0x36,
	  VersionReg         = 0x37,
	  AnalogTestReg      = 0x38,
	  TestDAC1Reg        = 0x39,
	  TestDAC2Reg        = 0x3A,
	  TestADCReg         = 0x3B,
	  Reserved31         = 0x3C,
	  Reserved32         = 0x3D,
	  Reserved33         = 0x3E,
	  Reserved34 	     = 0x3F,	
};

/*GPIO Registers*/
enum GPIO_OMAP_Reg { 
	OMAP_GPIO_OE 	       = 0x0134,
	OMAP_GPIO_SETDATAOUT   = 0x0194,
	OMAP_GPIO_CLEARDATAOUT = 0x0190,
	OMAP_GPIO_DATAIN       = 0x0138,
};

/*GPIO Base Addresses */
enum GPIO_BASE_Addr {
	GPIO0		       = 0x44E06000,
        GPIO1		       = 0x4804C000,
	GPIO2		       = 0x481AC000,
	GPIO3		       = 0x481AE000,
};

/*MIFARE Status */
enum Status {
	MI_OK       = 0,
	MI_NOTAGERR = 1,
	MI_ERR      = 2,
};

#define MAX_LEN 16

struct spi_dev {

	struct spi_device       *spi;
	
	/* for Device status*/

	int     status;
	int     backLen;
	int 	backDataLen; 
	int 	keyLen;
	int 	uidLen;
};

static void gpio_init_and_set(void)
{
	unsigned int data = 0;

	data = readl_relaxed(GPIO0 + OMAP_GPIO_OE);
	data = data & 0xFFFFFDFF;
	pr_debug("GPIO Init: Direction of pin is set: %x\n",data);
	writel_relaxed(data, GPIO0 + OMAP_GPIO_OE);

	data = data | (1U << RST);

	pr_debug("GPIO Set: Direction of pin is set: %x\n",data);

	writel_relaxed(data, GPIO0 + OMAP_GPIO_SETDATAOUT); //High
}
/* -----------------------------------------------------------------
 * Read from a register
 * -----------------------------------------------------------------*/
static int mfrc522_read_value(struct spi_device *spi, u8 reg)
{
	reg = (reg & 0x7E) | 0x80;
	pr_debug("%s: Value : %d\n",__func__,spi_w8r8(spi,reg));
	return spi_w8r8(spi,reg);
}

/* -----------------------------------------------------------------
 * Write to a register
 * -----------------------------------------------------------------*/
static int mfrc522_write_value(struct spi_device *spi, u8 reg,u8 value)
{
	unsigned char buf[2];
 
	buf[0] = reg & 0x7E;
	buf[1] = value;
	pr_debug("%s: Value : %d\n",__func__,spi_write_then_read(spi, buf, 2, NULL, 0));
	return spi_write_then_read(spi, buf, 2, NULL, 0);
}
/* -----------------------------------------------------------------
 * MFRC522 reset function
 * -----------------------------------------------------------------*/
static void MFRC522_Reset(struct spi_device *spi)
{
	struct spi_dev *dev = spi_get_drvdata(spi);
	mfrc522_write_value(dev->spi,CommandReg,PCD_RESETPHASE);
	dev_info(&spi->dev,"Device Reset successfull\n");
}

/* -----------------------------------------------------------------
 * Set and clear BIT Mask
 * -----------------------------------------------------------------*/
static void  SetBitMask(struct spi_device *spi,u8 reg,u8 mask)
{
	int temp = mfrc522_read_value(spi, reg);
	pr_debug("SeTBitMask: is %x\n",temp);
	mfrc522_write_value(spi, reg ,temp | mask);
}
static void  ClearBitMask(struct spi_device *spi,u8 reg,u8 mask)
{
	int temp = mfrc522_read_value(spi, reg);
	pr_debug("ClearBitMask: is %x\n",temp);
	mfrc522_write_value(spi, reg, temp & (~mask));
}


/* -----------------------------------------------------------------
 * Antenna ON/OFF Function
 * -----------------------------------------------------------------*/
static void AntennaOn(struct spi_device *spi)
{
	int temp = mfrc522_read_value(spi, TxControlReg);
	pr_debug("Antenna Reg Value %d\n",temp & 0x03);
	if (~(temp & 0x03))
		SetBitMask(spi, TxControlReg, 0x03);
}

static void Antennaoff(struct spi_device *spi)
{
	ClearBitMask(spi,TxControlReg,0x03);
}

static int *MFRC522_ToCard(struct spi_device *spi,u8 command,
					int *sendData,int dataLen)
{
	int *backData; 
	int lastBits;
	u8 irqEn = 0x00;
	u8  waitIRq = 0x00;
	int i = 0;
	int n = 0; 
		
	struct spi_dev *dev = spi_get_drvdata(spi);
	dev->status = MI_ERR;
	dev->backLen = 0;
	/*Page 38: ComIEnReg register*/
	if (command == PCD_AUTHENT) {
		irqEn = 0x12;
		waitIRq = 0x10;      //IdleIEn
	}
	if (command == PCD_TRANSCEIVE) {
		irqEn = 0x77;   
                waitIRq = 0x30; //RxIEn | IdleIEn
	}
	
	mfrc522_write_value(spi, CommIEnReg, irqEn|0x80);
	SetBitMask(spi, CommIEnReg, 0x80);
	ClearBitMask(spi, FIFOLevelReg, 0x80); //immediately clears the internal FIFO buffer’s

	/*No action, cancels current command Execution */
	mfrc522_write_value(spi, CommandReg, PCD_IDLE); 

	while (i < dataLen)
		mfrc522_write_value(spi, FIFODataReg, sendData[i++]);
		
	mfrc522_write_value(spi, CommandReg, command);

	if (command == PCD_TRANSCEIVE)
		SetBitMask(spi, BitFramingReg, 0x80); //page: 46
	
	/*waiting for valid cmd*/
	while(1) {
		n = mfrc522_read_value(spi, CommIrqReg);
		i--;
		if ((i != 0) && (~(n & 0x01)) && (~(n & waitIRq)))
			break;
	}
	ClearBitMask(spi, BitFramingReg, 0x80); //page: 46

	if (i != 0) {
		if (mfrc522_read_value(spi,ErrorReg) & 0x1B == 0x00) 
				dev->status = MI_OK;
		if (n & irqEn & 0x01)
				dev->status = MI_NOTAGERR;
		if (command == PCD_TRANSCEIVE) {
			n = mfrc522_read_value(spi , FIFOLevelReg);
			lastBits = mfrc522_read_value(spi, ControlReg) & 0x07; //pg 45
			if (lastBits != 0) 
				dev->backLen = (n-1) * 8 + lastBits;
			else 
				dev->backLen = n*8;
			if (n == 0)
				n = 1;
			if (n > MAX_LEN)
				n = MAX_LEN;
			i = 0;
			
			backData = (int *)kmalloc(n, GFP_KERNEL);
			while (i < n)
				backData[i++] = mfrc522_read_value(spi, FIFODataReg);
			dev->backDataLen = i;
		}
	}else{ 
			dev->status = MI_ERR;
	}

	return backData;
}
static int MFRC522_Request(struct spi_device *spi,int reqMode)
{
	int status;
	int backBits;
	int TagType[1];
	
	struct spi_dev *dev = spi_get_drvdata(spi);
	
	mfrc522_write_value(spi, BitFramingReg, 0x07);
	TagType[0] = reqMode;
	
	MFRC522_ToCard(spi, PCD_TRANSCEIVE, TagType,1);

	if ((dev->status != MI_OK) | (dev->backLen != 0x10))
		dev->status = MI_ERR;
	
	return dev->status;
}
/*Anticollision : */
static int *MFRC522_Anticoll(struct spi_device *spi)
{
	int *backData;
	int serNumCheck = 0;
	int serNum[2];
	
	struct spi_dev *dev = spi_get_drvdata(spi);

	mfrc522_write_value(spi , BitFramingReg, 0x00);

	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;  // page 14 

	backData = MFRC522_ToCard(spi, PCD_TRANSCEIVE, serNum,2);

	if (dev->status == MI_OK) {
		int i = 0;
		if (dev->backDataLen == 5)
			while (i < 4) {
				serNumCheck = serNumCheck ^ backData[i++];
			}
			if (serNumCheck != backData[i])
				dev->status = MI_ERR;
			else dev->status = MI_OK;
	}

	return backData;
}


/*Calculate CRC :*/

static int *CalulateCRC(struct spi_device *spi,
				int *pIndata,int inputdataLen)
{
	int n, i = 0;
	int pOutData[2];
	ClearBitMask(spi, DivIrqReg, 0x04); // page 40
	SetBitMask(spi, FIFOLevelReg, 0x80);

	while (i < inputdataLen) {
		mfrc522_write_value(spi, FIFODataReg, pIndata[i++]);
	}
		
	mfrc522_write_value(spi, CommandReg, PCD_CALCCRC);
	i = 0xFF;

	while (1) {
		n = mfrc522_read_value(spi, DivIrqReg);
		i--;
		if ((i == 0) && (n & 0x04)) //TODO
			break;
	}

	pOutData[0] = mfrc522_read_value(spi, CRCResultRegL);
	pOutData[1] = mfrc522_read_value(spi, CRCResultRegM);
	return pOutData;
}

static int MFRC522_SelectTag(struct spi_device *spi, int *serNum)
{
	int *backData;
	int buff[8];
	int *pOut;
	int j = 2, i = 0;

	struct spi_dev *dev = spi_get_drvdata(spi);

	buff[0] = PICC_SElECTTAG;
	buff[1] = 0x70;

	while (i < 5) {
		buff[j++] = serNum[i++];
	}
	pOut = CalulateCRC(spi, buff, 7);

	buff[j] = pOut[0];
	buff[++j] = pOut[1];
	backData = MFRC522_ToCard(spi, PCD_TRANSCEIVE, buff,8);

	if ((dev->status == MI_OK) && (dev->backLen == 0x18))
		return backData[0];
	else 
		return 0;

}

static int MFRC522_Auth(struct spi_device *spi, int authMode, 
			int BlockAddr, int *Sectorkey, int *serNum) 
{
	int buff[16];
	int j =2,i = 0;	
	
	struct spi_dev *dev = spi_get_drvdata(spi);
	/*First byte should be the authMode (A or B)*/
	buff[0] = authMode;
	
	/*Second byte is the trailerBlock (usually 7)*/
	buff[1] = BlockAddr;

	/*Now we need to append the authKey which usually is 6 bytes of 0xFF */

	while (i < dev->keyLen) {
		buff[j++] = Sectorkey[i++];
	}

	i = 0;

	/*Next we append the first 4 bytes of the UID*/
	while (i < 4)
		buff[j++] = serNum[i];

	/*Now we start the authentication itself*/

	MFRC522_ToCard(spi, PCD_AUTHENT,buff,4);

	/*Check if an error occurred */
	if (dev->status != MI_OK) {
		pr_err("%s: AUTHENTICATION ERROR !\n",__func__);
		pr_debug("%s: AUTHENTICATION ERROR ! And Error type %d\n",__func__,dev->status);
	}
	
	if ((mfrc522_read_value(spi, Status2Reg) &  0x08) == 0) { //TODO
		pr_err("%s: AUTHENTICATION ERROR ! (status2reg & 0x08) == 0\n",__func__);
	}
	
	/*Return the status*/
	return dev->status;
}
/* Indicates that the MIFARE Crypto1 unit is switched on and
 * therefore all data communication with the card is encrypted
 * can only be set to logic 1 by a successful execution of the
 * MFAuthent command 
 */
static void MFRC522_StopCrypto1(struct spi_device *spi)
{
#if DEBUG 
	pr_debug("%s: StopCryptol\n",__func__);
#endif 
	ClearBitMask(spi, Status2Reg, 0x08); //Page 43
}

static void MFRC522_Read(struct spi_device *spi, int blockAddr)
{
	int  recvData[4]; 
	int *pOut;
	int *backData;
	
	struct spi_dev *dev = spi_get_drvdata(spi);
#if DEBUG
	pr_debug("%s: Block Address %2X\n",__func__,blockAddr);
#endif
	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;

	pOut = CalulateCRC(spi,recvData,2);
	
	recvData[2] = pOut[0];
	recvData[3] = pOut[1];
	
	backData = MFRC522_ToCard(spi, PCD_TRANSCEIVE, recvData,4);
	
	if(dev->status != MI_OK )
		pr_err("%s: Error while reading!\n",__func__);

	if(dev->backDataLen == 16)
		pr_info("Sector %2X Data %s\n",blockAddr,backData);	
		
}

static void MFRC522_Write(struct spi_device *spi,
				int blockAddr, int *writeData)
{
	int buff[18];
	int *crc;
	int *backData;
	int i = 0; 
	struct spi_dev *dev = spi_get_drvdata(spi);
#if DEBUG
	pr_debug("%s: Block Address %2X and Data: %2X \n",__func__,blockAddr,writeData[0]);
#endif
	
	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	
	crc =  CalulateCRC(spi,buff,2);
	
	buff[2] = crc[0];
	buff[3] = crc[1];

	
	backData = MFRC522_ToCard(spi, PCD_TRANSCEIVE, buff,4); //Buffsize is 4
	
	if ((dev->status != MI_OK) || (dev->backLen != 4) || (backData[0] & 0x0F) != 0x0A)
			dev->status = MI_ERR;
	pr_info("BackLen %d",dev->backLen); //TODO

	if (dev->status == MI_OK) {
		
		while (i < 16){
			buff[i] = writeData[i];
			i++;
		}
		crc = CalulateCRC(spi,buff,16);
		buff[16] = crc[0];
		buff[17] = crc[1];
	
		backData = MFRC522_ToCard(spi, PCD_TRANSCEIVE, buff,18); //buff size is 18
		if ((dev->status != MI_OK) || (dev->backLen != 4) || (backData[0] & 0x0F) != 0x0A)
			pr_info("%s: Error while writing. !\n",__func__);
		if (dev->status == MI_OK)
			pr_info("%s: Data written successful\n",__func__);
	}
}
static void MFRC522_DumpClassic1K(struct spi_device *spi, int *key, int *uid)
{
	int i = 0; 
	int status;
#if DEBUG 
	pr_debug("%s: DumpClassic1K \n",__func__);
#endif 
	while (i < 64) {
		status =  MFRC522_Auth(spi, PICC_AUTHENT1A, i, key, uid);
		/*Check if authenticated*/
		if (status == MI_OK)
			MFRC522_Read(spi, i);
		else
			pr_err("%s: Authentication error\n",__func__);
		i++;
	}
}
static int mfrc522_probe(struct spi_device *spi)
{
	int status;
	struct spi_dev *dev; 
	/*setup SPI mode and clock rate*/            
        status = spi_setup(spi);
        if (status < 0) { 
                dev_dbg(&spi->dev, "needs SPI mode %02x, %d KHz; %d\n",
                                spi->mode, spi->max_speed_hz / 1000,
                                status);
                return status;
        }    
	
	dev = (struct spi_dev*)kmalloc(sizeof(struct spi_dev), GFP_KERNEL);	
	dev->spi = spi;
	
	/* device driver data */
	spi_set_drvdata(spi, dev);
	return 0;
}

static int mfrc522_remove(struct spi_device *spi)
{
	return 0;
} 
static const struct of_device_id mfrc522_of_match[]= {
	{.compatible = "RFID,MFRC522", },
	{ }
};

static struct spi_driver mfrc522_driver  = {
	.driver = {
		.name           = "mfrc522",
		.of_match_table = mfrc522_of_match,
	},
	.probe  = mfrc522_probe,
	.remove = mfrc522_remove,
};

/**
 * module_spi_driver() - Helper macro for registering a SPI driver
 * @__spi_driver: spi_driver struct
 *
 * Helper macro for SPI drivers which do not do anything special in module
 * init/exit. This eliminates a lot of boilerplate. Each module may only
 * use this macro once, and calling it replaces module_init() and module_exit()
 */

module_spi_driver(mfrc522_driver);



MODULE_DESCRIPTION("Driver for SPI based RFID : MFRC522");
MODULE_AUTHOR("Chandan jha <beingchandanjha@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(".1");
