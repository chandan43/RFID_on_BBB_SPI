#define pr_fmt(fmt) "MFRC522: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/delay.h>

#define RST 10
/*command overview : pg 70 :PDC(Proximity Coupling Device)*/
enum PDC_CMD {
	PCD_IDLE       = 0x00       //	No action, cancels current command execution
  	PCD_AUTHENT    = 0x0E       //	Performs the MIFARE standard authentication as a reader
  	PCD_RECEIVE    = 0x08       //	Activates the receiver circuits
  	PCD_TRANSMIT   = 0x04       //	Transmits data from FIFO buffer 
  	PCD_TRANSCEIVE = 0x0C       //	Transmits data from FIFO buffer to antenna 
  	PCD_RESETPHASE = 0x0F       //	Rsets the MFRC522
	PCD_CALCCRC    = 0x03	    //	activates the CRC coprocessor or performs a self test
};

/* Commands sent to the PICC (Proximity Integrated Circuit Card).*/
/*The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)*/
enum PICC_CMD {
	PICC_REQIDL    = 0x26
  	PICC_REQALL    = 0x52
  	PICC_ANTICOLL  = 0x93
  	PICC_SElECTTAG = 0x93
  	PICC_AUTHENT1A = 0x60
  	PICC_AUTHENT1B = 0x61
 	PICC_READ      = 0x30
 	PICC_WRITE     = 0xA0
  	PICC_DECREMENT = 0xC0
  	PICC_INCREMENT = 0xC1
  	PICC_RESTORE   = 0xC2
  	PICC_TRANSFER  = 0xB0
	PICC_HALT      = 0x50	
};
/*MFRC522 registers : pg 35*/
enum PCD_Reg {
	  /*Page 0:Command and status */
	  Reserved00         = 0x00
	  CommandReg         = 0x01
	  CommIEnReg         = 0x02
	  DivlEnReg          = 0x03
	  CommIrqReg         = 0x04
	  DivIrqReg          = 0x05
	  ErrorReg           = 0x06
	  Status1Reg         = 0x07
	  Status2Reg         = 0x08
	  FIFODataReg        = 0x09
	  FIFOLevelReg       = 0x0A
	  WaterLevelReg      = 0x0B
	  ControlReg         = 0x0C
	  BitFramingReg      = 0x0D
	  CollReg            = 0x0E
	  Reserved01         = 0x0F
	 
	  /*Page 1: Command*/ 
	  Reserved10         = 0x10
	  ModeReg            = 0x11
	  TxModeReg          = 0x12
	  RxModeReg          = 0x13
	  TxControlReg       = 0x14
	  TxAutoReg          = 0x15
	  TxSelReg           = 0x16
	  RxSelReg           = 0x17
	  RxThresholdReg     = 0x18
	  DemodReg           = 0x19
	  Reserved11         = 0x1A
	  Reserved12         = 0x1B
	  MifareReg          = 0x1C
	  Reserved13         = 0x1D
	  Reserved14         = 0x1E
	  SerialSpeedReg     = 0x1F
	  
	  /*Page 2: Configuration*/
	  Reserved20         = 0x20  
	  CRCResultRegM      = 0x21
	  CRCResultRegL      = 0x22
	  Reserved21         = 0x23
	  ModWidthReg        = 0x24
	  Reserved22         = 0x25
	  RFCfgReg           = 0x26
	  GsNReg             = 0x27
	  CWGsPReg           = 0x28
	  ModGsPReg          = 0x29
	  TModeReg           = 0x2A
	  TPrescalerReg      = 0x2B
	  TReloadRegH        = 0x2C
	  TReloadRegL        = 0x2D
	  TCounterValueRegH  = 0x2E
	  TCounterValueRegL  = 0x2F
	 
	  /*Page 3: Test register*/ 
	  Reserved30         = 0x30
	  TestSel1Reg        = 0x31
	  TestSel2Reg        = 0x32
	  TestPinEnReg       = 0x33
	  TestPinValueReg    = 0x34
	  TestBusReg         = 0x35
	  AutoTestReg        = 0x36
	  VersionReg         = 0x37
	  AnalogTestReg      = 0x38
	  TestDAC1Reg        = 0x39
	  TestDAC2Reg        = 0x3A
	  TestADCReg         = 0x3B
	  Reserved31         = 0x3C
	  Reserved32         = 0x3D
	  Reserved33         = 0x3E
	  Reserved34 	     = 0x3F	
};

/*GPIO Registers*/
enum GPIO_OMAP_Reg { 
	OMAP_GPIO_OE 	       = 0x0134
	OMAP_GPIO_SETDATAOUT   = 0x0194
	OMAP_GPIO_CLEARDATAOUT = 0x0190
	OMAP_GPIO_DATAIN       = 0x0138
};

/*GPIO Base Addresses */
enum GPIO_BASE_Addr {
	GPIO0		       = 0x44E06000
        GPIO1		       = 0x4804C000
	GPIO2		       = 0x481AC000
	GPIO3		       = 0x481AE000
};

/*MIFARE Status */
enum Status {
	MI_OK       = 0
	MI_NOTAGERR = 1
	MI_ERR      = 2
};

#define MAX_LEN 16

struct spi_dev {

	struct spi_device       *spi;
	
	/* for status readback */

	struct spi_transfer     status;
	struct spi_message      readback;
};

static void gpio_init_and_set(void)
{
	unsigned int data = 0;

	data = readl_relaxed(GPIO0 + OMAP_GPIO_OE);
	data = data & 0xFFFFFDFF;
	pr_debug("GPIO Init: Direction of pin is set: %x\n",data);
	writel_relaxed(data, gbank_base + OMAP_GPIO_OE);

	data = data | (1U << RST);

	pr_debug("GPIO Set: Direction of pin is set: %x\n",data);

	writel_relaxed(data, gbank_base + OMAP_GPIO_SETDATAOUT); //High
}
/* -----------------------------------------------------------------
 * Read from a register
 * -----------------------------------------------------------------*/
static int mfrc522_read_value(struct spi_device *spi, u8 reg)
{
	reg = (reg & 0x7E) | 0x80;
	pr_debug("%s: Value : %d\n",spi_w8r8(spi,reg));
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
	pr_debug("%s: Value : %d\n",spi_write_then_read(spi, buf, 2, NULL, 0));
	return spi_write_then_read(spi, buf, 2, NULL, 0);
}
/* -----------------------------------------------------------------
 * MFRC522 reset function
 * -----------------------------------------------------------------*/
static void MFRC522_Reset(static spi_device *spi)
{
	struct spi_dev *dev = spi_set_drvdata(spi);
	mfrc522_write_value(dev->spi,CommandReg,PCD_RESETPHASE);
	dev_info(&spi->dev,"Device Reset successfull\n");
}

/* -----------------------------------------------------------------
 * Set and clear BIT Mask
 * -----------------------------------------------------------------*/
static void  SetBitMask(static spi_device *spi,u8 reg,u8 mask)
{
	int temp = mfrc522_read_value(spi, reg);
	pr_debug("SeTBitMask: is %x\n",temp);
	mfrc522_write_value(spi, reg ,temp | mask);
}
static void  ClearBitMask(static spi_device *spi,u8 reg,u8 mask)
{
	int temp = mfrc522_read_value(spi, reg);
	pr_debug("ClearBitMask: is %x\n",temp);
	mfrc522_write_value(spi, reg, temp & (~mask));
}


/* -----------------------------------------------------------------
 * Antenna ON/OFF Function
 * -----------------------------------------------------------------*/
static void AntennaOn(static spi_device *spi)
{
	int temp = mfrc522_read_value(spi, TxControlReg);
	pr_debug("Antenna Reg Value %d\n",temp & 0x03);
	if (~(temp & 0x03))
		SeTBitMask(spi, TxControlReg, 0x03);
}

static void Antennaoff(static spi_device *spi)
{
	ClearBitMask(spi,TxControlReg,0x03);
}

static int MFRC522_ToCard(static spi_device *spi,u8 command,int *sendData)
{
	int backData[10];
	int backLen = 0
	int status = MI_ERR;
	int lastBits;
	u8 irqEn = 0x00;
	u8  waitIRq = 0x00;
	int i = 0;
	int n = 0; 
		
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
	SeTBitMask(spi, CommIEnReg, 0x80);
	ClearBitMask(spi, FIFOLevelReg, 0x80); //immediately clears the internal FIFO buffer’s

	/*No action, cancels current command Execution */
	mfrc522_write_value(spi, CommandReg, PCD_IDLE); 

	while (i < strlen(sendData))
		mfrc522_write_value(spi, FIFODataReg, sendData[i++]);
		
	mfrc522_write_value(spi, CommandReg, command);

	if (command == PCD_TRANSCEIVE)
		SeTBitMask(spi, BitFramingReg, 0x80); //page: 46
	
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
				status = MI_OK;
		if (n & irqEn & 0x01)
				status = MI_NOTAGERR;
		if (command == PCD_TRANSCEIVE) {
			n = mfrc522_read_value(spi , FIFOLevelReg);
			lastBits = mfrc522_read_value(spi, ControlReg) & 0x07; //pg 45
			if (lastBits != 0)
				backLen = (n-1)*8 + lastBits;
			else 
				backLen = n*8;
			if (n == 0)
				n = 1;
			if (n > MAX_LEN)
				n = MAX_LEN;
			i = 0;

			while (i < n)
				backData[i++] = mfrc522_read_value(spi, FIFODataReg);
		}
	}else{ 
			status = MI_ERR 
	}

	return TODO:--
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
	
	dev = (status spi_dev*)kmalloc(sizeof(struct spi_dev), GFP_KERNEL);	
	dev->spi = spi;
	
	/* device driver data */
	spi_set_drvdata(spi, dev);
	return 0;
}

static int mfrc522_remove(static spi_device *spi)
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
