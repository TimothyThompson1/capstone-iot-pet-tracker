#include "ArduCAM.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include <cstring>


ArduCAM::ArduCAM()
{
  sensor_model = OV5642;
  sensor_addr = 0x78;
}
ArduCAM::ArduCAM(byte model ,int CS)
{
	B_CS = CS;
	//*P_CS=CS;
  	sbi(P_CS, B_CS);
	sensor_model = model;
	switch (sensor_model)
	{
    case OV2640:
      	sensor_addr = 0x30;
    break;
     case OV5642:
      	sensor_addr = 0x78; // TODO:review
    break;
		default:
		  sensor_addr = 0x60;
		break;
	}	
}

void ArduCAM::InitCAM()
{
	wrSensorReg16_8(0x3008, 0x80);
	if (m_fmt == RAW)
	{
		//Init and set 640x480;
		wrSensorRegs16_8(OV5642_1280x960_RAW);	
		wrSensorRegs16_8(OV5642_640x480_RAW);	
	}
	else
	{	
		wrSensorRegs16_8(OV5642_QVGA_Preview);
		vTaskDelay(pdMS_TO_TICKS(100));
		if (m_fmt == JPEG)
		{
			vTaskDelay(pdMS_TO_TICKS(100));
			wrSensorRegs16_8(OV5642_JPEG_Capture_QSXGA);
			wrSensorRegs16_8(ov5642_320x240);
			vTaskDelay(pdMS_TO_TICKS(100));
			wrSensorReg16_8(0x3818, 0xa8);
			wrSensorReg16_8(0x3621, 0x10);
			wrSensorReg16_8(0x3801, 0xb0);
			wrSensorReg16_8(0x4407, 0x04);
		}
		else
		{
			byte reg_val;
			wrSensorReg16_8(0x4740, 0x21);
			wrSensorReg16_8(0x501e, 0x2a);
			wrSensorReg16_8(0x5002, 0xf8);
			wrSensorReg16_8(0x501f, 0x01);
			wrSensorReg16_8(0x4300, 0x61);
			rdSensorReg16_8(0x3818, &reg_val);
			wrSensorReg16_8(0x3818, (reg_val | 0x60) & 0xff);
			rdSensorReg16_8(0x3621, &reg_val);
			wrSensorReg16_8(0x3621, reg_val & 0xdf);
		}
	}
}

void ArduCAM::CS_HIGH()
{
	gpio_set_level(PIN_CS, 1);
}

void ArduCAM::CS_LOW()
{
	gpio_set_level(PIN_CS, 0);	
}

void ArduCAM::flush_fifo(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void ArduCAM::start_capture(void)
{
	write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}


void ArduCAM::clear_fifo_flag(void )
{
	write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}


uint8_t ArduCAM::read_fifo(void)
{
	uint8_t data;
	data = bus_read(SINGLE_FIFO_READ);
	return data;
}


uint8_t ArduCAM::read_reg(uint8_t addr)
{
  uint8_t value = 0;
	addr = addr& 0x7f;
 	cbi(P_CS, B_CS);
	spi_write_blocking(SPI_PORT, &addr, 1);
  spi_read_blocking(SPI_PORT, 0, &value, 1);
  sbi(P_CS, B_CS);
	return value;
}

void ArduCAM::write_reg(uint8_t addr, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = addr|WRITE_BIT ;  // remove read bit as this is a write
    buf[1] = data;
    cbi(P_CS, B_CS);
    spi_write_blocking(SPI_PORT, buf, 2);
    sbi(P_CS, B_CS);
    vTaskDelay(pdMS_TO_TICKS(1));
}


uint32_t ArduCAM::read_fifo_length(void)
{
	uint32_t len1,len2,len3,length=0;
	len1 = read_reg(FIFO_SIZE1);
  len2 = read_reg(FIFO_SIZE2);
  len3 = read_reg(FIFO_SIZE3) & 0x7f;
  length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return length;	
}

void ArduCAM::set_fifo_burst()
{
    uint8_t value;
    spi_read_blocking(SPI_PORT, BURST_FIFO_READ, &value, 1);	
}

//Set corresponding bit  
void ArduCAM::set_bit(uint8_t addr, uint8_t bit)
{
	uint8_t temp;
	temp = read_reg(addr);
	write_reg(addr, temp | bit);
}


//Clear corresponding bit 
void ArduCAM::clear_bit(uint8_t addr, uint8_t bit)
{
	uint8_t temp;
	temp = read_reg(addr);
	write_reg(addr, temp & (~bit));
}


//Get corresponding bit status
uint8_t ArduCAM::get_bit(uint8_t addr, uint8_t bit)
{
  uint8_t temp;
  temp = read_reg(addr);
  temp = temp & bit;
  return temp;
}

uint8_t ArduCAM::bus_write(int address,int value)
{	
	cbi(P_CS, B_CS);
		//SPI.transfer(address);
		//SPI.transfer(value);
	sbi(P_CS, B_CS);
	return 1;
}


uint8_t ArduCAM:: bus_read(int address)
{
	uint8_t value=0; //TODO:Review
	cbi(P_CS, B_CS);
	//	  SPI.transfer(address);
//		 value = SPI.transfer(0x00);
	sbi(P_CS, B_CS);
	return value;
}

	// Write 8 bit values to 8 bit register address
int ArduCAM::wrSensorRegs8_8(const struct sensor_reg reglist[])
{
		int err = 0;
	  uint16_t reg_addr = 0;
	  uint16_t reg_val = 0;
	  const struct sensor_reg *next = reglist;
	  reg_addr = next->reg;
	     reg_val = next->val;
	  while ((reg_addr != 0xff) | (reg_val != 0xff))
	  {
	    reg_addr = next->reg;
	    reg_val = next->val;
	    err = wrSensorReg8_8(reg_addr, reg_val);
	    next++;
	  }
	return 1;
}

	// Write 16 bit values to 8 bit register address
int ArduCAM::wrSensorRegs8_16(const struct sensor_reg reglist[])
{

		int err = 0;
	  unsigned int reg_addr, reg_val;
	  const struct sensor_reg *next = reglist;
	  reg_addr = next->reg;
	     reg_val = next->val;
	  while ((reg_addr != 0xff) | (reg_val != 0xffff))
	  {

	     reg_addr = next->reg;
	     reg_val = next->val;
	    err = wrSensorReg8_16(reg_addr, reg_val);
	    next++;
	  }
	return 1;
}
// Write 8 bit values to 16 bit register address
int ArduCAM::wrSensorRegs16_8(const struct sensor_reg reglist[])
{
		int err = 0;
	  unsigned int reg_addr;
	  unsigned char reg_val;
	  const struct sensor_reg *next = reglist;
	  reg_addr = next->reg;
	     reg_val = next->val;
	  while ((reg_addr != 0xffff) | (reg_val != 0xff))
	  {
	     reg_addr = next->reg;
	     reg_val = next->val;
	    err = wrSensorReg16_8(reg_addr, reg_val);
	    next++;
	  }
	return 1;
}

// Read/write 8 bit value to/from 16 bit register address
byte ArduCAM::wrSensorReg16_8(int regID, int regDat)
{
    uint8_t buf[3]={0};
    buf[0]=(regID >> 8)&0xff;
    buf[1]=(regID)&0xff;
    buf[2]=regDat;
    i2c_write_blocking(I2C_PORT, sensor_addr, buf,  3, true );
		vTaskDelay(pdMS_TO_TICKS(2));
	  return 1;
}

// Read/write 8 bit value to/from 8 bit register address	
byte ArduCAM::wrSensorReg8_8(int regID, int regDat)
{
uint8_t buf[2];
    buf[0] = regID;
    buf[1] = regDat;
    i2c_write_blocking(I2C_PORT, sensor_addr, buf,  2, true );
	return 1;
	
}

	void ArduCAM::OV2640_set_Special_effects(uint8_t Special_effect)
	{
// #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))	
		switch(Special_effect)
		{
			case Antique:
	
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x18);
				wrSensorReg8_8(0x7c, 0x05);
				wrSensorReg8_8(0x7d, 0x40);
				wrSensorReg8_8(0x7d, 0xa6);
			break;
			case Bluish:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x18);
				wrSensorReg8_8(0x7c, 0x05);
				wrSensorReg8_8(0x7d, 0xa0);
				wrSensorReg8_8(0x7d, 0x40);
			break;
			case Greenish:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x18);
				wrSensorReg8_8(0x7c, 0x05);
				wrSensorReg8_8(0x7d, 0x40);
				wrSensorReg8_8(0x7d, 0x40);
			break;
			case Reddish:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x18);
				wrSensorReg8_8(0x7c, 0x05);
				wrSensorReg8_8(0x7d, 0x40);
				wrSensorReg8_8(0x7d, 0xc0);
			break;
			case BW:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x18);
				wrSensorReg8_8(0x7c, 0x05);
				wrSensorReg8_8(0x7d, 0x80);
				wrSensorReg8_8(0x7d, 0x80);
			break;
			case Negative:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x40);
				wrSensorReg8_8(0x7c, 0x05);
				wrSensorReg8_8(0x7d, 0x80);
				wrSensorReg8_8(0x7d, 0x80);
			break;
			case BWnegative:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x58);
				wrSensorReg8_8(0x7c, 0x05);
				wrSensorReg8_8(0x7d, 0x80);
			  wrSensorReg8_8(0x7d, 0x80);
	
			break;
			case Normal:
		
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x00);
				wrSensorReg8_8(0x7c, 0x05);
				wrSensorReg8_8(0x7d, 0x80);
				wrSensorReg8_8(0x7d, 0x80);
			
			break;
					
		}
	// #endif
	}


	void ArduCAM::OV2640_set_Contrast(uint8_t Contrast)
	{
//  #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))	
		switch(Contrast)
		{
			case Contrast2:
		
			wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x04);
				wrSensorReg8_8(0x7c, 0x07);
				wrSensorReg8_8(0x7d, 0x20);
				wrSensorReg8_8(0x7d, 0x28);
				wrSensorReg8_8(0x7d, 0x0c);
				wrSensorReg8_8(0x7d, 0x06);
			break;
			case Contrast1:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x04);
				wrSensorReg8_8(0x7c, 0x07);
				wrSensorReg8_8(0x7d, 0x20);
				wrSensorReg8_8(0x7d, 0x24);
				wrSensorReg8_8(0x7d, 0x16);
				wrSensorReg8_8(0x7d, 0x06); 
			break;
			case Contrast0:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x04);
				wrSensorReg8_8(0x7c, 0x07);
				wrSensorReg8_8(0x7d, 0x20);
				wrSensorReg8_8(0x7d, 0x20);
				wrSensorReg8_8(0x7d, 0x20);
				wrSensorReg8_8(0x7d, 0x06); 
			break;
			case Contrast_1:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x04);
				wrSensorReg8_8(0x7c, 0x07);
				wrSensorReg8_8(0x7d, 0x20);
				wrSensorReg8_8(0x7d, 0x20);
				wrSensorReg8_8(0x7d, 0x2a);
		  wrSensorReg8_8(0x7d, 0x06);	
			break;
			case Contrast_2:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x04);
				wrSensorReg8_8(0x7c, 0x07);
				wrSensorReg8_8(0x7d, 0x20);
				wrSensorReg8_8(0x7d, 0x18);
				wrSensorReg8_8(0x7d, 0x34);
				wrSensorReg8_8(0x7d, 0x06);
			break;
		}
// #endif		
	}

	void ArduCAM::OV2640_set_Brightness(uint8_t Brightness)
	{
	// #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))
		switch(Brightness)
		{
			case Brightness2:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x04);
				wrSensorReg8_8(0x7c, 0x09);
				wrSensorReg8_8(0x7d, 0x40);
				wrSensorReg8_8(0x7d, 0x00);
			break;
			case Brightness1:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x04);
				wrSensorReg8_8(0x7c, 0x09);
				wrSensorReg8_8(0x7d, 0x30);
				wrSensorReg8_8(0x7d, 0x00);
			break;	
			case Brightness0:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x04);
				wrSensorReg8_8(0x7c, 0x09);
				wrSensorReg8_8(0x7d, 0x20);
				wrSensorReg8_8(0x7d, 0x00);
			break;
			case Brightness_1:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x04);
				wrSensorReg8_8(0x7c, 0x09);
				wrSensorReg8_8(0x7d, 0x10);
				wrSensorReg8_8(0x7d, 0x00);
			break;
			case Brightness_2:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x04);
				wrSensorReg8_8(0x7c, 0x09);
				wrSensorReg8_8(0x7d, 0x00);
				wrSensorReg8_8(0x7d, 0x00);
			break;	
		}
// #endif	
			
	}

	void ArduCAM::OV2640_set_Color_Saturation(uint8_t Color_Saturation)
	{
	// #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))
		switch(Color_Saturation)
		{
			case Saturation2:
			
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x02);
				wrSensorReg8_8(0x7c, 0x03);
				wrSensorReg8_8(0x7d, 0x68);
				wrSensorReg8_8(0x7d, 0x68);
			break;
			case Saturation1:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x02);
				wrSensorReg8_8(0x7c, 0x03);
				wrSensorReg8_8(0x7d, 0x58);
				wrSensorReg8_8(0x7d, 0x58);
			break;
			case Saturation0:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x02);
				wrSensorReg8_8(0x7c, 0x03);
				wrSensorReg8_8(0x7d, 0x48);
				wrSensorReg8_8(0x7d, 0x48);
			break;
			case Saturation_1:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x02);
				wrSensorReg8_8(0x7c, 0x03);
				wrSensorReg8_8(0x7d, 0x38);
				wrSensorReg8_8(0x7d, 0x38);
			break;
			case Saturation_2:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0x7c, 0x00);
				wrSensorReg8_8(0x7d, 0x02);
				wrSensorReg8_8(0x7c, 0x03);
				wrSensorReg8_8(0x7d, 0x28);
				wrSensorReg8_8(0x7d, 0x28);
			break;	
		}
// #endif	
	}

void ArduCAM::OV2640_set_Light_Mode(uint8_t Light_Mode)
	{
//  #if (defined (OV2640_CAM)||defined (OV2640_MINI_2MP)||defined (OV2640_MINI_2MP_PLUS))
		 switch(Light_Mode)
		 {
			
			  case Auto:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0xc7, 0x00); //AWB on
			  break;
			  case Sunny:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0xc7, 0x40); //AWB off
			  wrSensorReg8_8(0xcc, 0x5e);
				wrSensorReg8_8(0xcd, 0x41);
				wrSensorReg8_8(0xce, 0x54);
			  break;
			  case Cloudy:
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0xc7, 0x40); //AWB off
				wrSensorReg8_8(0xcc, 0x65);
				wrSensorReg8_8(0xcd, 0x41);
				wrSensorReg8_8(0xce, 0x4f);  
			  break;
			  case Office:
			  wrSensorReg8_8(0xff, 0x00);
			  wrSensorReg8_8(0xc7, 0x40); //AWB off
			  wrSensorReg8_8(0xcc, 0x52);
			  wrSensorReg8_8(0xcd, 0x41);
			  wrSensorReg8_8(0xce, 0x66);
			  break;
			  case Home:
			  wrSensorReg8_8(0xff, 0x00);
			  wrSensorReg8_8(0xc7, 0x40); //AWB off
			  wrSensorReg8_8(0xcc, 0x42);
			  wrSensorReg8_8(0xcd, 0x3f);
			  wrSensorReg8_8(0xce, 0x71);
			  break;
			  default :
				wrSensorReg8_8(0xff, 0x00);
				wrSensorReg8_8(0xc7, 0x00); //AWB on
			  break; 
		 }	
// #endif
	}

byte ArduCAM::rdSensorReg8_8(uint8_t regID, uint8_t* regDat)
{	
  i2c_write_blocking(I2C_PORT, sensor_addr, &regID, 1, true );
  i2c_read_blocking(I2C_PORT, sensor_addr, regDat,  1, false );
  return 1;
	
}

byte ArduCAM::rdSensorReg16_8(uint16_t regID, uint8_t* regDat)
{
	uint8_t buffer[2]={0};
	buffer[0]=(regID>>8)&0xff;
	buffer[1]=regID&0xff;
	i2c_write_blocking(I2C_PORT, sensor_addr, buffer, 2, true );
//	i2c_write_blocking(I2C_PORT, sensor_addr, &low, 1, true );
	i2c_read_blocking(I2C_PORT, sensor_addr, regDat,  1, false );
	return 1;
}

void ArduCAM::set_format(byte fmt)
{
  if (fmt == BMP)
    m_fmt = BMP;
  else if(fmt == RAW)
    m_fmt = RAW;
  else
    m_fmt = JPEG;
}
unsigned char usart_symbol=0;
unsigned char usart_Command = 0;
// RX interrupt handler
void on_uart_rx() {
    // while (uart_is_readable(UART_ID)) {
    //    usart_Command = uart_getc(UART_ID);
    //    usart_symbol=1;
    // }
}



void ArduCAM:: Arducam_init(void)
{
    // This  will use I2C0 on GPIO4 (SDA) and GPIO5 (SCL)
	i2c_master_init();
  configure_GPIO();
  // Make the I2C pins available to picotool
  //bi_decl( bi_2pins_with_func(PIN_SDA, PIN_SCL, GPIO_FUNC_I2C));
    // This example will use SPI0 at 0.5MHz.
  init_spi();
}

void ArduCAM::OV5642_set_JPEG_size(uint8_t size)
{
  uint8_t reg_val;

  switch (size)
  {
    case OV5642_320x240:
      wrSensorRegs16_8(ov5642_320x240);
      break;
    case OV5642_640x480:
      wrSensorRegs16_8(ov5642_640x480);
      break;
    case OV5642_1024x768:
      wrSensorRegs16_8(ov5642_1024x768);
      break;
    case OV5642_1280x960:
      wrSensorRegs16_8(ov5642_1280x960);
      break;
    case OV5642_1600x1200:
      wrSensorRegs16_8(ov5642_1600x1200);
      break;
    case OV5642_2048x1536:
      wrSensorRegs16_8(ov5642_2048x1536);
      break;
    case OV5642_2592x1944:
      wrSensorRegs16_8(ov5642_2592x1944);
      break;
    default:
      wrSensorRegs16_8(ov5642_320x240);
      break;
  }
//#endif
}

void ArduCAM::OV5642_set_Light_Mode(uint8_t Light_Mode)
{
		switch(Light_Mode)
		{
		
			case Advanced_AWB:
			wrSensorReg16_8(0x3406 ,0x0 );
			wrSensorReg16_8(0x5192 ,0x04);
			wrSensorReg16_8(0x5191 ,0xf8);
			wrSensorReg16_8(0x518d ,0x26);
			wrSensorReg16_8(0x518f ,0x42);
			wrSensorReg16_8(0x518e ,0x2b);
			wrSensorReg16_8(0x5190 ,0x42);
			wrSensorReg16_8(0x518b ,0xd0);
			wrSensorReg16_8(0x518c ,0xbd);
			wrSensorReg16_8(0x5187 ,0x18);
			wrSensorReg16_8(0x5188 ,0x18);
			wrSensorReg16_8(0x5189 ,0x56);
			wrSensorReg16_8(0x518a ,0x5c);
			wrSensorReg16_8(0x5186 ,0x1c);
			wrSensorReg16_8(0x5181 ,0x50);
			wrSensorReg16_8(0x5184 ,0x20);
			wrSensorReg16_8(0x5182 ,0x11);
			wrSensorReg16_8(0x5183 ,0x0 );	
			break;
			case Simple_AWB:
			wrSensorReg16_8(0x3406 ,0x00);
			wrSensorReg16_8(0x5183 ,0x80);
			wrSensorReg16_8(0x5191 ,0xff);
			wrSensorReg16_8(0x5192 ,0x00);
			break;
			case Manual_day:
			wrSensorReg16_8(0x3406 ,0x1 );
			wrSensorReg16_8(0x3400 ,0x7 );
			wrSensorReg16_8(0x3401 ,0x32);
			wrSensorReg16_8(0x3402 ,0x4 );
			wrSensorReg16_8(0x3403 ,0x0 );
			wrSensorReg16_8(0x3404 ,0x5 );
			wrSensorReg16_8(0x3405 ,0x36);
			break;
			case Manual_A:
			wrSensorReg16_8(0x3406 ,0x1 );
			wrSensorReg16_8(0x3400 ,0x4 );
			wrSensorReg16_8(0x3401 ,0x88);
			wrSensorReg16_8(0x3402 ,0x4 );
			wrSensorReg16_8(0x3403 ,0x0 );
			wrSensorReg16_8(0x3404 ,0x8 );
			wrSensorReg16_8(0x3405 ,0xb6);
			break;
			case Manual_cwf:
			wrSensorReg16_8(0x3406 ,0x1 );
			wrSensorReg16_8(0x3400 ,0x6 );
			wrSensorReg16_8(0x3401 ,0x13);
			wrSensorReg16_8(0x3402 ,0x4 );
			wrSensorReg16_8(0x3403 ,0x0 );
			wrSensorReg16_8(0x3404 ,0x7 );
			wrSensorReg16_8(0x3405 ,0xe2);
			break;
			case Manual_cloudy:
			wrSensorReg16_8(0x3406 ,0x1 );
			wrSensorReg16_8(0x3400 ,0x7 );
			wrSensorReg16_8(0x3401 ,0x88);
			wrSensorReg16_8(0x3402 ,0x4 );
			wrSensorReg16_8(0x3403 ,0x0 );
			wrSensorReg16_8(0x3404 ,0x5 );
			wrSensorReg16_8(0x3405 ,0x0);
			break;
			default :
			break; 
		}	
}


void ArduCAM::OV5642_set_Color_Saturation(uint8_t Color_Saturation)
{
	
		switch(Color_Saturation)
		{
			case Saturation4:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5583 ,0x80);
				wrSensorReg16_8(0x5584 ,0x80);
				wrSensorReg16_8(0x5580 ,0x02);
			break;
			case Saturation3:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5583 ,0x70);
				wrSensorReg16_8(0x5584 ,0x70);
				wrSensorReg16_8(0x5580 ,0x02);
			break;
			case Saturation2:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5583 ,0x60);
				wrSensorReg16_8(0x5584 ,0x60);
				wrSensorReg16_8(0x5580 ,0x02);
			break;
			case Saturation1:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5583 ,0x50);
				wrSensorReg16_8(0x5584 ,0x50);
				wrSensorReg16_8(0x5580 ,0x02);
			break;
			case Saturation0:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5583 ,0x40);
				wrSensorReg16_8(0x5584 ,0x40);
				wrSensorReg16_8(0x5580 ,0x02);
			break;		
			case Saturation_1:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5583 ,0x30);
				wrSensorReg16_8(0x5584 ,0x30);
				wrSensorReg16_8(0x5580 ,0x02);
			break;
				case Saturation_2:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5583 ,0x20);
				wrSensorReg16_8(0x5584 ,0x20);
				wrSensorReg16_8(0x5580 ,0x02);
			break;
				case Saturation_3:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5583 ,0x10);
				wrSensorReg16_8(0x5584 ,0x10);
				wrSensorReg16_8(0x5580 ,0x02);
			break;
				case Saturation_4:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5583 ,0x00);
				wrSensorReg16_8(0x5584 ,0x00);
				wrSensorReg16_8(0x5580 ,0x02);
			break;
		}	
}


void ArduCAM::OV5642_set_Brightness(uint8_t Brightness)
{
	
		switch(Brightness)
		{
			case Brightness4:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5589 ,0x40);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x558a ,0x00);
			break;
			case Brightness3:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5589 ,0x30);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x558a ,0x00);
			break;	
			case Brightness2:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5589 ,0x20);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x558a ,0x00);
			break;
			case Brightness1:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5589 ,0x10);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x558a ,0x00);
			break;
			case Brightness0:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5589 ,0x00);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x558a ,0x00);
			break;	
			case Brightness_1:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5589 ,0x10);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x558a ,0x08);
			break;	
			case Brightness_2:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5589 ,0x20);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x558a ,0x08);
			break;	
			case Brightness_3:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5589 ,0x30);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x558a ,0x08);
			break;	
			case Brightness_4:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5589 ,0x40);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x558a ,0x08);
			break;	
		}	
}


void ArduCAM::OV5642_set_Contrast(uint8_t Contrast)
{
		switch(Contrast)
		{
			case Contrast4:
			wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x5587 ,0x30);
				wrSensorReg16_8(0x5588 ,0x30);
				wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast3:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x5587 ,0x2c);
				wrSensorReg16_8(0x5588 ,0x2c);
				wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast2:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x5587 ,0x28);
				wrSensorReg16_8(0x5588 ,0x28);
				wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast1:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x5587 ,0x24);
				wrSensorReg16_8(0x5588 ,0x24);
				wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast0:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x5587 ,0x20);
				wrSensorReg16_8(0x5588 ,0x20);
				wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast_1:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x5587 ,0x1C);
				wrSensorReg16_8(0x5588 ,0x1C);
				wrSensorReg16_8(0x558a ,0x1C);
			break;
			case Contrast_2:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x5587 ,0x18);
				wrSensorReg16_8(0x5588 ,0x18);
				wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast_3:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x5587 ,0x14);
				wrSensorReg16_8(0x5588 ,0x14);
				wrSensorReg16_8(0x558a ,0x00);
			break;
			case Contrast_4:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x04);
				wrSensorReg16_8(0x5587 ,0x10);
				wrSensorReg16_8(0x5588 ,0x10);
				wrSensorReg16_8(0x558a ,0x00);
			break;
		}
}


void ArduCAM::OV5642_set_hue(uint8_t degree)
{
		switch(degree)
		{
			case degree_180:
			wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x80);
				wrSensorReg16_8(0x5582 ,0x00);
				wrSensorReg16_8(0x558a ,0x32);
			break;
			case degree_150:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x6f);
				wrSensorReg16_8(0x5582 ,0x40);
				wrSensorReg16_8(0x558a ,0x32);
			break;
			case degree_120:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x40);
				wrSensorReg16_8(0x5582 ,0x6f);
				wrSensorReg16_8(0x558a ,0x32);
			break;
			case degree_90:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x00);
				wrSensorReg16_8(0x5582 ,0x80);
				wrSensorReg16_8(0x558a ,0x02);
			break;
			case degree_60:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x40);
				wrSensorReg16_8(0x5582 ,0x6f);
				wrSensorReg16_8(0x558a ,0x02);
			break;
			case degree_30:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x6f);
				wrSensorReg16_8(0x5582 ,0x40);
				wrSensorReg16_8(0x558a ,0x02);
			break;
			case degree_0:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x80);
				wrSensorReg16_8(0x5582 ,0x00);
				wrSensorReg16_8(0x558a ,0x01);
			break;
			case degree30:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x6f);
				wrSensorReg16_8(0x5582 ,0x40);
				wrSensorReg16_8(0x558a ,0x01);
			break;
			case degree60:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x40);
				wrSensorReg16_8(0x5582 ,0x6f);
				wrSensorReg16_8(0x558a ,0x01);
			break;
			case degree90:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x00);
				wrSensorReg16_8(0x5582 ,0x80);
				wrSensorReg16_8(0x558a ,0x31);
			break;
			case degree120:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x40);
				wrSensorReg16_8(0x5582 ,0x6f);
				wrSensorReg16_8(0x558a ,0x31);
			break;
			case degree150:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x01);
				wrSensorReg16_8(0x5581 ,0x6f);
				wrSensorReg16_8(0x5582 ,0x40);
				wrSensorReg16_8(0x558a ,0x31);
			break;
		}
		
}


void ArduCAM::OV5642_set_Special_effects(uint8_t Special_effect)
{
		switch(Special_effect)
		{
			case Bluish:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x18);
				wrSensorReg16_8(0x5585 ,0xa0);
				wrSensorReg16_8(0x5586 ,0x40);
			break;
			case Greenish:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x18);
				wrSensorReg16_8(0x5585 ,0x60);
				wrSensorReg16_8(0x5586 ,0x60);
			break;
			case Reddish:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x18);
				wrSensorReg16_8(0x5585 ,0x80);
				wrSensorReg16_8(0x5586 ,0xc0);
			break;
			case BW:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x18);
				wrSensorReg16_8(0x5585 ,0x80);
				wrSensorReg16_8(0x5586 ,0x80);
			break;
			case Negative:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x40);
			break;
			
				case Sepia:
				wrSensorReg16_8(0x5001 ,0xff);
				wrSensorReg16_8(0x5580 ,0x18);
				wrSensorReg16_8(0x5585 ,0x40);
				wrSensorReg16_8(0x5586 ,0xa0);
			break;
			case Normal:
				wrSensorReg16_8(0x5001 ,0x7f);
				wrSensorReg16_8(0x5580 ,0x00);		
			break;		
		}
}


void ArduCAM::OV5642_set_Exposure_level(uint8_t level)
{
		switch(level)
		{
			case Exposure_17_EV:
			  wrSensorReg16_8(0x3a0f ,0x10);
				wrSensorReg16_8(0x3a10 ,0x08);
				wrSensorReg16_8(0x3a1b ,0x10);
				wrSensorReg16_8(0x3a1e ,0x08);
				wrSensorReg16_8(0x3a11 ,0x20);
				wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure_13_EV:
				wrSensorReg16_8(0x3a0f ,0x18);
				wrSensorReg16_8(0x3a10 ,0x10);
				wrSensorReg16_8(0x3a1b ,0x18);
				wrSensorReg16_8(0x3a1e ,0x10);
				wrSensorReg16_8(0x3a11 ,0x30);
				wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure_10_EV:
				wrSensorReg16_8(0x3a0f ,0x20);
				wrSensorReg16_8(0x3a10 ,0x18);
				wrSensorReg16_8(0x3a11 ,0x41);
				wrSensorReg16_8(0x3a1b ,0x20);
				wrSensorReg16_8(0x3a1e ,0x18);
				wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure_07_EV:
				wrSensorReg16_8(0x3a0f ,0x28);
				wrSensorReg16_8(0x3a10 ,0x20);
				wrSensorReg16_8(0x3a11 ,0x51);
				wrSensorReg16_8(0x3a1b ,0x28);
				wrSensorReg16_8(0x3a1e ,0x20);
				wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure_03_EV:
				wrSensorReg16_8(0x3a0f ,0x30);
				wrSensorReg16_8(0x3a10 ,0x28);
				wrSensorReg16_8(0x3a11 ,0x61);
				wrSensorReg16_8(0x3a1b ,0x30);
				wrSensorReg16_8(0x3a1e ,0x28);
				wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure_default:
				wrSensorReg16_8(0x3a0f ,0x38);
				wrSensorReg16_8(0x3a10 ,0x30);
				wrSensorReg16_8(0x3a11 ,0x61);
				wrSensorReg16_8(0x3a1b ,0x38);
				wrSensorReg16_8(0x3a1e ,0x30);
				wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure03_EV:
				wrSensorReg16_8(0x3a0f ,0x40);
				wrSensorReg16_8(0x3a10 ,0x38);
				wrSensorReg16_8(0x3a11 ,0x71);
				wrSensorReg16_8(0x3a1b ,0x40);
				wrSensorReg16_8(0x3a1e ,0x38);
				wrSensorReg16_8(0x3a1f ,0x10);
			break;
			case Exposure07_EV:
				wrSensorReg16_8(0x3a0f ,0x48);
				wrSensorReg16_8(0x3a10 ,0x40);
				wrSensorReg16_8(0x3a11 ,0x80);
				wrSensorReg16_8(0x3a1b ,0x48);
				wrSensorReg16_8(0x3a1e ,0x40);
				wrSensorReg16_8(0x3a1f ,0x20);
			break;
			case Exposure10_EV:
				wrSensorReg16_8(0x3a0f ,0x50);
				wrSensorReg16_8(0x3a10 ,0x48);
				wrSensorReg16_8(0x3a11 ,0x90);
				wrSensorReg16_8(0x3a1b ,0x50);
				wrSensorReg16_8(0x3a1e ,0x48);
				wrSensorReg16_8(0x3a1f ,0x20);
			break;
			case Exposure13_EV:
				wrSensorReg16_8(0x3a0f ,0x58);
				wrSensorReg16_8(0x3a10 ,0x50);
				wrSensorReg16_8(0x3a11 ,0x91);
				wrSensorReg16_8(0x3a1b ,0x58);
				wrSensorReg16_8(0x3a1e ,0x50);
				wrSensorReg16_8(0x3a1f ,0x20);
			break;
			case Exposure17_EV:
				wrSensorReg16_8(0x3a0f ,0x60);
				wrSensorReg16_8(0x3a10 ,0x58);
				wrSensorReg16_8(0x3a11 ,0xa0);
				wrSensorReg16_8(0x3a1b ,0x60);
				wrSensorReg16_8(0x3a1e ,0x58);
				wrSensorReg16_8(0x3a1f ,0x20);
			break;
		}
}


void ArduCAM::OV5642_set_Sharpness(uint8_t Sharpness)
{
		switch(Sharpness)
		{
			case Auto_Sharpness_default:
			wrSensorReg16_8(0x530A ,0x00);
				wrSensorReg16_8(0x530c ,0x0 );
				wrSensorReg16_8(0x530d ,0xc );
				wrSensorReg16_8(0x5312 ,0x40);
			break;
			case Auto_Sharpness1:
				wrSensorReg16_8(0x530A ,0x00);
				wrSensorReg16_8(0x530c ,0x4 );
				wrSensorReg16_8(0x530d ,0x18);
				wrSensorReg16_8(0x5312 ,0x20);
			break;
			case Auto_Sharpness2:
				wrSensorReg16_8(0x530A ,0x00);
				wrSensorReg16_8(0x530c ,0x8 );
				wrSensorReg16_8(0x530d ,0x30);
				wrSensorReg16_8(0x5312 ,0x10);
			break;
			case Manual_Sharpnessoff:
				wrSensorReg16_8(0x530A ,0x08);
				wrSensorReg16_8(0x531e ,0x00);
				wrSensorReg16_8(0x531f ,0x00);
			break;
			case Manual_Sharpness1:
				wrSensorReg16_8(0x530A ,0x08);
				wrSensorReg16_8(0x531e ,0x04);
				wrSensorReg16_8(0x531f ,0x04);
			break;
			case Manual_Sharpness2:
				wrSensorReg16_8(0x530A ,0x08);
				wrSensorReg16_8(0x531e ,0x08);
				wrSensorReg16_8(0x531f ,0x08);
			break;
			case Manual_Sharpness3:
				wrSensorReg16_8(0x530A ,0x08);
				wrSensorReg16_8(0x531e ,0x0c);
				wrSensorReg16_8(0x531f ,0x0c);
			break;
			case Manual_Sharpness4:
				wrSensorReg16_8(0x530A ,0x08);
				wrSensorReg16_8(0x531e ,0x0f);
				wrSensorReg16_8(0x531f ,0x0f);
			break;
			case Manual_Sharpness5:
				wrSensorReg16_8(0x530A ,0x08);
				wrSensorReg16_8(0x531e ,0x1f);
				wrSensorReg16_8(0x531f ,0x1f);
			break;
		}
}


void ArduCAM::OV5642_set_Mirror_Flip(uint8_t Mirror_Flip)
{
			 uint8_t reg_val;
	switch(Mirror_Flip)
		{
			case MIRROR:
				rdSensorReg16_8(0x3818,&reg_val);
				reg_val = reg_val|0x00;
				reg_val = reg_val&0x9F;
			wrSensorReg16_8(0x3818 ,reg_val);
			rdSensorReg16_8(0x3621,&reg_val);
				reg_val = reg_val|0x20;
				wrSensorReg16_8(0x3621, reg_val );
			
			break;
			case FLIP:
				rdSensorReg16_8(0x3818,&reg_val);
				reg_val = reg_val|0x20;
				reg_val = reg_val&0xbF;
			wrSensorReg16_8(0x3818 ,reg_val);
			rdSensorReg16_8(0x3621,&reg_val);
				reg_val = reg_val|0x20;
				wrSensorReg16_8(0x3621, reg_val );
			break;
			case MIRROR_FLIP:
			 rdSensorReg16_8(0x3818,&reg_val);
				reg_val = reg_val|0x60;
				reg_val = reg_val&0xFF;
			wrSensorReg16_8(0x3818 ,reg_val);
			rdSensorReg16_8(0x3621,&reg_val);
				reg_val = reg_val&0xdf;
				wrSensorReg16_8(0x3621, reg_val );
			break;
			case Normal:
				  rdSensorReg16_8(0x3818,&reg_val);
				reg_val = reg_val|0x40;
				reg_val = reg_val&0xdF;
			wrSensorReg16_8(0x3818 ,reg_val);
			rdSensorReg16_8(0x3621,&reg_val);
				reg_val = reg_val&0xdf;
				wrSensorReg16_8(0x3621, reg_val );
			break;
		}
}


void ArduCAM::OV5642_set_Compress_quality(uint8_t quality)
{
	switch(quality)
		{
			case high_quality:
				wrSensorReg16_8(0x4407, 0x02);
				break;
			case default_quality:
				wrSensorReg16_8(0x4407, 0x04);
				break;
			case low_quality:
				wrSensorReg16_8(0x4407, 0x08);
				break;
		}
}


void ArduCAM::OV5642_Test_Pattern(uint8_t Pattern)
{	
	  switch(Pattern)
		{
			case Color_bar:
				wrSensorReg16_8(0x503d , 0x80);
				wrSensorReg16_8(0x503e, 0x00);
				break;
			case Color_square:
				wrSensorReg16_8(0x503d , 0x85);
				wrSensorReg16_8(0x503e, 0x12);
				break;
			case BW_square:
				wrSensorReg16_8(0x503d , 0x85);
				wrSensorReg16_8(0x503e, 0x1a);
				break;
			case DLI:
				wrSensorReg16_8(0x4741 , 0x4);
				break;
		}
}


void ArduCAM::configure_GPIO()
{
    //gpio_pad_select_gpio(PIN_CS);
	esp_rom_gpio_pad_select_gpio((gpio_num_t)PIN_CS);
    gpio_set_direction((gpio_num_t)PIN_CS, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_CS, 1); // Set CS high initially
}


// I2C initialization
void ArduCAM::i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = PIN_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

// SPI initialization function
void ArduCAM::init_spi() {
    // Initialize SPI bus and device as described earlier
    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_MOSI,
		.miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .mode = 0,
        .clock_speed_hz = 4 * 1000 * 1000, // 10MHz
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };
    spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO); // TODO verify
    spi_bus_add_device(SPI2_HOST, &dev_config, &spi);
}

esp_err_t ArduCAM::spi_write_blocking(spi_device_handle_t spi, const uint8_t *src, size_t len) {
	spi_transaction_t t;
	std::memset(&t, 0, sizeof(t));  // Zero out the transaction structure

	t.length = len * 8;  // Length in bits
	t.tx_buffer = src;
	t.rx_buffer = NULL;  // No data is expected back

	esp_err_t ret = spi_device_transmit(spi, &t);
	return ret;
}

esp_err_t ArduCAM::spi_read_blocking(spi_device_handle_t spi, uint8_t repeated_tx_data, uint8_t *dst, size_t len) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));  // Zero out the transaction structure

    t.length = len * 8;  // Length in bits
    t.tx_buffer = &repeated_tx_data;  // Data to be sent (repeated_tx_data)
    t.rx_buffer = dst;  // Buffer to receive data

    esp_err_t ret = spi_device_transmit(spi, &t);
    return ret;
}

esp_err_t ArduCAM::i2c_write_blocking(i2c_port_t i2c_num, uint8_t addr, const uint8_t *src, size_t len, bool nostop) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();  // Create an I2C command link
    esp_err_t ret;

    i2c_master_start(cmd);  // Start the I2C transaction
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);  // Send device address with write bit

    // Write data bytes
    i2c_master_write(cmd, (uint8_t *)src, len, nostop ? true : false);

    if (!nostop) {
        i2c_master_stop(cmd);  // Stop the I2C transaction if nostop is false
    }

    ret = i2c_master_cmd_begin(i2c_num, cmd, portMAX_DELAY);  // Execute the I2C command

    i2c_cmd_link_delete(cmd);  // Delete the I2C command link

    return ret;
}

esp_err_t ArduCAM::i2c_read_blocking(i2c_port_t i2c_num, uint8_t addr, uint8_t *dst, size_t len, bool nostop) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();  // Create an I2C command link
    esp_err_t ret;

    i2c_master_start(cmd);  // Start the I2C transaction
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);  // Send device address with read bit

    if (len > 1) {
        i2c_master_read(cmd, dst, len - 1, I2C_MASTER_ACK);  // Read bytes with ACK
    }
    i2c_master_read_byte(cmd, dst + len - 1, I2C_MASTER_NACK);  // Read the last byte with NACK

    if (!nostop) {
        i2c_master_stop(cmd);  // Stop the I2C transaction if nostop is false
    }

    ret = i2c_master_cmd_begin(i2c_num, cmd, portMAX_DELAY);  // Execute the I2C command

    i2c_cmd_link_delete(cmd);  // Delete the I2C command link

    return ret;
}