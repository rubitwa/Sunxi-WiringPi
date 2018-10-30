#define	OPI_ONEP_01	-1
#define	OPI_ONEP_03	GPIO_PH06
#define	OPI_ONEP_05	GPIO_PH05
#define	OPI_ONEP_07	GPIO_PH04
#define	OPI_ONEP_09	-1
#define	OPI_ONEP_11	GPIO_PD24
#define	OPI_ONEP_13	GPIO_PD23
#define	OPI_ONEP_15	GPIO_PD26
#define	OPI_ONEP_17	-1
#define	OPI_ONEP_19	GPIO_PC02
#define	OPI_ONEP_21	GPIO_PC03
#define	OPI_ONEP_23	GPIO_PC00
#define	OPI_ONEP_25	-1

#define	OPI_ONEP_02	-1
#define	OPI_ONEP_04	-1
#define	OPI_ONEP_06	-1
#define	OPI_ONEP_08	GPIO_PD21
#define	OPI_ONEP_10	GPIO_PD22
#define	OPI_ONEP_12	GPIO_PC09
#define	OPI_ONEP_14	-1
#define	OPI_ONEP_16	GPIO_PC08
#define	OPI_ONEP_18	GPIO_PC07
#define	OPI_ONEP_20	-1
#define	OPI_ONEP_22	GPIO_PD25
#define	OPI_ONEP_24	GPIO_PC05
#define	OPI_ONEP_26	GPIO_PH03

//map wpi gpio_num(index) to bp bpio_num(element)
int pinToGpio_OPI_ONEP [64] =
{
   OPI_ONEP_11, OPI_ONEP_12,        //0, 1
   OPI_ONEP_13, OPI_ONEP_15,        //2, 3
   OPI_ONEP_16, OPI_ONEP_18,        //4, 5
   OPI_ONEP_22, OPI_ONEP_07,        //6, 7
   OPI_ONEP_03, OPI_ONEP_05,        //8, 9
   OPI_ONEP_24, OPI_ONEP_26,        //10, 11
   OPI_ONEP_19, OPI_ONEP_21,        //12, 13
   OPI_ONEP_23, OPI_ONEP_08,        //14, 15
   OPI_ONEP_10,        -1,        //16, 17
          -1,        -1,        //18, 19
          -1, -1,        //20, 21
   -1, -1,        //22, 23
   -1, -1,        //24, 25
   -1, -1,        //26, 27
   -1, -1,        //28. 29
   -1, -1,        //30, 31
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
} ;

//map bcm gpio_num(index) to bp gpio_num(element)
int pinTobcm_OPI_ONEP [64] =
{
  -1, -1,  //0, 1
  OPI_ONEP_03, OPI_ONEP_05,  //2, 3
  OPI_ONEP_07, -1,  //4, 5
  -1, OPI_ONEP_26,  //6, 7
  OPI_ONEP_24, OPI_ONEP_21,  //8, 9
  OPI_ONEP_19, OPI_ONEP_23,  //10, 11
  -1, -1,  //12, 13
  OPI_ONEP_08, OPI_ONEP_10,  //14, 15
  -1, OPI_ONEP_11,  //16, 17
  OPI_ONEP_12, -1,	 //18, 19
  -1, -1,  //20, 21
  OPI_ONEP_15, OPI_ONEP_16,  //22, 23
  OPI_ONEP_18, OPI_ONEP_22,  //24, 25
  -1, OPI_ONEP_13,  //26, 27
  -1, -1,
  -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
} ;

//map phys_num(index) to bp gpio_num(element)
int physToGpio_OPI_ONEP [64] =
{
          -1,                //0
          -1,        -1,     //1, 2
   OPI_ONEP_03,        -1,     //3, 4
   OPI_ONEP_05,        -1,     //5, 6
   OPI_ONEP_07, OPI_ONEP_08,     //7, 8
          -1, OPI_ONEP_10,     //9, 10
   OPI_ONEP_11, OPI_ONEP_12,     //11, 12
   OPI_ONEP_13,        -1,     //13, 14
   OPI_ONEP_15, OPI_ONEP_16,     //15, 16
          -1, OPI_ONEP_18,     //17, 18
   OPI_ONEP_19,        -1,     //19, 20
   OPI_ONEP_21, OPI_ONEP_22,     //21, 22
   OPI_ONEP_23, OPI_ONEP_24,     //23, 24
          -1, OPI_ONEP_26,     //25, 26
   -1, -1,     //27, 28
   -1,        -1,     //29, 30
   -1, -1,     //31, 32      
   -1,        -1,     //33, 34
   -1, -1,     //35, 36
   -1, -1,     //37, 38
          -1, -1,     //39, 40
   -1,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
   -1,   -1, -1, -1, -1, -1, -1, -1 // 56-> 63
} ;

#define OPI_ONEP_I2C_DEV		"/dev/i2c-0"
#define OPI_ONEP_SPI_DEV		"/dev/spidev0.0"

#define OPI_ONEP_I2C_OFFSET  2
#define OPI_ONEP_SPI_OFFSET  3
#define OPI_ONEP_PWM_OFFSET  3
