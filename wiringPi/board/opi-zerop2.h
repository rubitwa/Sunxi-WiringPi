#define	OPI_ZEROP2_01	-1
#define	OPI_ZEROP2_03	GPIO_PA12
#define	OPI_ZEROP2_05	GPIO_PA11
#define	OPI_ZEROP2_07	GPIO_PA06
#define	OPI_ZEROP2_09	-1
#define	OPI_ZEROP2_11	GPIO_PL00
#define	OPI_ZEROP2_13	GPIO_PL01
#define	OPI_ZEROP2_15	GPIO_PA03
#define	OPI_ZEROP2_17	-1
#define	OPI_ZEROP2_19	GPIO_PA15
#define	OPI_ZEROP2_21	GPIO_PA16
#define	OPI_ZEROP2_23	GPIO_PA14
#define	OPI_ZEROP2_25	-1

#define	OPI_ZEROP2_02	-1
#define	OPI_ZEROP2_04	-1
#define	OPI_ZEROP2_06	-1
#define	OPI_ZEROP2_08	GPIO_PA00
#define	OPI_ZEROP2_10	GPIO_PA01
#define	OPI_ZEROP2_12	GPIO_PD11
#define	OPI_ZEROP2_14	-1
#define	OPI_ZEROP2_16	GPIO_PA19
#define	OPI_ZEROP2_18	GPIO_PA18
#define	OPI_ZEROP2_20	-1
#define	OPI_ZEROP2_22	GPIO_PA02
#define	OPI_ZEROP2_24	GPIO_PA13
#define	OPI_ZEROP2_26	GPIO_PD14
// endif


//map wpi gpio_num(index) to bp bpio_num(element)
int pinToGpio_OPI_ZEROP2 [64] =
{
   OPI_ZEROP2_11, OPI_ZEROP2_12,        //0, 1
   OPI_ZEROP2_13, OPI_ZEROP2_15,        //2, 3
   OPI_ZEROP2_16, OPI_ZEROP2_18,        //4, 5
   OPI_ZEROP2_22, OPI_ZEROP2_07,        //6, 7
   OPI_ZEROP2_03, OPI_ZEROP2_05,        //8, 9
   OPI_ZEROP2_24, OPI_ZEROP2_26,        //10, 11
   OPI_ZEROP2_19, OPI_ZEROP2_21,        //12, 13
   OPI_ZEROP2_23, OPI_ZEROP2_08,        //14, 15
   OPI_ZEROP2_10,        -1,        //16, 17
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
int pinTobcm_OPI_ZEROP2 [64] =
{
  -1, -1,  //0, 1
  OPI_ZEROP2_03, OPI_ZEROP2_05,  //2, 3
  OPI_ZEROP2_07, -1,  //4, 5
  -1, OPI_ZEROP2_26,  //6, 7
  OPI_ZEROP2_24, OPI_ZEROP2_21,  //8, 9
  OPI_ZEROP2_19, OPI_ZEROP2_23,  //10, 11
  OPI_ZEROP2_32, -1,  //12, 13
  OPI_ZEROP2_08, OPI_ZEROP2_10,  //14, 15
  -1, OPI_ZEROP2_11,  //16, 17
  OPI_ZEROP2_12, -1,	 //18, 19
  -1, -1,  //20, 21
  OPI_ZEROP2_15, OPI_ZEROP2_16,  //22, 23
  OPI_ZEROP2_18, OPI_ZEROP2_22,  //24, 25
  -1, OPI_ZEROP2_13,  //26, 27
  -1, -1,
  -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
} ;

//map phys_num(index) to bp gpio_num(element)
int physToGpio_OPI_ZEROP2 [64] =
{
          -1,                //0
          -1,        -1,     //1, 2
   OPI_ZEROP2_03,        -1,     //3, 4
   OPI_ZEROP2_05,        -1,     //5, 6
   OPI_ZEROP2_07, OPI_ZEROP2_08,     //7, 8
          -1, OPI_ZEROP2_10,     //9, 10
   OPI_ZEROP2_11, OPI_ZEROP2_12,     //11, 12
   OPI_ZEROP2_13,        -1,     //13, 14
   OPI_ZEROP2_15, OPI_ZEROP2_16,     //15, 16
          -1, OPI_ZEROP2_18,     //17, 18
   OPI_ZEROP2_19,        -1,     //19, 20
   OPI_ZEROP2_21, OPI_ZEROP2_22,     //21, 22
   OPI_ZEROP2_23, OPI_ZEROP2_24,     //23, 24
          -1, OPI_ZEROP2_26,     //25, 26
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

#define OPI_ZEROP2_I2C_DEV		"/dev/i2c-0"
#define OPI_ZEROP2_SPI_DEV		"/dev/spidev1.0"

#define OPI_ZEROP2_I2C_OFFSET  2
#define OPI_ZEROP2_SPI_OFFSET  3
#define OPI_ZEROP2_PWM_OFFSET  3
