#define	OPI_PC_01	-1
#define	OPI_PC_03	GPIO_PA12
#define	OPI_PC_05	GPIO_PA11
#define	OPI_PC_07	GPIO_PA06
#define	OPI_PC_09	-1
#define	OPI_PC_11	GPIO_PA01
#define	OPI_PC_13	GPIO_PA00
#define	OPI_PC_15	GPIO_PA03
#define	OPI_PC_17	-1
#define	OPI_PC_19	GPIO_PC00
#define	OPI_PC_21	GPIO_PC01
#define	OPI_PC_23	GPIO_PC02
#define	OPI_PC_25	-1
#define	OPI_PC_27	GPIO_PA19
#define	OPI_PC_29	GPIO_PA07
#define	OPI_PC_31	GPIO_PA08
#define	OPI_PC_33	GPIO_PA09
#define	OPI_PC_35	GPIO_PA10
#define	OPI_PC_37	GPIO_PA20
#define	OPI_PC_39	-1

#define	OPI_PC_02	-1
#define	OPI_PC_04	-1
#define	OPI_PC_06	-1
#define	OPI_PC_08	GPIO_PA13
#define	OPI_PC_10	GPIO_PA14
#define	OPI_PC_12	GPIO_PD14
#define	OPI_PC_14	-1
#define	OPI_PC_16	GPIO_PC04
#define	OPI_PC_18	GPIO_PC07
#define	OPI_PC_20	-1
#define	OPI_PC_22	GPIO_PA02
#define	OPI_PC_24	GPIO_PC03
#define	OPI_PC_26	GPIO_PA21
#define	OPI_PC_28	GPIO_PA18
#define	OPI_PC_30	-1
#define	OPI_PC_32	GPIO_PG08
#define	OPI_PC_34	-1
#define	OPI_PC_36	GPIO_PG09
#define	OPI_PC_38	GPIO_PG06
#define	OPI_PC_40	GPIO_PG07

//map wpi gpio_num(index) to bp bpio_num(element)
int pinToGpio_OPI_PC [64] =
{
   OPI_PC_11, OPI_PC_12,        //0, 1
   OPI_PC_13, OPI_PC_15,        //2, 3
   OPI_PC_16, OPI_PC_18,        //4, 5
   OPI_PC_22, OPI_PC_07,        //6, 7
   OPI_PC_03, OPI_PC_05,        //8, 9
   OPI_PC_24, OPI_PC_26,        //10, 11
   OPI_PC_19, OPI_PC_21,        //12, 13
   OPI_PC_23, OPI_PC_08,        //14, 15
   OPI_PC_10,        -1,        //16, 17
          -1,        -1,        //18, 19
          -1, OPI_PC_29,        //20, 21
   OPI_PC_31, OPI_PC_33,        //22, 23
   OPI_PC_35, OPI_PC_37,        //24, 25
   OPI_PC_32, OPI_PC_36,        //26, 27
   OPI_PC_38, OPI_PC_40,        //28. 29
   OPI_PC_27, OPI_PC_28,        //30, 31
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
} ;

//map bcm gpio_num(index) to bp gpio_num(element)
int pinTobcm_OPI_PC [64] =
{
  OPI_PC_27, OPI_PC_28,  //0, 1
  OPI_PC_03, OPI_PC_05,  //2, 3
  OPI_PC_07, OPI_PC_29,  //4, 5
  OPI_PC_31, OPI_PC_26,  //6, 7
  OPI_PC_24, OPI_PC_21,  //8, 9
  OPI_PC_19, OPI_PC_23,  //10, 11
  OPI_PC_32, OPI_PC_33,  //12, 13
  OPI_PC_08, OPI_PC_10,  //14, 15
  OPI_PC_36, OPI_PC_11,  //16, 17
  OPI_PC_12, OPI_PC_35,	 //18, 19
  OPI_PC_38, OPI_PC_40,  //20, 21
  OPI_PC_15, OPI_PC_16,  //22, 23
  OPI_PC_18, OPI_PC_22,  //24, 25
  OPI_PC_37, OPI_PC_13,  //26, 27
  -1, -1,
  -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
} ;

//map phys_num(index) to bp gpio_num(element)
int physToGpio_OPI_PC [64] =
{
          -1,                //0
          -1,        -1,     //1, 2
   OPI_PC_03,        -1,     //3, 4
   OPI_PC_05,        -1,     //5, 6
   OPI_PC_07, OPI_PC_08,     //7, 8
          -1, OPI_PC_10,     //9, 10
   OPI_PC_11, OPI_PC_12,     //11, 12
   OPI_PC_13,        -1,     //13, 14
   OPI_PC_15, OPI_PC_16,     //15, 16
          -1, OPI_PC_18,     //17, 18
   OPI_PC_19,        -1,     //19, 20
   OPI_PC_21, OPI_PC_22,     //21, 22
   OPI_PC_23, OPI_PC_24,     //23, 24
          -1, OPI_PC_26,     //25, 26
   OPI_PC_27, OPI_PC_28,     //27, 28
   OPI_PC_29,        -1,     //29, 30
   OPI_PC_31, OPI_PC_32,     //31, 32      
   OPI_PC_33,        -1,     //33, 34
   OPI_PC_35, OPI_PC_36,     //35, 36
   OPI_PC_37, OPI_PC_38,     //37, 38
          -1, OPI_PC_40,     //39, 40
   -1,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
   -1,   -1, -1, -1, -1, -1, -1, -1 // 56-> 63
} ;

#define OPI_PC_I2C_DEV		"/dev/i2c-0"
#define OPI_PC_SPI_DEV		"/dev/spidev0.0"

#define OPI_PC_I2C_OFFSET  2
#define OPI_PC_SPI_OFFSET  3
#define OPI_PC_PWM_OFFSET  3
