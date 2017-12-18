#define	OPI_WIN_01 -1
#define	OPI_WIN_03 GPIO_PH03
#define	OPI_WIN_05 GPIO_PH02
#define	OPI_WIN_07 GPIO_PL10
#define	OPI_WIN_09 -1
#define	OPI_WIN_11 GPIO_PH05
#define	OPI_WIN_13 GPIO_PH04
#define	OPI_WIN_15 GPIO_PH07
#define	OPI_WIN_17 -1
#define	OPI_WIN_19 GPIO_PD02
#define	OPI_WIN_21 GPIO_PD03
#define	OPI_WIN_23 GPIO_PD01
#define	OPI_WIN_25 -1
#define	OPI_WIN_27 GPIO_PE15
#define	OPI_WIN_29 GPIO_PB04
#define	OPI_WIN_31 GPIO_PB05
#define	OPI_WIN_33 GPIO_PB06
#define	OPI_WIN_35 GPIO_PB07
#define	OPI_WIN_37 GPIO_PD05
#define	OPI_WIN_39 -1

#define	OPI_WIN_02 -1
#define	OPI_WIN_04 -1
#define	OPI_WIN_06 -1
#define	OPI_WIN_08 GPIO_PL02
#define	OPI_WIN_10 GPIO_PL03
#define	OPI_WIN_12 GPIO_PD04
#define	OPI_WIN_14 -1
#define	OPI_WIN_16 GPIO_PL09
#define	OPI_WIN_18 GPIO_PC04
#define	OPI_WIN_20 -1
#define	OPI_WIN_22 GPIO_PH06
#define	OPI_WIN_24 GPIO_PD00
#define	OPI_WIN_26 GPIO_PD06
#define	OPI_WIN_28 GPIO_PE14
#define	OPI_WIN_30 -1
#define	OPI_WIN_32 GPIO_PB02
#define	OPI_WIN_34 -1
#define	OPI_WIN_36 GPIO_PB03
#define	OPI_WIN_38 GPIO_PB00
#define	OPI_WIN_40 GPIO_PB01

//map wpi gpio_num(index) to bp bpio_num(element)
int pinToGpio_OPI_WIN [64] =
{
   OPI_WIN_11, OPI_WIN_12,        //0, 1
   OPI_WIN_13, OPI_WIN_15,        //2, 3
   OPI_WIN_16, OPI_WIN_18,        //4, 5
   OPI_WIN_22, OPI_WIN_07,        //6, 7
   OPI_WIN_03, OPI_WIN_05,        //8, 9
   OPI_WIN_24, OPI_WIN_26,        //10, 11
   OPI_WIN_19, OPI_WIN_21,        //12, 13
   OPI_WIN_23, OPI_WIN_08,        //14, 15
   OPI_WIN_10,        -1,        //16, 17
          -1,        -1,        //18, 19
          -1, OPI_WIN_29,        //20, 21
   OPI_WIN_31, OPI_WIN_33,        //22, 23
   OPI_WIN_35, OPI_WIN_37,        //24, 25
   OPI_WIN_32, OPI_WIN_36,        //26, 27
   OPI_WIN_38, OPI_WIN_40,        //28. 29
   OPI_WIN_27, OPI_WIN_28,        //30, 31
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
} ;

//map bcm gpio_num(index) to bp gpio_num(element)
int pinTobcm_OPI_WIN [64] =
{
  OPI_WIN_27, OPI_WIN_28,  //0, 1
  OPI_WIN_03, OPI_WIN_05,  //2, 3
  OPI_WIN_07, OPI_WIN_29,  //4, 5
  OPI_WIN_31, OPI_WIN_26,  //6, 7
  OPI_WIN_24, OPI_WIN_21,  //8, 9
  OPI_WIN_19, OPI_WIN_23,  //10, 11
  OPI_WIN_32, OPI_WIN_33,  //12, 13
  OPI_WIN_08, OPI_WIN_10,  //14, 15
  OPI_WIN_36, OPI_WIN_11,  //16, 17
  OPI_WIN_12, OPI_WIN_35,	 //18, 19
  OPI_WIN_38, OPI_WIN_40,  //20, 21
  OPI_WIN_15, OPI_WIN_16,  //22, 23
  OPI_WIN_18, OPI_WIN_22,  //24, 25
  OPI_WIN_37, OPI_WIN_13,  //26, 27
  -1, -1,
  -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // ... 63
} ;

//map phys_num(index) to bp gpio_num(element)
int physToGpio_OPI_WIN [64] =
{
          -1,                //0
          -1,        -1,     //1, 2
   OPI_WIN_03,        -1,     //3, 4
   OPI_WIN_05,        -1,     //5, 6
   OPI_WIN_07, OPI_WIN_08,     //7, 8
          -1, OPI_WIN_10,     //9, 10
   OPI_WIN_11, OPI_WIN_12,     //11, 12
   OPI_WIN_13,        -1,     //13, 14
   OPI_WIN_15, OPI_WIN_16,     //15, 16
          -1, OPI_WIN_18,     //17, 18
   OPI_WIN_19,        -1,     //19, 20
   OPI_WIN_21, OPI_WIN_22,     //21, 22
   OPI_WIN_23, OPI_WIN_24,     //23, 24
          -1, OPI_WIN_26,     //25, 26
   OPI_WIN_27, OPI_WIN_28,     //27, 28
   OPI_WIN_29,        -1,     //29, 30
   OPI_WIN_31, OPI_WIN_32,     //31, 32      
   OPI_WIN_33,        -1,     //33, 34
   OPI_WIN_35, OPI_WIN_36,     //35, 36
   OPI_WIN_37, OPI_WIN_38,     //37, 38
          -1, OPI_WIN_40,     //39, 40
   -1,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
   -1,   -1, -1, -1, -1, -1, -1, -1 // 56-> 63
} ;

#define OPI_WIN_I2C_DEV		"/dev/i2c-1"
#define OPI_WIN_SPI_DEV		"/dev/spidev1.0"

#define OPI_WIN_I2C_OFFSET  2
#define OPI_WIN_SPI_OFFSET  4
#define OPI_WIN_PWM_OFFSET  -1

