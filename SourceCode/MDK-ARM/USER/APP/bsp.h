#ifndef BSP_H
#define BSP_H

//常用的宏定义
#define BIT(n) (1<<(n))

#define     BYTE0(n)            ((unsigned char)((unsigned short)(n)))
#define     BYTE1(n)            ((unsigned char)(((unsigned short)(n))>>8))
#define     BYTE2(n)            ((unsigned char)(((unsigned short)(((unsigned long)(n))>>8))>>8))
#define     BYTE3(n)            ((unsigned char)(((unsigned short)(((unsigned long)(n))>>16))>>8))

#define TRUE   1
#define FALSE  0
#define NULL   0


#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr))
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum))
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 	 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入


inline  void __LEG1_TXEN() {HAL_GPIO_WritePin(LEG1_TXE_GPIO_Port,LEG1_TXE_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(LEG1_RXE_GPIO_Port,LEG1_RXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG1_TXUEN() {HAL_GPIO_WritePin(LEG1_TXE_GPIO_Port,LEG1_TXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG1_RXEN() {HAL_GPIO_WritePin(LEG1_TXE_GPIO_Port,LEG1_TXE_Pin,GPIO_PIN_RESET);HAL_GPIO_WritePin(LEG1_RXE_GPIO_Port,LEG1_RXE_Pin,GPIO_PIN_SET);}

inline  void __LEG2_TXEN() {HAL_GPIO_WritePin(LEG2_TXE_GPIO_Port,LEG2_TXE_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(LEG2_RXE_GPIO_Port,LEG2_RXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG2_TXUEN() {HAL_GPIO_WritePin(LEG2_TXE_GPIO_Port,LEG2_TXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG2_RXEN() {HAL_GPIO_WritePin(LEG2_TXE_GPIO_Port,LEG2_TXE_Pin,GPIO_PIN_RESET);HAL_GPIO_WritePin(LEG2_RXE_GPIO_Port,LEG2_RXE_Pin,GPIO_PIN_SET);}

inline  void __LEG3_TXEN() {HAL_GPIO_WritePin(LEG3_TXE_GPIO_Port,LEG3_TXE_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(LEG3_RXE_GPIO_Port,LEG3_RXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG3_TXUEN() {HAL_GPIO_WritePin(LEG3_TXE_GPIO_Port,LEG3_TXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG3_RXEN() {HAL_GPIO_WritePin(LEG3_TXE_GPIO_Port,LEG3_TXE_Pin,GPIO_PIN_RESET);HAL_GPIO_WritePin(LEG3_RXE_GPIO_Port,LEG3_RXE_Pin,GPIO_PIN_SET);}

inline  void __LEG4_TXEN() {HAL_GPIO_WritePin(LEG4_TXE_GPIO_Port,LEG4_TXE_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(LEG4_RXE_GPIO_Port,LEG4_RXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG4_TXUEN() {HAL_GPIO_WritePin(LEG4_TXE_GPIO_Port,LEG4_TXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG4_RXEN() {HAL_GPIO_WritePin(LEG4_TXE_GPIO_Port,LEG4_TXE_Pin,GPIO_PIN_RESET);HAL_GPIO_WritePin(LEG4_RXE_GPIO_Port,LEG4_RXE_Pin,GPIO_PIN_SET);}

inline  void __LEG5_TXEN() {HAL_GPIO_WritePin(LEG5_TXE_GPIO_Port,LEG5_TXE_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(LEG5_RXE_GPIO_Port,LEG5_RXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG5_TXUEN() {HAL_GPIO_WritePin(LEG5_TXE_GPIO_Port,LEG5_TXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG5_RXEN() {HAL_GPIO_WritePin(LEG5_TXE_GPIO_Port,LEG5_TXE_Pin,GPIO_PIN_RESET);HAL_GPIO_WritePin(LEG5_RXE_GPIO_Port,LEG5_RXE_Pin,GPIO_PIN_SET);}

inline  void __LEG6_TXEN() {HAL_GPIO_WritePin(LEG6_TXE_GPIO_Port,LEG6_TXE_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(ARM_RXE_GPIO_Port,ARM_RXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG6_TXUEN() {HAL_GPIO_WritePin(LEG6_TXE_GPIO_Port,LEG6_TXE_Pin,GPIO_PIN_RESET);}
inline  void __LEG6_RXEN() {HAL_GPIO_WritePin(LEG6_TXE_GPIO_Port,LEG6_TXE_Pin,GPIO_PIN_RESET);HAL_GPIO_WritePin(ARM_RXE_GPIO_Port,ARM_RXE_Pin,GPIO_PIN_SET);}

inline  void __ARM_TXEN() {HAL_GPIO_WritePin(ARM_TXE_GPIO_Port,ARM_TXE_Pin,GPIO_PIN_SET);HAL_GPIO_WritePin(ARM_RXE_GPIO_Port,ARM_RXE_Pin,GPIO_PIN_RESET);}
inline  void __ARM_TXUEN() {HAL_GPIO_WritePin(ARM_TXE_GPIO_Port,ARM_TXE_Pin,GPIO_PIN_RESET);}
inline  void __ARM_RXEN() {HAL_GPIO_WritePin(ARM_TXE_GPIO_Port,ARM_TXE_Pin,GPIO_PIN_RESET);HAL_GPIO_WritePin(ARM_RXE_GPIO_Port,ARM_RXE_Pin,GPIO_PIN_SET);}


#endif
