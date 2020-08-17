/*******************************************************************************

                      I2C从机通用驱动程序接口

********************************************************************************/
#ifndef __I2C_SLV_H
#define __I2C_SLV_H


/*******************************************************************************
                        相关结构
********************************************************************************/
//I2c命令字定义
struct _I2cSlvCmd{
  unsigned char CmdSize;  //命令字大小,可以为0
  unsigned char DataSize; //数据字缓冲区大小,>= 1;
  unsigned char Flag;     //相关标志字，见定义

  unsigned char *pCmd;    //命令字缓冲区，大小为命令字大小,    
  unsigned char *pData;   //数据字缓冲区
};

//其中，相关标志字定义为：  
//是否允许接收广播地址,初始化函数写入
#define   I2C_SLVCMD_EN_GC  	0x80
//接收到广播地址标志,I2C数据处理过程中使用
#define   I2C_SLVCMD_GC_FLAG  0x08

//I2c总线工作状态机定义
enum _eI2cSlvState
{
  //I2c停止期间:
  eI2cSlvIdie    =0,     //总线空闲
  //I2c运行期间:
  eI2cSlvRdy     =1,     //I2c准备状态
  eI2SlvCmdRcv   =2,     //命令字接收状态,收定命令字大小个数后自动转到读写状态
  eI2cSlvWr      =3,     //写数据状态,即主机向从机请求数据
  eI2cSlvRd      =4,     //读数据状态,即主机向从机发送数据
  //I2c结束期间:
  eI2cSlvErr =5          //I2c状态机错误
};

//I2c设备定义
struct _I2cSlv{
  void *pI2cHw;                            //挂接的I2c硬件设备指针
  struct _I2cSlvCmd  *pCmd;               //正在通读的I2c从机设备或命令
  unsigned char Id;                      //此设备ID号
  volatile enum _eI2cSlvState eState;  //状态机,外部只读
  unsigned char Index;                   //用于标识正在处理那一位
};

/*******************************************************************************
                            相关函数接口
*******************************************************************************/

//------------------------------I2c设备初始化函数----------------------
//此函数将始化设备结构，并将挂接的I2c硬件初始化
//注：此函数不负责配置IO端口以及其中断入口
void I2cSlv_Init(struct _I2cSlv *pI2cSlv,  //未初始化的设备指针
                 void *pI2cHw,              //挂接的I2c硬件
                 unsigned char Id,         //此设备ID号，可用于快速处理
                 unsigned char SlvAdr,     //从机地址,1-127
                 struct _I2cSlvCmd *pCmd);  //挂接的命令字缓冲区

//-----------------------------I2c从机启动函数-------------------------
//置为从机准备接收数据状态
int I2cSlv_ReStart(struct _I2cSlv *pI2cSlv); //设备指针

//-----------------------------I2c强制复位函数-------------------------
//停止并强制I2c复位
void I2cSlv_Reset(struct _I2cSlv *pI2cSlv); 

//-----------------------------I2c中断处理程序-------------------------
//将此函数放入中断处理程序中
void I2cSlv_IRQ(struct _I2cSlv *pI2cSlv);

//--------------------------I2c得到状态函数------------------------------
//enum eI2cState_t I2cSlv_eGetSatate(const struct _I2cSlv *pI2cSlv);
#define I2cSlv_eGetSatate(pI2cSlv) ((pI2cSlv)->eState)

//--------------------------------I2c从机回调函数--------------------------
//当接收到读命令或接收写命令结束时调用此函数，
//读命令时(状态机为读)，用户应在此函数中准备需发送的数据
//写命令时(状态机为写),用户应处理接收到的有效数据
//返回1时强制停止I2C从机，返回0时内部自动置为从机接收状态
int I2cSlv_cbFun(const struct _I2cSlv *pI2cSlv,
                 //主机读取数据时的写缓冲区,大小由主机决定
                 unsigned char *pData);	

#endif
