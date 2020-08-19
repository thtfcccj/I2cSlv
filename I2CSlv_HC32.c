/*******************************************************************************

               I2C从机通用驱动程序-在HC32上的实现
经检查，此I2C(已使用到)的STATE码，与LPC32相同,模块硬件结构与基本相同
********************************************************************************/

#include "I2cSlv.h"
#include "HC32.h"
#include <string.h>

//------------------------------I2c设备初始化函数----------------------
//此函数将始化设备结构，并将挂接的I2c硬件初始化
//注：此函数不负责配置IO端口以及其中断入口,时钟，模块开启等
void I2cSlv_Init(struct _I2cSlv *pI2cSlv,  //未初始化的设备指针
                 void *pHw,                 //挂接的I2c硬件
                 unsigned char Id,         //此设备ID号，可用于快速处理
                 unsigned char SlvAdr,     //从机地址,0-127
                 struct _I2cSlvCmd *pCmd)   //挂接的命令字缓冲区   
{
  //初始化结构
  memset(pI2cSlv,0,sizeof(struct _I2cSlv));
  pI2cSlv->pI2cHw = pHw; //硬件挂接
  pI2cSlv->Id = Id;    
  pI2cSlv->SlvAdr = SlvAdr;     
  pI2cSlv->pCmd = pCmd;  
  pI2cSlv->eState = eI2cSlvIdie;  

  //外1: Step1: SCL、SDA映射到需要的管脚；并配置 SCL、SDA管脚为开漏模式
  //外2: Step2：设置 PERI_CLKEN.I2Cx为 1，使能 I2Cx模
  //外3: Step3：向 PERI_RESET.I2Cx依次写入 0、1，复位 ，复位 I2Cx
 
  M0P_I2C_TypeDef *pI2cHw = (M0P_I2C_TypeDef *)pHw;
  //1. Step4：设置 I2Cx_CR.ens为 1，使能 ，使能I2C
  pI2cHw->CR_f.ENS = 1;
  //2. Step5：配置 I2Cx_ADDR为当前地址
  if(pCmd->Flag & I2C_SLVCMD_EN_GC) //广播地址使能
    pI2cHw->ADDR = 0x01 | (SlvAdr << 1);
  else pI2cHw->ADDR = (SlvAdr << 1);
}

//---------------------------更新从机地址-----------------------------
void I2cSlv_UpdateSlvAdr(struct _I2cSlv *pI2cSlv,
                         unsigned char SlvAdr)     //从机地址,1-127
{
  I2cSlv_Reset(pI2cSlv);
  pI2cSlv->SlvAdr = SlvAdr;
  M0P_I2C_TypeDef *pI2cHw = (M0P_I2C_TypeDef *)pI2cSlv->pI2cHw;
  if(pI2cSlv->pCmd->Flag & I2C_SLVCMD_EN_GC) //广播地址使能
    pI2cHw->ADDR = 0x01 | (SlvAdr << 1);
  else pI2cHw->ADDR = (SlvAdr << 1); 
  I2cSlv_ReStart(pI2cSlv);
}

//-----------------------------I2c从机启动函数-------------------------
//置为从机准备接收数据状态
signed char I2cSlv_ReStart(struct _I2cSlv *pI2cSlv) //设备指针        
{
  enum _eI2cSlvState eState = pI2cSlv->eState;
  //I2c进行过程中禁止访问
  if(eState != eI2cSlvIdie)  return -1;
  pI2cSlv->eState = eI2cSlvRdy;//准备接收数据
  
  M0P_I2C_TypeDef *pI2cHw = (M0P_I2C_TypeDef *)pI2cSlv->pI2cHw;
  //Step6：设置 I2Cx_CR.aa为 1，使能应答标志。
  pI2cHw->CR_f.AA = 1;//使能应答位 
  return 0;
}

//-----------------------------I2c强制停止函数-------------------------
//停止并强制I2c复位
void I2cSlv_Reset(struct _I2cSlv *pI2cSlv)
{
  M0P_I2C_TypeDef *pI2cHw = (M0P_I2C_TypeDef *)pI2cSlv->pI2cHw;
  pI2cHw->CR_f.AA = 0;//禁止I2c应答
  pI2cSlv->eState = eI2cSlvIdie;
}

//-----------------------------I2c中断处理程序-------------------------
//将此函数放入中断处理程序中
 void I2cSlv_IRQ(struct _I2cSlv *pI2cSlv)
{ 
  struct _I2cSlvCmd *pCmd = pI2cSlv->pCmd;
  M0P_I2C_TypeDef *pI2cHw = (M0P_I2C_TypeDef *)pI2cSlv->pI2cHw;
  unsigned long StateHw = pI2cHw->STAT;//读硬件状态位
  enum _eI2cSlvState eState = pI2cSlv->eState;//读状态机
  
  unsigned char Index;
  
  switch(StateHw){
  //======================接收到器件地址+写标志,从机为读================
  case  0x78://广播地址被接收到,主机仲载丢失，但不影响从机
  case  0x70://广播地址被接收到!
  case  0x68://主机仲载丢失，但不影响从机
  case  0x60://从机的地址从机已应答
    if(eState == eI2cSlvRdy){   //从机准备好时
      //这里初始化相关变量
      pI2cSlv->Index = 0;
      if((StateHw == 0x78) || (StateHw == 0x70))
        pCmd->Flag |= I2C_SLVCMD_GC_FLAG; //广播地址标志
      else
        pCmd->Flag &= ~I2C_SLVCMD_GC_FLAG; //去掉广播地址标志      
        
      if(pCmd->CmdSize)	//含命令字时,准备接收命令
        eState = eI2SlvCmdRcv;
      else//无命令时直接接收数据
        eState = eI2cSlvRd;
      pI2cHw->CR_f.AA = 1; //继续应答
    }
    else eState = eI2cSlvErr;   //状态机错误
    break;
  //===================接收到主机写数据,从机读数据======================
  case 0x80://从机应答返回
  case 0x90://从机广播地址应答返回
   Index = pI2cSlv->Index;
   if(eState == eI2SlvCmdRcv){//收命令
     *(pCmd->pCmd + Index) = pI2cHw->DATA;
     Index++;
     if(Index >= pCmd->CmdSize){
       Index = 0;
       eState = eI2cSlvRd;//预切换到读数据状态
     }
     pI2cHw->CR_f.AA = 1; //继续应答 
   }
   else if(eState == eI2cSlvRd){//收数据
     if(Index < pCmd->DataSize){
       *(pCmd->pData + Index) = pI2cHw->DATA;
       Index++;
       pI2cHw->CR_f.AA = 1; //继续应答 
     }
     else //超出接收缓冲区丢后面数据
       pI2cHw->CR_f.AA = 0; //停止应答  
   }
   else eState = eI2cSlvErr;   //状态机错误
   pI2cSlv->Index = Index;
   break;
  //===================接收数据状态结束标志======================
  case 0xA0:
    if(eState == eI2cSlvRd){//收数据中止
      //没有收到数据，但收到命令了，此状态表示为主机写从机数据状态
      if(!pI2cSlv->Index){
        eState = eI2cSlvWr;          //强制在写状态
        pI2cSlv->eState = eState;//回调函数使用
      }
      //else //接收到了数据，表示在接收数据状态
      
      //回调函数处理:准备需发送的数据或接收数据处理
      if(I2cSlv_cbFun(pI2cSlv,pCmd->pData))	
       pI2cHw->CR_f.AA = 0;//停止I2C处理
      else{ //继续接收
        if(eState == eI2cSlvRd) eState = eI2cSlvRdy;//读数据状态处理完成，重新准备接收
        pI2cHw->CR_f.AA = 1; //继续应答
      }
    }
    else
      eState = eI2cSlvErr;   //状态机错误
    break;
  //======================接收到器件地址+读标志,从机为写================
  case  0xB0://地址收到并已应答
  case  0xA8://地址收到并已应答,主机仲载丢失，但不影响从机
    //无命令字或有命令字且已在从机状态时才允许读
    if((!pCmd->CmdSize) || (eState == eI2cSlvWr)){
      pI2cSlv->Index = 0;//重新复位
      eState = eI2cSlvWr;	//强制读状态
      //继续向主机返回数据
    }
    else{
      eState = eI2cSlvErr;   //状态机错误
      break;
    }
  //======================从机向主机写数据过程中收到应答================
  case  0xB8://数据已发出且收到了应答
  case  0xC0://数据已发出但未收到应答,表示发送最后一个数据了  
   if(eState == eI2cSlvWr){//发送数据
     Index = pI2cSlv->Index;
     if(Index < pCmd->DataSize){
       pI2cHw->DATA = *(pCmd->pData + Index);
       Index++;
       pI2cHw->CR_f.AA = 1; //仍置为接收状态,继续应答
     }
     else pI2cHw->DATA = 0;//超过缓冲区大小了
     pI2cSlv->Index = Index; 
     if(StateHw == 0xC0) //发送已完成，强制重新置为准备状态
         eState = eI2cSlvRdy;
        break;    
   }
   else
      eState = eI2cSlvErr;   //状态机错误
  //=====================其它为错误状态================
  //case  0xC8://最后一个数已发出了，但仍收到了应答,状态错误
  //case 0x88://从机应答不返回后，仍收到了数据,,状态错误
  //case 0x98://从机广播地址不应答返回后，仍收到了数据,状态错误
  default:		//其它状态均认为是器件出错
    eState = eI2cSlvErr;   //状态机错误
    break;
  }
  
  //若I2C状态出错,则强行结束I2c总线
  if(eState == eI2cSlvErr){
    pI2cHw->CR_f.AA = 0; //停止应答 
    //重启自动恢复
    eState = eI2cSlvRdy;
    pI2cHw->CR_f.AA = 1; //开启应答 
  }
  pI2cSlv->eState = eState;
  
  pI2cHw->CR_f.SI = 0;//清中断 (写入 0，I2C进行下一步操作)
}



