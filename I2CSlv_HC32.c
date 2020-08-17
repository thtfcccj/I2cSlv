/*******************************************************************************

               I2C�ӻ�ͨ����������-��HC32�ϵ�ʵ��
����飬��I2C(��ʹ�õ�)��STATE�룬��LPC32��ͬ,ģ��Ӳ���ṹ�������ͬ
********************************************************************************/

#include "I2cSlv.h"
#include "HC32.h"
#include <string.h>

//------------------------------I2c�豸��ʼ������----------------------
//�˺�����ʼ���豸�ṹ�������ҽӵ�I2cӲ����ʼ��
//ע���˺�������������IO�˿��Լ����ж����,ʱ�ӣ�ģ�鿪����
void I2cSlv_Init(struct _I2cSlv *pI2cSlv,  //δ��ʼ�����豸ָ��
                 void *pHw,                 //�ҽӵ�I2cӲ��
                 unsigned char Id,         //���豸ID�ţ������ڿ��ٴ���
                 unsigned char SlvAdr,     //�ӻ���ַ,0-127
                 struct _I2cSlvCmd *pCmd)   //�ҽӵ������ֻ�����   
{
  //��ʼ���ṹ
  memset(pI2cSlv,0,sizeof(struct _I2cSlv));
  pI2cSlv->pI2cHw = pHw; //Ӳ���ҽ�
  pI2cSlv->Id = Id;    
  pI2cSlv->SlvAdr = SlvAdr;     
  pI2cSlv->pCmd = pCmd;  
  pI2cSlv->eState = eI2cSlvIdie;  

  //��1: Step1: SCL��SDAӳ�䵽��Ҫ�Ĺܽţ������� SCL��SDA�ܽ�Ϊ��©ģʽ
  //��2: Step2������ PERI_CLKEN.I2CxΪ 1��ʹ�� I2Cxģ
  //��3: Step3���� PERI_RESET.I2Cx����д�� 0��1����λ ����λ I2Cx
 
  M0P_I2C_TypeDef *pI2cHw = (M0P_I2C_TypeDef *)pHw;
  //1. Step4������ I2Cx_CR.ensΪ 1��ʹ�� ��ʹ��I2C
  pI2cHw->CR_f.ENS = 1;
  //2. Step5������ I2Cx_ADDRΪ��ǰ��ַ
  if(pCmd->Flag & I2C_SLVCMD_GC_FLAG) //�㲥��ַʹ��
    pI2cHw->ADDR = 0x01 | (SlvAdr << 1);
  else pI2cHw->ADDR = (SlvAdr << 1);
}

//-----------------------------I2c�ӻ���������-------------------------
//��Ϊ�ӻ�׼����������״̬
signed char I2cSlv_ReStart(struct _I2cSlv *pI2cSlv) //�豸ָ��        
{
  enum _eI2cSlvState eState = pI2cSlv->eState;
  //I2c���й����н�ֹ����
  if(eState != eI2cSlvIdie)  return -1;
  pI2cSlv->eState = eI2cSlvRdy;//׼����������
  
  M0P_I2C_TypeDef *pI2cHw = (M0P_I2C_TypeDef *)pI2cSlv->pI2cHw;
  //Step6������ I2Cx_CR.aaΪ 1��ʹ��Ӧ���־��
  pI2cHw->CR_f.AA = 1;//ʹ��Ӧ��λ 
  return 0;
}

//-----------------------------I2cǿ��ֹͣ����-------------------------
//ֹͣ��ǿ��I2c��λ
void I2cSlv_Reset(struct _I2cSlv *pI2cSlv)
{
  M0P_I2C_TypeDef *pI2cHw = (M0P_I2C_TypeDef *)pI2cSlv->pI2cHw;
  pI2cHw->CR_f.AA = 0;//��ֹI2cӦ��
  pI2cSlv->eState = eI2cSlvIdie;
}

//-----------------------------I2c�жϴ�������-------------------------
//���˺��������жϴ���������
 void I2cSlv_IRQ(struct _I2cSlv *pI2cSlv)
{ 
  struct _I2cSlvCmd *pCmd = pI2cSlv->pCmd;
  M0P_I2C_TypeDef *pI2cHw = (M0P_I2C_TypeDef *)pI2cSlv->pI2cHw;
  unsigned long StateHw = pI2cHw->STAT;//��Ӳ��״̬λ
  enum _eI2cSlvState eState = pI2cSlv->eState;//��״̬��
  
  unsigned char Index;
  
  switch(StateHw){
  //======================���յ�������ַ+д��־,�ӻ�Ϊ��================
  case  0x78://�㲥��ַ�����յ�,�������ض�ʧ������Ӱ��ӻ�
  case  0x70://�㲥��ַ�����յ�!
  case  0x68://�������ض�ʧ������Ӱ��ӻ�
  case  0x60://�ӻ��ĵ�ַ�ӻ���Ӧ��
    if(eState == eI2cSlvRdy){   //�ӻ�׼����ʱ
      //�����ʼ����ر���
      pI2cSlv->Index = 0;
      if((StateHw == 0x78) || (StateHw == 0x70))
        pCmd->Flag |= I2C_SLVCMD_GC_FLAG; //�㲥��ַ��־
      else
        pCmd->Flag &= ~I2C_SLVCMD_GC_FLAG; //ȥ���㲥��ַ��־      
        
      if(pCmd->CmdSize)	//��������ʱ,׼����������
        eState = eI2SlvCmdRcv;
      else//������ʱֱ�ӽ�������
        eState = eI2cSlvRd;
      pI2cHw->CR_f.AA = 1; //����Ӧ��
    }
    else eState = eI2cSlvErr;   //״̬������
    break;
  //===================���յ�����д����,�ӻ�������======================
  case 0x80://�ӻ�Ӧ�𷵻�
  case 0x90://�ӻ��㲥��ַӦ�𷵻�
   Index = pI2cSlv->Index;
   if(eState == eI2SlvCmdRcv){//������
     *(pCmd->pCmd + Index) = pI2cHw->DATA;
     Index++;
     if(Index >= pCmd->CmdSize){
       Index = 0;
       eState = eI2cSlvRd;//Ԥ�л���������״̬
     }
     pI2cHw->CR_f.AA = 1; //����Ӧ�� 
   }
   else if(eState == eI2cSlvRd){//������
     if(Index < pCmd->DataSize){
       *(pCmd->pData + Index) = pI2cHw->DATA;
       Index++;
       pI2cHw->CR_f.AA = 1; //����Ӧ�� 
     }
     else //�������ջ���������������
       pI2cHw->CR_f.AA = 0; //ֹͣӦ��  
   }
   else eState = eI2cSlvErr;   //״̬������
   pI2cSlv->Index = Index;
   break;
  //===================��������״̬������־======================
  case 0xA0:
    if(eState == eI2cSlvRd){//��������ֹ
      //û���յ����ݣ����յ������ˣ���״̬��ʾΪ����д�ӻ�����״̬
      if(!pI2cSlv->Index){
        eState = eI2cSlvWr;          //ǿ����д״̬
        pI2cSlv->eState = eState;//�ص�����ʹ��
      }
      //else //���յ������ݣ���ʾ�ڽ�������״̬
      
      //�ص���������:׼���跢�͵����ݻ�������ݴ���
      if(I2cSlv_cbFun(pI2cSlv,pCmd->pData))	
       pI2cHw->CR_f.AA = 0;//ֹͣI2C����
      else{ //��������
        if(eState == eI2cSlvRd) eState = eI2cSlvRdy;//������״̬������ɣ�����׼������
        pI2cHw->CR_f.AA = 1; //����Ӧ��
      }
    }
    else
      eState = eI2cSlvErr;   //״̬������
    break;
  //======================���յ�������ַ+����־,�ӻ�Ϊд================
  case  0xB0://��ַ�յ�����Ӧ��
  case  0xA8://��ַ�յ�����Ӧ��,�������ض�ʧ������Ӱ��ӻ�
    //�������ֻ��������������ڴӻ�״̬ʱ��������
    if((!pCmd->CmdSize) || (eState == eI2cSlvWr)){
      pI2cSlv->Index = 0;//���¸�λ
      eState = eI2cSlvWr;	//ǿ�ƶ�״̬
      //������������������
    }
    else{
      eState = eI2cSlvErr;   //״̬������
      break;
    }
  //======================�ӻ�������д���ݹ������յ�Ӧ��================
  case  0xB8://�����ѷ������յ���Ӧ��
  case  0xC0://�����ѷ�����δ�յ�Ӧ��,��ʾ�������һ��������  
   if(eState == eI2cSlvWr){//��������
     Index = pI2cSlv->Index;
     if(Index < pCmd->DataSize){
       pI2cHw->DATA = *(pCmd->pData + Index);
       Index++;
       pI2cHw->CR_f.AA = 1; //����Ϊ����״̬,����Ӧ��
     }
     else pI2cHw->DATA = 0;//������������С��
     pI2cSlv->Index = Index; 
     if(StateHw == 0xC0) //��������ɣ�ǿ��������Ϊ׼��״̬
         eState = eI2cSlvRdy;
        break;    
   }
   else
      eState = eI2cSlvErr;   //״̬������
  //=====================����Ϊ����״̬================
  //case  0xC8://���һ�����ѷ����ˣ������յ���Ӧ��,״̬����
  //case 0x88://�ӻ�Ӧ�𲻷��غ����յ�������,,״̬����
  //case 0x98://�ӻ��㲥��ַ��Ӧ�𷵻غ����յ�������,״̬����
  default:		//����״̬����Ϊ����������
    eState = eI2cSlvErr;   //״̬������
    break;
  }
  
  //��I2C״̬����,��ǿ�н���I2c����
  if(eState == eI2cSlvErr){
    pI2cHw->CR_f.AA = 0; //ֹͣӦ�� 
    //�����Զ��ָ�
    eState = eI2cSlvRdy;
    pI2cHw->CR_f.AA = 1; //����Ӧ�� 
  }
  pI2cSlv->eState = eState;
  
  pI2cHw->CR_f.SI = 0;//���ж� (д�� 0��I2C������һ������)
}


