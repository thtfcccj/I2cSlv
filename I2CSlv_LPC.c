/*******************************************************************************

               I2C�ӻ�ͨ����������-��LPC�ϵ�ʵ��
��LPC2103��������������Review��δ���������
********************************************************************************/

#include "I2cSlv.h"
//#include "LPC2103.h"
//#include "LPC2103_bit.h"
#include "string.h"

//------------------------------I2c�豸��ʼ������----------------------
//�˺�����ʼ���豸�ṹ�������ҽӵ�I2cӲ����ʼ��
//ע���˺�������������IO�˿��Լ����ж����
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

  //��ǿ��ֹͣI2c����
  LPCS_I2C *pI2cHw = (LPCS_I2C *)pHw;
  pI2cHw->CONCLR = LPC_I2C_EN | LPC_I2C_SI | LPC_I2C_STA | LPC_I2C_AA;
  
  //���õ�ַ��
  pI2cHw->ADR = SlvAdr << 1;
  if(pCmd->Flag & I2C_SLVCMD_EN_GC) pI2cHw->ADR |= 0x01;
}

//---------------------------���´ӻ���ַ-----------------------------
void I2cSlv_UpdateSlvAdr(struct _I2cSlv *pI2cSlv,
                         unsigned char SlvAdr)     //�ӻ���ַ,1-127
{
  I2cSlv_Reset(pI2cSlv);
  pI2cSlv->SlvAdr = SlvAdr;
  LPCS_I2C *pI2cHw = (LPCS_I2C *)pI2cSlv->pI2cHw;
  if(pI2cSlv->pCmd->Flag & I2C_SLVCMD_EN_GC) //�㲥��ַʹ��
    pI2cHw->ADR = 0x01 | (SlvAdr << 1);
  else pI2cHw->ADR = (SlvAdr << 1); 
  I2cSlv_ReStart(pI2cSlv);
}

//-----------------------------I2c�ӻ���������-------------------------
//��Ϊ�ӻ�׼����������״̬
signed char I2cSlv_ReStart(struct _I2cSlv *pI2cSlv) //�豸ָ��        
{
  enum _eI2cSlvState eState = pI2cSlv->eState;
  //I2c���й����н�ֹ����
  if(eState != eI2cSlvIdie)  return -1;
  
  LPCS_I2C *pI2cHw = (LPCS_I2C *)pI2cSlv->pI2cHw;
  
  pI2cHw->CONCLR = LPC_I2C_SI | LPC_I2C_STA | LPC_I2C_AA;  //��ֹͣ
  
  pI2cSlv->eState = eI2cSlvRdy;//׼����������
  //����I2c�ӻ���ʼ����:
  pI2cHw->CONSET = LPC_I2C_EN | LPC_I2C_AA;//ʹ��I2c��Ӧ��λ 
  return 0;
}

//-----------------------------I2cǿ��ֹͣ����-------------------------
//ֹͣ��ǿ��I2c��λ
void I2cSlv_Reset(struct _I2cSlv *pI2cSlv)
{
  LPCS_I2C *pI2cHw = (LPCS_I2C *)pI2cSlv->pI2cHw;
  pI2cHw->CONCLR = LPC_I2C_SI | LPC_I2C_STA | LPC_I2C_AA;
  pI2cSlv->eState = eI2cSlvIdie;
}

//-----------------------------I2c�жϴ������-------------------------
//���˺��������жϴ��������
 void I2cSlv_IRQ(struct _I2cSlv *pI2cSlv)
{ 
  struct _I2cSlvCmd *pCmd = pI2cSlv->pCmd;
  LPCS_I2C *pI2cHw = (LPCS_I2C *)pI2cSlv->pI2cHw;
  unsigned long StateHw = pI2cHw->STAT;//��Ӳ��״̬λ
  enum _eI2cSlvState eState = pI2cSlv->eState;//��״̬��
  
  unsigned char Index;
  
  switch(StateHw){
  //======================���յ�������ַ+д��־,�ӻ�Ϊ��================
  case  0x78://�㲥��ַ�����յ�,�������ض�ʧ������Ӱ��ӻ�
  case  0x70://�㲥��ַ�����յ�!
  case  0x68://�������ض�ʧ������Ӱ��ӻ�
  case  0x60://�ӻ��ĵ�ַ+д����,�ӻ���Ӧ��
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
      pI2cHw->CONSET = LPC_I2C_AA;
    }
    else eState = eI2cSlvErr;   //״̬������
    break;
  //===================���յ�����д����,�ӻ�������======================
  case 0x80://�ӻ�Ӧ�𷵻�
  case 0x90://�ӻ��㲥��ַӦ�𷵻�
   Index = pI2cSlv->Index;
   if(eState == eI2SlvCmdRcv){//������
     *(pCmd->pCmd + Index) = pI2cHw->DAT;
     Index++;
     if(Index >= pCmd->CmdSize){
       Index = 0;
       eState = eI2cSlvRd;//Ԥ�л���������״̬
     }
     pI2cHw->CONSET = LPC_I2C_AA; 
   }
   else if(eState == eI2cSlvRd){//������
     if(Index < pCmd->DataSize){
       *(pCmd->pData + Index) = pI2cHw->DAT;
       Index++;
       pI2cHw->CONSET = LPC_I2C_AA; 
     }
     else //�������ջ���������������
       pI2cHw->CONCLR = LPC_I2C_AA;  
   }
   else eState = eI2cSlvErr;   //״̬������
   pI2cSlv->Index = Index;
   break;
  //===================��������״̬������־======================
    case 0xA0:{
    unsigned char CmdSize, DataSize;
    if(eState == eI2cSlvRd){//��������ֹ
      //û���յ�����������������)����״̬��ʾ����������
      if(!pI2cSlv->Index){
        eState = eI2SlvCmdRcv;
        pI2cSlv->eState = eI2SlvCmdRcv;
      }
      //���յ������ݣ���ʾ�ڽ�������״̬��ֹ
      CmdSize = pCmd->CmdSize;
      DataSize = pI2cSlv->Index;
    }
    else{//����������ǰ����
      CmdSize = pI2cSlv->Index;
      DataSize = 0;
    }
    //�ص���������:׼���跢�͵����ݻ�������ݴ���
    signed char Resume = I2cSlv_cbFun(pI2cSlv, CmdSize, DataSize);
    if(Resume <= 0) {//ֹͣI2C����
      pI2cHw->CR_f.AA = 0;
      eState = eI2cSlvRdy;//׼��
    }
    else{ //׼����д����
      pI2cHw->CONSET = LPC_I2C_AA; //Ӧ��
      pCmd->DataSize = Resume;
      pI2cSlv->Index = 0;
      eState = eI2cSlvWr;//׼��д����
    }
    break;
  }
  //======================���յ�������ַ+����־,�ӻ�Ϊд================
  case  0xB0://��ַ�յ�����Ӧ��
  case  0xA8://��ַ�յ�����Ӧ��,�������ض�ʧ������Ӱ��ӻ�
    //�������ֻ��������������ڴӻ�״̬ʱ�������
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
       pI2cHw->DAT = *(pCmd->pData + Index);
       Index++;
       pI2cHw->CONSET = LPC_I2C_AA; //����Ϊ����״̬
     }
     else pI2cHw->DAT = 0;//������������С��
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
  
  pI2cHw->CONCLR = LPC_I2C_SI;//���ж�
  
  //��I2C״̬����,��ǿ�н���I2c����
  if(eState == eI2cSlvErr){
    pI2cHw->CONCLR = LPC_I2C_AA;
    //�����Զ��ָ�
    eState = eI2cSlvRdy;
    pI2cHw->CONSET = LPC_I2C_AA; 
  }
  
  pI2cSlv->eState = eState;
  
  LPC_BASE_VIC->VectAddr = 0x00;              // �жϴ������
}
