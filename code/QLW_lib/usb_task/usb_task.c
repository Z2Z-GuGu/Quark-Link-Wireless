/**********************************************************************************
 * @name        usb_task.c
 * @version     R3.3
 * @brief       usb task
 * @author      Z2Z GuGu
 * @date        2023/04/28
 * @code        GB2312
 * @details     Implementing USB Composite Device (Framework) Using CDC-ACM Protocol
 *              - Simplest Program for Single Serial Port
 *              - USB IN OUT UART RX TX finsh
 **********************************************************************************/

#include "usb_task.h"

uint8_t UsbConfigRdy;
const uint8_t* pDescr;    //USB���ñ�־

uint8_t DevConfig;
uint8_t SetupReqCode;
uint16_t SetupReqLen;

/*�豸������*/
const uint8_t TAB_USB_CDC_DEV_DES[] = 
{
  0x12,
  0x01,
  0x00, // 0x10
  0x02, // 0x01
  0x00,                 //bDeviceClass: CDC-control 0x02 
  0x00,
  0x00,
  MAX_PACKET_SIZE,
  0x86, 0x1a,           //����ID 861a->VID=1a86    0xc4, 0x10,
  0x40, 0x80,           //��ƷID 4080->PID=8040 //CH9340    0x60, 0xea,  CP210x    CDCOK:0x22, 0x57,
  0x00, 0x01,           //�豸�汾��
  0x01,                 //�����ߵ��ַ���������������ֵ
  0x02,                 //��Ʒ���ַ���������������ֵ
  0x03,                 //��ŵ��ַ���������������ֵ
  0x01                  //�������õ���Ŀ
};

/* ���������� */
const uint8_t TAB_USB_CDC_CFG_DES[] =
{
  0x09, /* bLength: Configuration Descriptor size */
  0x02, /* bDescriptorType: Configuration */
  0x6B, // sizeof(CustomHID_ConfigDescriptor), ��̬���� 107 = 9 + 2 * 49
  0x00, /* wTotalLength: Bytes returned */
  4,    /* bNumInterfaces: 1 interface �ܵĽӿ�����*/
  0x01, /* bConfigurationValue: Configuration value */
  0x00, /* iConfiguration: Index of string descriptor describing the configuration*/
  0xC0, /* bmAttributes: Self powered */
  0x32, /* MaxPower 100 mA: this current is used for detecting Vbus */

  /*---------------------------------------------------------------------------*/
  //  IAD  Interface Association Descriptor��//IAD��������Ҫ���
  0x08, // bLength: Interface Descriptor size
  0x0B, // bDescriptorType: IAD
  0x00, // bFirstInterface                      //��һ�����ƽӿڵ���� ���ƽӿ�0
  0x02, // bInterfaceCount                      //��IDA�Ľӿ����� Ĭ��2
  0x02, // bFunctionClass: CDC                  //������IAD��һ��CDC�豸
  0x02, // bFunctionSubClass                    //Ĭ��
  0x01, // bFunctionProtocol                    //����Э���������Ҳ������Ĭ�Ͼ���
  0x02, // iFunction

  /*Interface Descriptor ���ƽӿ�*/
  0x09, /* bLength: Interface Descriptor size */
  0x04, /* bDescriptorType: Interface */ /* Interface descriptor type */
  0x00, /* bInterfaceNumber: Number of Interface  */
  0x00, /* bAlternateSetting: Alternate setting */
  0x00, /* bNumEndpoints: One endpoints used */
  0x02, /* bInterfaceClass: Communication Interface Class */
  0x02, /* bInterfaceSubClass: Abstract Control Model */
  0x01, /* bInterfaceProtocol: Common AT commands */
  0x00, /* iInterface: */
        //    /*Header Functional Descriptor �Ǳ�����*/
        //    0x05,   /* bLength: Endpoint Descriptor size */
        //    0x24,   /* bDescriptorType: CS_INTERFACE */
        //    0x00,   /* bDescriptorSubtype: Header Func Desc */
        //    0x10,   /* bcdCDC: spec release number */
        //    0x01,
        //    /*Call Management Functional Descriptor*/
        //    0x05,   /* bFunctionLength */
        //    0x24,   /* bDescriptorType: CS_INTERFACE */
        //    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
        //    0x00,   /* bmCapabilities: D0+D1 */
        //    0x01,   /* bDataInterface: 1 */
  /*ACM Functional Descriptor ������*/
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Ҫ֧�ֵ������� SET_LINE_CODING��Set_Control_Line_State��Get_Line_Coding��Serial_State */
  0x0F, /* bmCapabilities  */

  /*Union Functional Descriptor �������*/
  0x05, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x06, /* bDescriptorSubtype: Union func desc */
  0x00, /* bMasterInterface: Communication class interface ���ƽӿ�0*/
  0x01, /* bSlaveInterface0: Data Class Interface ���ݽӿ�1*/
        //    /*Endpoint 2 Descriptor �Ǳ�����Ŀ ��ǰ�ӿڶ˵���������Ϊ0*/
        //    0x07,   /* bLength: Endpoint Descriptor size */
        //    0x05,   /* bDescriptorType: Endpoint */
        //    0x8A,   /* bEndpointAddress: (INx)  ��Ч�˿�*/
        //    0x03,   /* bmAttributes: Interrupt */
        //    8,      /* wMaxPacketSize: */
        //    0x00,
        //    0xFF,   /* bInterval: */

  /*Data class interface descriptor  ���ݽӿ�*/
  0x09,                          /* bLength: Endpoint Descriptor size */
  0x04, /* bDescriptorType: */
  0x01, /* bInterfaceNumber: Number of Interface */
  0x00, /* bAlternateSetting: Alternate setting */
  0x02, /* bNumEndpoints: Two endpoints used */
  0x0A, /* bInterfaceClass: CDC */
  0x00, /* bInterfaceSubClass: */
  0x00, /* bInterfaceProtocol: */
  0x00, /* iInterface: */

  /*Endpoint 3 Descriptor*/
  0x07, /* bLength: Endpoint Descriptor size */
  0x05, /* bDescriptorType: Endpoint */
  0x01, /* bEndpointAddress: (OUT1) */
  0x02, /* bmAttributes: Bulk */
  MAX_PACKET_SIZE,   /* wMaxPacketSize: */
  0x00,
  0x00, /* bInterval: ignore for Bulk transfer */

  /*Endpoint 1 Descriptor*/
  0x07, /* bLength: Endpoint Descriptor size */
  0x05, /* bDescriptorType: Endpoint */
  0x81, /* bEndpointAddress: (IN1) */
  0x02, /* bmAttributes: Bulk */
  MAX_PACKET_SIZE,   /* wMaxPacketSize: */
  0x00,
  0x00, /* bInterval */

  /*---------------------------------------------------------------------------*/
  // IAD  Interface Association Descriptor
  0x08, // bLength: Interface Descriptor size
  0x0B, // bDescriptorType: IAD
  0x02, // bFirstInterface    ���ƽӿ�2
  0x02, // bInterfaceCount    Ĭ��2
  0x02, // bFunctionClass: CDC
  0x02, // bFunctionSubClass
  0x01, // bFunctionProtocol
  0x02, // iFunction

  /*Interface Descriptor*/
  0x09, /* bLength: Interface Descriptor size */
  0x04, /* bDescriptorType: Interface */ /* Interface descriptor type */
  0x02, /* bInterfaceNumber: Number of Interface */
  0x00, /* bAlternateSetting: Alternate setting */
  0x00, /* bNumEndpoints: One endpoints used */
  0x02, /* bInterfaceClass: Communication Interface Class */
  0x02, /* bInterfaceSubClass: Abstract Control Model */
  0x01, /* bInterfaceProtocol: Common AT commands */
  0x00, /* iInterface: */
        //    /*Header Functional Descriptor*/
        //    0x05,   /* bLength: Endpoint Descriptor size */
        //    0x24,   /* bDescriptorType: CS_INTERFACE */
        //    0x00,   /* bDescriptorSubtype: Header Func Desc */
        //    0x10,   /* bcdCDC: spec release number */
        //    0x01,
        //    /*Call Management Functional Descriptor*/
        //    0x05,   /* bFunctionLength */
        //    0x24,   /* bDescriptorType: CS_INTERFACE */
        //    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
        //    0x00,   /* bmCapabilities: D0+D1 */
        //    0x01,   /* bDataInterface: 1 */
  /*ACM Functional Descriptor*/
  0x04, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x02, /* bDescriptorSubtype: Abstract Control Management desc */
  0x0F, /* bmCapabilities */

  /*Union Functional Descriptor*/
  0x05, /* bFunctionLength */
  0x24, /* bDescriptorType: CS_INTERFACE */
  0x06, /* bDescriptorSubtype: Union func desc */
  0x02, /* bMasterInterface: Communication class interface ���ƽӿ�2*/
  0x03, /* bSlaveInterface0: Data Class Interface ���ݽӿ�3 */
        //    /*Endpoint 2 Descriptor*/
        //    0x07,   /* bLength: Endpoint Descriptor size */
        //    0x05,   /* bDescriptorType: Endpoint */
        //    0x8B,   /* bEndpointAddress: (INx)  ��Ч�˿�*/
        //    0x03,   /* bmAttributes: Interrupt */
        //    8,      /* wMaxPacketSize: */
        //    0x00,
        //    0xFF,   /* bInterval: */

  //���ݽӿ�
  /*Data class interface descriptor*/
  0x09, /* bLength: Endpoint Descriptor size */
  0x04, /* bDescriptorType: */
  0x03, /* bInterfaceNumber: Number of Interface */
  0x00, /* bAlternateSetting: Alternate setting */
  0x02, /* bNumEndpoints: Two endpoints used */
  0x0A, /* bInterfaceClass: CDC */
  0x00, /* bInterfaceSubClass: */
  0x00, /* bInterfaceProtocol: */
  0x00, /* iInterface: */
  /*Endpoint 3 Descriptor*/
  0x07, /* bLength: Endpoint Descriptor size */
  0x05, /* bDescriptorType: Endpoint */
  0x02, /* bEndpointAddress: (OUT1) */
  0x02, /* bmAttributes: Bulk */
  MAX_PACKET_SIZE,   /* wMaxPacketSize: */
  0x00,
  0x00, /* bInterval: ignore for Bulk transfer */
  /*Endpoint 1 Descriptor*/
  0x07, /* bLength: Endpoint Descriptor size */
  0x05, /* bDescriptorType: Endpoint */
  0x82, /* bEndpointAddress: (IN1) */
  0x02, /* bmAttributes: Bulk */
  MAX_PACKET_SIZE,   /* wMaxPacketSize: */
  0x00,
  0x00, /* bInterval */
};     /* CustomHID_ConfigDescriptor */

/*CDC�ַ���������*/
const uint8_t MyLangDescr[] = {0x04, 0x03, 0x09, 0x04};    //����������
// const uint8_t SerDes[] = {                              //���к��ַ���������
//     0x14, 0x03, 0x32, 0x00, 0x30, 0x00, 0x31, 0x00, 0x37, 0x00, 0x2D, 0x00, 0x32, 0x00, 0x2D, 0x00, 0x32, 0x00, 0x35,
//     0x00};
const uint8_t USB_SERIAL_STR[]=      "Quark-Linker-";
const uint8_t USB_PRODUCT_STR[]=     "Z2Z Device";
const uint8_t USB_MANUFACTURE_STR[]= "OpenSource";

const uint8_t MyProdInfo[] = {                             //��Ʒ�ַ���������
    0x14, 0x03, 0x43, 0x00, 0x48, 0x00, 0x35, 0x00, 0x35, 0x00, 0x34, 0x00, 0x5F, 0x00, 0x43, 0x00, 0x44, 0x00, 0x43,
    0x00,};
const uint8_t MyManuInfo[] = {0x0A, 0x03, 0x5F, 0x6c, 0xCF, 0x82, 0x81, 0x6c, 0x52, 0x60,};

//cdc ����
uint8_t LineCoding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};    //��ʼ��������Ϊ115200��1ֹͣλ����У�飬8����λ��

__attribute__((aligned(4))) uint8_t EP0_Databuf[MAX_PACKET_SIZE * 1];    //ep0(64)
__attribute__((aligned(4))) uint8_t EP1_Databuf[MAX_PACKET_SIZE * 2];    //ep1_out1(64)+ep1_out2(64)+ep1_in1(64)+ep1_in2(64)
__attribute__((aligned(4))) uint8_t EP2_Databuf[MAX_PACKET_SIZE * 2];    //ep2_out(64)+ep2_in(64)
__attribute__((aligned(4))) uint8_t EP1_TEMP_TX_BUF[MAX_PACKET_SIZE];
__attribute__((aligned(4))) uint8_t USB_TEMP_BUF[MAX_PACKET_SIZE];

// __attribute__((aligned(4)))      uint8_t EP3_Databuf[MAX_PACKET_SIZE * 2];    //ep3_out(64)+ep3_in(64)

/* USB��ʱ���ݴ������� */
uint8_t *pUSB_TEMP_BUF;
uint8_t *pEP1_TEMP_TX_BUF;
volatile uint8_t BOOT_State = 0x00;

/*******************************************************************************
 * @fn      InitCDCDevice
 * @brief   ��ʼ�� USB CDC �豸
 * @return  None
*******************************************************************************/
void InitCDCDevice(void)
{
  R8_UEP0_T_LEN = 0;
  R8_UEP1_T_LEN = 0;      //Ԥʹ�÷��ͳ���һ��Ҫ���
  // R8_UEP2_T_LEN = 0;      //Ԥʹ�÷��ͳ���һ��Ҫ���
  // R8_UEP3_T_LEN = 0;
  /* ��ʼ������ */
  R8_USB_CTRL = 0x00;                                                 // ���趨ģʽ

  R8_UEP4_1_MOD = RB_UEP1_RX_EN | RB_UEP1_TX_EN; // �˵�1 OUT+IN
  R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN;
  // R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN; // �˵�2 OUT+IN,�˵�3 OUT+IN

  pEP0_RAM_Addr = EP0_Databuf;
  pEP1_RAM_Addr = EP1_Databuf;
  pEP2_RAM_Addr = EP2_Databuf;
  // pEP3_RAM_Addr = EP3_Databuf;
  pUSB_TEMP_BUF = USB_TEMP_BUF;
  pEP1_TEMP_TX_BUF = EP1_TEMP_TX_BUF;

  R16_UEP0_DMA = (uint16_t)(uint32_t)pEP0_RAM_Addr;
  R16_UEP1_DMA = (uint16_t)(uint32_t)pEP1_RAM_Addr;
  R16_UEP2_DMA = (uint16_t)(uint32_t)pEP2_RAM_Addr;
  // R16_UEP3_DMA = (uint16_t)(uint32_t)pEP3_RAM_Addr;
  // R16_UEP4_DMA = (uint16_t)(uint32_t)EP4_Databuf;    // EP4������������EP0

  R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
  R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
  R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
  // R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
  // R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

  /* �豸��ַ */
  R8_USB_DEV_AD = 0x00;

  //��ֹDP/DM��������s
  R8_UDEV_CTRL = RB_UD_PD_DIS;

  // ����USB�豸��DMA�����ж��ڼ��жϱ�־δ���ǰ�Զ�����NAK
  R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN;
  // ��ֹUSB�˿ڸ��ռ���������
  R16_PIN_ANALOG_IE |= RB_PIN_USB_IE | RB_PIN_USB_DP_PU;         

  // ���жϱ�־
  R8_USB_INT_FG = 0xFF;
  // ����USB�˿�
  R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;                   
  //�����ж�          ����            �������         ���߸�λ
  R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_TRANSFER | RB_UIE_BUS_RST;
  PFIC_EnableIRQ(USB_IRQn);

  //ʹ��USB�˿�
  R8_UDEV_CTRL |= RB_UD_PORT_EN;

  // ��ʼ��BOOT����
  BOOT_State_Init(&BOOT_State);
}

// �����ˣ���д�������ɣ���UART�Ǳ�һ��������������Ż�����
void usb_auto_send(void)
{
  if(EP1_Buf_State & T_Data_EN_Bit)
  {
    if(EP1_Buf_State & T_Datax_Busy_Bit)
    {  // Data1 busy
      // USB auto send
      R8_UEP1_T_LEN = EP1_T_Buf1_Len;
      R8_UEP1_CTRL = R8_UEP1_CTRL & (~MASK_UEP_T_RES);
      EP1_Buf_State &= ~T_Data_EN;
    }
    else
    {  // Data0 busy
      R8_UEP1_T_LEN = EP1_T_Buf0_Len;
      R8_UEP1_CTRL = R8_UEP1_CTRL & (~MASK_UEP_T_RES);
      EP1_Buf_State &= ~T_Data_EN;
    }
  }
}

volatile uint8_t EP1_Buf_State = 0x00;
volatile uint8_t LINE_CODING_STATE = 0x00;
volatile uint8_t COM0_CTRL_LINE_STATE = 0x00;
uint8_t EP1_R_Buf0_Len = 0;
uint8_t EP1_R_Buf1_Len = 0;
uint8_t EP1_R_Buf_Len = 0;
uint8_t EP1_TEMP_Buf_Ready = 0;
uint8_t EP1_IN_Busy = 0;
uint8_t EP1_IN_Tail = 0;
uint8_t EP1_IN_Tail_LEN = 0;
// ��64֡��������
// = 0 DATA0��֡
// = 1 DATA1��֡
// = 0xff δ���ֿ�֡
uint8_t Frame_loss_mark = 0xff;
// uint32_t USB_R_Count = 0;
/*******************************************************************************
 * @fn      USB_IRQHandler
 * @brief   USB�жϺ���
 * @return  none
*******************************************************************************/
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void USB_IRQHandler(void)
{
  uint8_t j, len, SetupType;
  uint8_t errflag = 0;
  uint8_t intflag = R8_USB_INT_FG;
          DEBUG_PIN(USB_Pin, GPIO_HIGH);

  if (intflag & RB_UIF_TRANSFER)
  {
    if ((R8_USB_INT_ST & MASK_UIS_TOKEN) != MASK_UIS_TOKEN)     // ��Setup������
    {
      switch (R8_USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP)) // �����������ƺͶ˵��
      {
        case UIS_TOKEN_IN | 0  : {
          switch (SetupReqCode)
          {
            case USB_GET_DESCRIPTOR :
              len = SetupReqLen >= MAX_PACKET_SIZE ?
                  MAX_PACKET_SIZE : SetupReqLen;    // ���δ��䳤��
              memcpy(pEP0_DataBuf, pDescr, len); /* �����ϴ����� */
              SetupReqLen -= len;
              pDescr += len;
              R8_UEP0_T_LEN = len;
              R8_UEP0_CTRL ^= RB_UEP_T_TOG;    // ��ת
              break;
            case USB_SET_ADDRESS :
              R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | SetupReqLen;
              R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
              break;
            default :
              R8_UEP0_T_LEN = 0;               // ״̬�׶�����жϻ�����ǿ���ϴ�0�������ݰ��������ƴ���
              R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
              break;
          }
        } break;
          
        case UIS_TOKEN_OUT | 0 :  {
          uint32_t set_bps;
          uint8_t  data_bit;
          uint8_t  stop_bit;
          uint8_t  ver_bit;
            // stop_bit = Ep0Buffer[4];
            // 0 - 1
            // 1 - 1.5
            // 2 - 2
            // ver_bit = Ep0Buffer[5];
            // 0 - ��
            // 1 - ��
            // 2 - ż
            // data_bit = Ep0Buffer[6];
          len = R8_USB_RX_LEN;
          R8_UEP0_CTRL = (R8_UEP0_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
          if(LINE_CODING_STATE & COM0_NEW_Data_Bit)
          {
            LINE_CODING_STATE &= ~COM0_NEW_Data;
            memcpy(&set_bps, EP0_Databuf, 4);
            stop_bit = EP0_Databuf[4];
            ver_bit = EP0_Databuf[5];
            data_bit = EP0_Databuf[6];

            // EP2_Databuf[64] = EP0_Databuf[0];
            // EP2_Databuf[65] = EP0_Databuf[1];
            // EP2_Databuf[66] = EP0_Databuf[2];
            // EP2_Databuf[67] = EP0_Databuf[3];
            // EP2_Databuf[68] = EP0_Databuf[4];
            // EP2_Databuf[69] = EP0_Databuf[5];
            // EP2_Databuf[70] = EP0_Databuf[6];
            // R8_UEP2_T_LEN = 7;
            // R8_UEP2_CTRL = R8_UEP2_CTRL & (~MASK_UEP_T_RES);


            if(set_bps != 0)
            {
              UART0_BaudRateCfg(set_bps);
              UART_RX_TimeOUT_Init(set_bps);
            }
            if(data_bit != 0)
            {
              switch (data_bit)
              {
                case HAL_UART_5_BITS_PER_CHAR:
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_WORD_SZ_MASK)) | UART_WORD_SZ_5bit;
                  break;
                case HAL_UART_6_BITS_PER_CHAR:
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_WORD_SZ_MASK)) | UART_WORD_SZ_6bit;
                  break;
                case HAL_UART_7_BITS_PER_CHAR:
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_WORD_SZ_MASK)) | UART_WORD_SZ_7bit;
                  break;
                case HAL_UART_8_BITS_PER_CHAR:
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_WORD_SZ_MASK)) | UART_WORD_SZ_8bit;
                  break;
                default:
                  break;
              }
              switch (stop_bit)
              {
                case HAL_UART_ONE_STOP_BIT:
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_STOP_BIT_MASK)) | UART_STOP_1bit;
                  break;
                case HAL_UART_1_5_STOP_BIT:
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_STOP_BIT_MASK)) | UART_STOP_1bit;
                  break;
                case HAL_UART_TWO_STOP_BITS:
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_STOP_BIT_MASK)) | UART_STOP_2bit;
                  break;
                default:
                  break;
              }
              switch (ver_bit)
              {
                case HAL_UART_NO_PARITY:
                  R8_UART0_LCR &= ~RB_LCR_PAR_EN;
                  break;
                case HAL_UART_ODD_PARITY:
                  R8_UART0_LCR |= RB_LCR_PAR_EN;
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_PAR_MOD_MASK)) | UART_ODD_PARITY;
                  break;
                case HAL_UART_EVEN_PARITY:
                  R8_UART0_LCR |= RB_LCR_PAR_EN;
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_PAR_MOD_MASK)) | UART_EVEN_PARITY;
                  break;
                case HAL_UART_MARK_PARITY:
                  R8_UART0_LCR |= RB_LCR_PAR_EN;
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_PAR_MOD_MASK)) | UART_MARK_PARITY;
                  break;
                case HAL_UART_SPACE_PARITY:
                  R8_UART0_LCR |= RB_LCR_PAR_EN;
                  R8_UART0_LCR = (R8_UART0_LCR & (~UART_PAR_MOD_MASK)) | UART_SPACE_PARITY;
                  break;
                default:
                  break;
              }
            }
          }
          else if(LINE_CODING_STATE & COM1_NEW_Data_Bit)
          {
            LINE_CODING_STATE &= ~COM1_NEW_Data;
            memcpy(&set_bps, EP0_Databuf, 4);
            stop_bit = EP0_Databuf[4];
            ver_bit = EP0_Databuf[5];
            data_bit = EP0_Databuf[6];
          }
        }  break;
          
        case UIS_TOKEN_IN | 1 : {
          // R8_UEP1_T_LEN = 0;
          R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
          EP1_IN_Busy = 0;
          if(EP1_IN_Tail)
          { // β֡����
            EP1_IN_Tail = 0;
            for(j=0; j<(MAX_PACKET_SIZE/4); j++)            //������֡
              ((UINT32 *)(EP1_Databuf + 64))[j] = ((UINT32 *)(EP1_TEMP_TX_BUF))[j];
            R8_UEP1_T_LEN = EP1_IN_Tail_LEN;
            R8_UEP1_CTRL = R8_UEP1_CTRL & (~MASK_UEP_T_RES);
            EP1_IN_Busy = 1;
          }
        }  break;
          
        case UIS_TOKEN_OUT | 1 :  {
          if (R8_USB_INT_ST & RB_UIS_TOG_OK)
          {                      // ��ͬ�������ݰ�������
            uint8_t R_index, T_index, j;
            // USB_R_Count += R8_USB_RX_LEN;
            EP1_R_Buf_Len = R8_USB_RX_LEN;
            for(j=0; j<(MAX_PACKET_SIZE/4); j++)            //������֡
              ((UINT32 *)(USB_TEMP_BUF))[j] = ((UINT32 *)(EP1_Databuf))[j];
            R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
            EP1_TEMP_Buf_Ready = 1;

            // BOOT Chech
            BOOT_Serial_Chack(EP1_Databuf[0]);
          }
        }  break;

        case UIS_TOKEN_OUT | 2 : {
          if (R8_USB_INT_ST & RB_UIS_TOG_OK)
          {                      // ��ͬ�������ݰ�������
            len = R8_USB_RX_LEN;
            // ����ACK��ֹ����
            R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
          }
        } break;
          
        case UIS_TOKEN_IN | 2 : {
          R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        }  break;
          
      /*  Other EndPoint
        case UIS_TOKEN_OUT | 3 :  {
          if (R8_USB_INT_ST & RB_UIS_TOG_OK)
          {                      // ��ͬ�������ݰ�������
            len = R8_USB_RX_LEN;
          }
        }  break;
          
        case UIS_TOKEN_IN | 3 : {
          R8_UEP3_CTRL = (R8_UEP3_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        } break;
          
        case UIS_TOKEN_OUT | 4 :  {
          if (R8_USB_INT_ST & RB_UIS_TOG_OK)
          {
            R8_UEP4_CTRL ^= RB_UEP_R_TOG;
            len = R8_USB_RX_LEN;
          }
        }  break;
          
        case UIS_TOKEN_IN | 4 : {
          R8_UEP4_CTRL ^= RB_UEP_T_TOG;
          R8_UEP4_CTRL = (R8_UEP4_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        }  break;
      */
        default :
          break;
      }
      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
    
    if (R8_USB_INT_ST & RB_UIS_SETUP_ACT)                  // Setup������
    {
      R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
      SetupReqLen = pSetupReqPak->wLength;
      SetupReqCode = pSetupReqPak->bRequest;
      SetupType = pSetupReqPak->bRequestType;
      
      len = 0;
      errflag = 0;
      if ((SetupType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) // �Ǳ�׼����
      {
        switch (SetupReqCode)
        {
          case 0x01 :                  //GetReport
            break;
          case 0x02 :                  //GetIdle
            break;
          case 0x03 :                  //GetProtocol
            break;
          case 0x09 :                  //SetReport
            break;
          case 0x0A :                  //SetIdle
            break;
          case 0x0B :                  //SetProtocol
            break;
          case DEF_GET_LINE_CODING :    //0x21  currently configured
            pDescr = LineCoding;
            len = sizeof(LineCoding);
            len = SetupReqLen >= DEFAULT_ENDP0_SIZE ?
                DEFAULT_ENDP0_SIZE : SetupReqLen;
            memcpy(pEP0_DataBuf, pDescr, len);
            SetupReqLen -= len;
            pDescr += len;
            break;
          case DEF_SET_CONTROL_LINE_STATE :    //0x22  generates RS-232/V.24 style control signals
            // DEBUG_PIN(A10_Pin, GPIO_INVER);
            // DTR RTS
            switch (UsbSetupBuf->wIndexL)
            {
              case COM0_indexL:
                // DEBUG_PIN(A10_Pin, GPIO_INVER);
                if(UsbSetupBuf->wValueL & EP0_DTR_Bit)
                {
                  if(!(COM0_CTRL_LINE_STATE & COM_DTR_Bit))
                  {
                    COM0_CTRL_LINE_STATE |= COM_DTR_NEW_Edge;
                    COM0_CTRL_LINE_STATE |= COM_DTR_Rising;
                  }
                  else
                  {
                    COM0_CTRL_LINE_STATE &= ~COM_DTR_NEW_Edge;
                  }
                  COM0_CTRL_LINE_STATE |= COM_NEW_DTR;
                  COM0_CTRL_LINE_STATE |= COM_DTR_H;
                  DEBUG_PIN(A12_Pin, GPIO_HIGH);
                }
                else
                {
                  if(COM0_CTRL_LINE_STATE & COM_DTR_Bit)
                  {
                    COM0_CTRL_LINE_STATE |= COM_DTR_NEW_Edge;
                    COM0_CTRL_LINE_STATE &= ~COM_DTR_Rising;
                  }
                  else
                  {
                    COM0_CTRL_LINE_STATE &= ~COM_DTR_NEW_Edge;
                  }
                  COM0_CTRL_LINE_STATE |= COM_NEW_DTR;
                  COM0_CTRL_LINE_STATE &= ~COM_DTR_H;
                  DEBUG_PIN(A12_Pin, GPIO_LOW);
                }
                if(UsbSetupBuf->wValueL & EP0_RTS_Bit)
                {
                  if(!(COM0_CTRL_LINE_STATE & COM_RTS_Bit))
                  {
                    COM0_CTRL_LINE_STATE |= COM_RTS_NEW_Edge;
                    COM0_CTRL_LINE_STATE |= COM_RTS_Rising;
                  }
                  else
                  {
                    COM0_CTRL_LINE_STATE &= ~COM_RTS_NEW_Edge;
                  }
                  COM0_CTRL_LINE_STATE |= COM_NEW_RTS;
                  COM0_CTRL_LINE_STATE |= COM_RTS_H;
                  DEBUG_PIN(A11_Pin, GPIO_HIGH);
                }
                else
                {
                  if(COM0_CTRL_LINE_STATE & COM_RTS_Bit)
                  {
                    COM0_CTRL_LINE_STATE |= COM_RTS_NEW_Edge;
                    COM0_CTRL_LINE_STATE &= ~COM_RTS_Rising;
                  }
                  else
                  {
                    COM0_CTRL_LINE_STATE &= ~COM_RTS_NEW_Edge;
                  }
                  COM0_CTRL_LINE_STATE |= COM_NEW_RTS;
                  COM0_CTRL_LINE_STATE &= ~COM_RTS_H;
                  DEBUG_PIN(A11_Pin, GPIO_LOW);
                }
                BOOT_CTRL_LINE_Check(COM0_CTRL_LINE_STATE);
                break;
              case COM1_indexL:
                // DEBUG_PIN(A10_Pin, GPIO_INVER);
                break;
              
              default:
                break;
            }
            break;
          case DEF_SET_LINE_CODING :      //0x20  Configure
                // DEBUG_PIN(A10_Pin, GPIO_INVER);
            switch (UsbSetupBuf->wIndexL)
            {
              case COM0_indexL:
                LINE_CODING_STATE |= COM0_NEW_Data;
                // DEBUG_PIN(A10_Pin, GPIO_INVER);
                break;
              case COM1_indexL:
                LINE_CODING_STATE |= COM1_NEW_Data;
                // DEBUG_PIN(A10_Pin, GPIO_INVER);
                break;
              
              default:
                len = 0;
                break;
            }
            break;
          default :
            errflag = 0xFF;
            break;
        }
      }
      else          // ��׼����
      {
        switch (SetupReqCode)
        {
          case USB_GET_DESCRIPTOR :
          {
            switch (((pSetupReqPak->wValue) >> 8))
            {
              case USB_DESCR_TYP_DEVICE :   // �豸������
              {
                pDescr = TAB_USB_CDC_DEV_DES;
                len = TAB_USB_CDC_DEV_DES[0];
              }
                break;
                
              case USB_DESCR_TYP_CONFIG :   // ����������
              {
                pDescr = TAB_USB_CDC_CFG_DES;
                len = TAB_USB_CDC_CFG_DES[2];
              }
                break;
                
              case USB_DESCR_TYP_STRING :   // �ַ���������
              {
                switch ((pSetupReqPak->wValue) & 0xff)
                {
                  case 1 :    //iManufacturer
                    pDescr = MyManuInfo;
                    len = MyManuInfo[0];
                    break;
                  case 2 :    // iProduct
                    pDescr = MyProdInfo;
                    len = MyProdInfo[0];
                    break;
                  case 0 :    // ����������
                    pDescr = MyLangDescr;
                    len = MyLangDescr[0];
                    break;
                  case 3 :    // iSerialNumber
                  {
                    uint8_t ep0_str_len;
                    uint8_t *p_send;
                    uint8_t *manu_str;
                    uint8_t tmp;

                    /* ȡ���� */
                    if(UsbSetupBuf->wValueL == 1)
                      manu_str = (uint8_t *)USB_MANUFACTURE_STR;
                    else if(UsbSetupBuf->wValueL == 2)
                      manu_str = (uint8_t *)USB_PRODUCT_STR;
                    else if(UsbSetupBuf->wValueL == 3)
                    {
                      // DEBUG_PIN(A10_Pin, GPIO_INVER);
                      // if(UsbSetupBuf->wIndexL == COM0_indexL)
                      // {
                      //   manu_str = (uint8_t *)USB_SERIAL0_STR;
                      // }
                      // else // if(UsbSetupBuf->wIndexL == COM1_indexL)
                      // {
                      //   manu_str = (uint8_t *)USB_SERIAL1_STR;
                      // }
                      manu_str = (uint8_t *)USB_SERIAL_STR;
                    }
                    ep0_str_len = (uint8_t)strlen((char *)manu_str);
                    p_send = USB_TEMP_BUF;
                    *p_send++ = ep0_str_len*2 + 2;
                    *p_send++ = 0x03;
                    for(tmp = 0; tmp<ep0_str_len; tmp++)
                    {
                      *p_send++ = manu_str[tmp];
                      *p_send++ = 0x00;
                    }

                    pDescr = USB_TEMP_BUF;
                    len = USB_TEMP_BUF[0];
                    // pDescr = SerDes;
                    // len = sizeof(SerDes);
                  }  break;
                  default :
                    errflag = 0xFF;          // ��֧�ֵ��ַ���������
                    break;
                }
              }
                break;
                
              default :
                errflag = 0xff;
                break;
            }
            if (SetupReqLen > len)
              SetupReqLen = len;      //ʵ�����ϴ��ܳ���
            len = (SetupReqLen >= MAX_PACKET_SIZE) ?
                MAX_PACKET_SIZE : SetupReqLen;
            memcpy(pEP0_DataBuf, pDescr, len);
            pDescr += len;
          }
            break;
            
          case USB_SET_ADDRESS :
            SetupReqLen = (pSetupReqPak->wValue) & 0xff;
            break;
            
          case USB_GET_CONFIGURATION :
            pEP0_DataBuf[0] = DevConfig;
            if (SetupReqLen > 1)
            {
              SetupReqLen = 1;
            }
            UsbConfigRdy = 1;    //USBö����ɱ�־
            break;
            
          case USB_SET_CONFIGURATION :
            DevConfig = (pSetupReqPak->wValue) & 0xff;
            break;
            
          case USB_CLEAR_FEATURE :
          {
            if ((pSetupReqPak->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)    // �˵�
            {
              switch ((pSetupReqPak->wIndex) & 0xff)
              {
                
                case 0x81 :
                  R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
                  break;
                case 0x01 :
                  R8_UEP1_CTRL = (R8_UEP1_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK;
                  break;
                case 0x82 :
                  R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
                  break;
                case 0x02 :
                  R8_UEP2_CTRL = (R8_UEP2_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK;
                  break;
                case 0x83 :
                  R8_UEP3_CTRL = (R8_UEP3_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
                  break;
                case 0x03 :
                  R8_UEP3_CTRL = (R8_UEP3_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK;
                  break;
                case 0x84 :
                  R8_UEP4_CTRL = (R8_UEP4_CTRL & ~(RB_UEP_T_TOG | MASK_UEP_T_RES)) | UEP_T_RES_NAK;
                  break;
                case 0x04 :
                  R8_UEP4_CTRL = (R8_UEP4_CTRL & ~(RB_UEP_R_TOG | MASK_UEP_R_RES)) | UEP_R_RES_ACK;
                  break;
                default :
                  errflag = 0xFF;                                 // ��֧�ֵĶ˵�
                  break;
              }
            }
            
            if ((pSetupReqPak->bRequestType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE)
            {
              break;
            }
            else
            {
              len = 0xFF;
            }
          }
            break;
          case USB_SET_FEATURE:                                              /* Set Feature */
            if ((pSetupReqPak->bRequestType & 0x1F) == 0x00) {
              if (pSetupReqPak->wValue == 0x01) {
                if (TAB_USB_CDC_CFG_DES[7] & 0x20) {
                } else {
                  len = 0xFF;
                }
              } else {
                len = 0xFF;
              }
            } else if ((pSetupReqPak->bRequestType & 0x1F) == 0x02) {
              if (pSetupReqPak->wValue == 0x00) {
                switch (pSetupReqPak->wIndex) {
                  case 0x04:
                    R8_UEP4_CTRL = (R8_UEP4_CTRL & (~RB_UEP_R_TOG)) | UEP_R_RES_STALL;
                    break;
                  case 0x84:
                    R8_UEP4_CTRL = (R8_UEP4_CTRL & (~RB_UEP_T_TOG)) | UEP_T_RES_STALL;
                    break;
                  case 0x01:
                    R8_UEP1_CTRL = (R8_UEP1_CTRL & (~RB_UEP_R_TOG)) | UEP_R_RES_STALL;
                    break;
                  case 0x81:
                    R8_UEP1_CTRL = (R8_UEP1_CTRL & (~RB_UEP_T_TOG)) | UEP_T_RES_STALL;
                    break;
                  case 0x02:
                    R8_UEP2_CTRL = (R8_UEP2_CTRL & (~RB_UEP_R_TOG)) | UEP_R_RES_STALL;
                    break;
                  case 0x82:
                    R8_UEP2_CTRL = (R8_UEP2_CTRL & (~RB_UEP_T_TOG)) | UEP_T_RES_STALL;
                    break;
                  case 0x03:
                    R8_UEP3_CTRL = (R8_UEP3_CTRL & (~RB_UEP_R_TOG)) | UEP_R_RES_STALL;
                    break;
                  case 0x83:
                    R8_UEP3_CTRL = (R8_UEP3_CTRL & (~RB_UEP_T_TOG)) | UEP_T_RES_STALL;
                    break;

                  default:
                    len = 0xFF;
                    break;
                }
              } else {
                len = 0xFF;
              }
            } else {
              len = 0xFF;
            }
            break;
            
          case USB_GET_INTERFACE :
          pEP0_DataBuf[0] = 0x00;
          if (SetupReqLen > 1)
          {
            SetupReqLen = 1;
          }
          break;

          case USB_GET_STATUS :
          pEP0_DataBuf[0] = 0x00;
          pEP0_DataBuf[1] = 0x00;
          if (SetupReqLen > 2)
          {
            SetupReqLen = 2;
          }
          break;

          default :
          errflag = 0xff;
          break;
        }
      }
      if (errflag == 0xff)        // �����֧��
      {
  //                  SetupReqCode = 0xFF;
        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;    // STALL
      }
      else
      {
        if (SetupType & 0x80)     // �ϴ�
        {
          len = (SetupReqLen > MAX_PACKET_SIZE) ?
              MAX_PACKET_SIZE : SetupReqLen;
          SetupReqLen -= len;
        }
        else
          len = 0;        // �´�
        R8_UEP0_T_LEN = len;
        R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;    // Ĭ�����ݰ���DATA1
      }

      R8_USB_INT_FG = RB_UIF_TRANSFER;
    }
  }
  else if (intflag & RB_UIF_BUS_RST)
  {
    R8_USB_DEV_AD = 0;
    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
    R8_USB_INT_FG = RB_UIF_BUS_RST;
 }
  else if (intflag & RB_UIF_SUSPEND)
  {
    if (R8_USB_MIS_ST & RB_UMS_SUSPEND)
    {
      ;
   }     // ����
    else
    {
      ;
   }     // ����
    R8_USB_INT_FG = RB_UIF_SUSPEND;
 }
  else
  {
    R8_USB_INT_FG = intflag;
 }
          DEBUG_PIN(USB_Pin, GPIO_LOW);
}
