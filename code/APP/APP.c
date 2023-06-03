/**********************************************************************************
 * @name        Application.c
 * @version     R1.1
 * @brief       Application
 * @author      Z2Z GuGu
 * @date        2023/05/28
 * @code        GB2312
 **********************************************************************************/

#include "APP.h"

static uint8_t task_list[Task_List_LENGTH] = {0};

app_drv_fifo_t task_list_fifo;

uint8_t APP_TaskID = INVALID_TASK_ID;

uint8_t *APP_BOOT_State;

// ϵͳ��״̬BitMap��������Data Flashͬ��ʵ�ֵ��籣��
volatile uint16_t SYS_STATE_BITMAP = 0x00;
// volatile uint16_t SYS_GPIO_BITMAP = 0x00;
// KEY & LED ״̬����
// volatile uint8_t LED_Bink_Mode[2] = {0};
// volatile uint16_t LED_STATE_BITMAP[2] = {0};
// volatile uint16_t KEY_STATE_BITMAP[2] = {0};
volatile uint8_t GPIO_FUNC[8] = {0};
// volatile uint8_t LED_STATE_BitMap[2] = {0};
volatile uint8_t LED_MODE[2] = {0};
volatile uint8_t KEY_MODE[2] = {0};
const uint32_t GPIO_Index[GPIO_INDEX_MAX] = {GPIO_G0_Pin, GPIO_G1_Pin, GPIO_G2_Pin, GPIO_G3_Pin, GPIO_G14_Pin, GPIO_G15_Pin, Board_QL_Pin};

void TIM2_Init()
{
    TMR2_TimerInit(0x3FFFFFF);
}

// #define SYS_STATE_Inex                  0x01
// #define SYS_LED_STATE_Inex              0x02
// #define SYS_KEY_STATE_Inex              0x03
// #define USR_LED_STATE_Inex              0x04
// #define USR_KEY_STATE_Inex              0x05
void SYS_Param_Config(uint8_t Param_Inex, uint16_t Param_Bit, uint8_t EN)
{
    if(EN)
    {
        switch(Param_Inex)
        {
            case SYS_STATE_Inex:
                SYS_STATE_BITMAP |= Param_Bit;
                break;
            case SYS_LED_STATE_Inex:
                LED_STATE_BITMAP[SYS_LED_INDEX] |= Param_Bit;
                break;
            case SYS_KEY_STATE_Inex:
                KEY_STATE_BITMAP[SYS_KEY_INDEX] |= Param_Bit;
                break;
            case USR_LED_STATE_Inex:
                LED_STATE_BITMAP[USR_LED_INDEX] |= Param_Bit;
                break;
            case USR_KEY_STATE_Inex:
                KEY_STATE_BITMAP[USR_KEY_INDEX] |= Param_Bit;
                break;
        }
    }
    else
    {
        switch(Param_Inex)
        {
            case SYS_STATE_Inex:
                SYS_STATE_BITMAP |= Param_Bit;
                break;
            case SYS_LED_STATE_Inex:
                LED_STATE_BITMAP[SYS_LED_INDEX] &= ~Param_Bit;
                break;
            case SYS_KEY_STATE_Inex:
                KEY_STATE_BITMAP[SYS_KEY_INDEX] &= ~Param_Bit;
                break;
            case USR_LED_STATE_Inex:
                LED_STATE_BITMAP[USR_LED_INDEX] &= ~Param_Bit;
                break;
            case USR_KEY_STATE_Inex:
                KEY_STATE_BITMAP[USR_KEY_INDEX] &= ~Param_Bit;
                break;
        }
    }
}

/*******************************************************************************
 * @fn      Timer_Task_Start
 * @brief   timeʱ���ִ�� id������
 * @param   overtime: ʱ��ms max = 1000;
 * @param   id:   ������
 * @return  SUCCESS / ~SUCCESS
 *******************************************************************************/
uint8_t Timer_Task_ID = 0;
uint8_t Timer_Task_Start(uint32_t overtime, uint8_t id)
{
    if(Timer_Task_ID)
        return ~SUCCESS;

    Timer_Task_ID = id;
    TMR2_ClearITFlag(TMR0_3_IT_CYC_END); // ����жϱ�־
    TMR2_TimerInit(overtime * 60000);     // ���ö�ʱʱ�� ms
    PFIC_EnableIRQ(TMR2_IRQn);
    TMR2_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // �����ж�
    return SUCCESS;
}

/*******************************************************************************
 * @fn      Timer_Task_Stop
 * @brief   ��ʱ����δ������
 * @return  None
*******************************************************************************/
void Timer_Task_Stop()
{
    Timer_Task_ID = 0;
    TMR2_ITCfg(DISABLE, TMR0_3_IT_CYC_END); // �ر��ж�
}

void BOOT_State_Init(uint8_t *pBOOT_State)
{
    APP_BOOT_State = pBOOT_State;
}

// �����Ź��ܳ����޸�ʱ��ִ�ж�Ӧ����
void GPIO_Mode_SETTING(void)
{
    if(GPIO_FUNC[7] != 0){
        if(GPIO_FUNC[7] & 0x01){
            // GPIO0
            if(GPIO_FUNC[0] == 0 || GPIO_FUNC[0] == 3 || GPIO_FUNC[0] == 4 || GPIO_FUNC[0] == 5){    // 5 = BOOT
                Pin_Ctrl(GPIO_G0_Pin, GPIO_HIGH);
                BB_IO_MODE_SETTING(GPIO_G0_Pin, GPIO_ModeOut_PP_5mA);
            }else{   // LED or R
                BB_IO_MODE_SETTING(GPIO_G0_Pin, GPIO_ModeIN_Floating);
            }
        }
        if(GPIO_FUNC[7] & 0x02){
            // GPIO1
            if(GPIO_FUNC[1] == 0 || GPIO_FUNC[1] == 5){
                // RX
                UART0_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                BB_IO_MODE_SETTING(GPIO_G1_Pin, GPIO_ModeIN_PU);
            }else if(GPIO_FUNC[1] == 3 || GPIO_FUNC[1] == 4){   // KEY
                UART0_INTCfg(DISABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                Pin_Ctrl(GPIO_G1_Pin, GPIO_HIGH);
                BB_IO_MODE_SETTING(GPIO_G1_Pin, GPIO_ModeOut_PP_5mA);
            }else{   // LED or R
                UART0_INTCfg(DISABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
                BB_IO_MODE_SETTING(GPIO_G1_Pin, GPIO_ModeIN_Floating);
            }
        }
        if(GPIO_FUNC[7] & 0x04){
            // GPIO2
            if(GPIO_FUNC[2] == 3 || GPIO_FUNC[2] == 4 || GPIO_FUNC[2] == 5){    // 5 = BOOT
                Pin_Ctrl(GPIO_G2_Pin, GPIO_HIGH);
                BB_IO_MODE_SETTING(GPIO_G2_Pin, GPIO_ModeOut_PP_5mA);
            }else{   // LED or R
                BB_IO_MODE_SETTING(GPIO_G2_Pin, GPIO_ModeIN_Floating);
            }
        }
        if(GPIO_FUNC[7] & 0x08){
            // GPIO3
            if(GPIO_FUNC[3] == 0 || GPIO_FUNC[3] == 5){
                R8_UART0_IER |= RB_IER_TXD_EN;
                BB_IO_MODE_SETTING(GPIO_G3_Pin, GPIO_ModeOut_PP_5mA);
            }if(GPIO_FUNC[3] == 3 || GPIO_FUNC[3] == 4){    // KEY
                R8_UART0_IER &= ~RB_IER_TXD_EN;
                Pin_Ctrl(GPIO_G3_Pin, GPIO_HIGH);
                BB_IO_MODE_SETTING(GPIO_G3_Pin, GPIO_ModeOut_PP_5mA);
            }else{   // LED or R
                R8_UART0_IER &= ~RB_IER_TXD_EN;
                BB_IO_MODE_SETTING(GPIO_G3_Pin, GPIO_ModeIN_Floating);
            }
        }
        if(GPIO_FUNC[7] & 0x10){
            // GPIO14
            if(GPIO_FUNC[4] == 5){    // SDIO
                BB_IO_MODE_SETTING(GPIO_G14_Pin, GPIO_ModeIN_PU);
            }else if(GPIO_FUNC[4] == 3 || GPIO_FUNC[4] == 4){    // KEY
                Pin_Ctrl(GPIO_G14_Pin, GPIO_HIGH);
                BB_IO_MODE_SETTING(GPIO_G14_Pin, GPIO_ModeOut_PP_5mA);
            }else{   // LED or R
                BB_IO_MODE_SETTING(GPIO_G14_Pin, GPIO_ModeIN_Floating);
            }
        }
        if(GPIO_FUNC[7] & 0x20){
            // GPIO15
            if(GPIO_FUNC[5] == 5){    // SDIO
                BB_IO_MODE_SETTING(GPIO_G15_Pin, GPIO_ModeIN_PU);
            }else if(GPIO_FUNC[5] == 3 || GPIO_FUNC[5] == 4){    // KEY
                Pin_Ctrl(GPIO_G15_Pin, GPIO_HIGH);
                BB_IO_MODE_SETTING(GPIO_G15_Pin, GPIO_ModeOut_PP_5mA);
            }else{   // LED or R
                BB_IO_MODE_SETTING(GPIO_G15_Pin, GPIO_ModeIN_Floating);
            }
        }
        if(GPIO_FUNC[7] & 0x40){
            // QL Pin
            if(GPIO_FUNC[6] == 0 || GPIO_FUNC[6] == 5){    // DEBUG
                BB_IO_MODE_SETTING(Board_QL_Pin, GPIO_ModeOut_PP_5mA);
            }else if(GPIO_FUNC[6] == 3 || GPIO_FUNC[6] == 4){    // KEY
                Pin_Ctrl(Board_QL_Pin, GPIO_HIGH);
                BB_IO_MODE_SETTING(Board_QL_Pin, GPIO_ModeOut_PP_5mA);
            }else{   // LED or R
                BB_IO_MODE_SETTING(Board_QL_Pin, GPIO_ModeIN_Floating);
            }
        }
    }
}

// ��ʵ��¼���GPIO��״̬������ΪLED״̬���¼��LED_MODE��
void GPIO_GETTING(void)
{
    // ����� LED_MODE ��Ҫ����֤һ��
    if((!(LED_MODE[SYS_LED_INDEX] & LED_COM_CTRL_Bit)) || (!(LED_MODE[USR_LED_INDEX] & LED_COM_CTRL_Bit))){
        GPIOStateTpDef TEMP_H[2];
        uint8_t i;
        uint8_t TEMP_LED_INDEX = 0xFF;
        // INVER
        if(LED_MODE[SYS_LED_INDEX] & LED_INVER_CTRL_Bit){
            TEMP_H[SYS_LED_INDEX] = GPIO_LOW;
        }else{
            TEMP_H[SYS_LED_INDEX] = GPIO_HIGH;
        }
        if(LED_MODE[USR_LED_INDEX] & LED_INVER_CTRL_Bit){
            TEMP_H[USR_LED_INDEX] = GPIO_LOW;
        }else{
            TEMP_H[USR_LED_INDEX] = GPIO_HIGH;
        }
        // GPIO0 - QL
        for (i = 0; i < GPIO_INDEX_MAX; i++){
            if (GPIO_FUNC[i] == 1)
            {
                TEMP_LED_INDEX = SYS_LED_INDEX;
            }else if(GPIO_FUNC[i] == 2){
                TEMP_LED_INDEX = USR_LED_INDEX;
            }
            // ����LED��UART�����������������
            if(LED_MODE[TEMP_LED_INDEX] & LED_COM_CTRL_Bit){
                TEMP_LED_INDEX = 0xFF;
            }
            if(TEMP_LED_INDEX != 0xFF){
                if(Pin_state(GPIO_Index[i]) == TEMP_H){
                    LED_MODE[TEMP_LED_INDEX] |= LED_ON_OFF_Bit;
                }else{
                    LED_MODE[TEMP_LED_INDEX] &= ~LED_ON_OFF_Bit;
                }
                // ��λ
                TEMP_LED_INDEX = 0xFF;
            }
        }
        // GPIO2 Default������INVERλӰ�죩
        if(LED_MODE[USR_LED_INDEX] & LED_COM_CTRL_Bit){
            if(GPIO_FUNC[2] == 0){
                if(Pin_state(GPIO_G2_Pin) == GPIO_HIGH){
                    LED_MODE[USR_LED_INDEX] |= LED_ON_OFF_Bit;
                }else{
                    LED_MODE[USR_LED_INDEX] &= ~LED_ON_OFF_Bit;
                }
            }
        }
    }
}

// ��ʵ��¼����KEY��״̬��KEY_MODE��
void KEY_STATE_GETTING(void)
{
    static GPIOStateTpDef KEY_old_val[2] = {GPIO_HIGH, GPIO_HIGH};
    static uint8_t SYS_KEY_DOWN_Time_Count = 0;
    GPIOStateTpDef KEY_val[2];
    // SYS KEY
    KEY_val[SYS_KEY_INDEX] = Pin_state(Sys_KEY_Pin);
    if(KEY_val[SYS_KEY_INDEX] != KEY_old_val[SYS_KEY_INDEX]){
        KEY_MODE[SYS_KEY_INDEX] |= KEY_NEW_DATA_Bit;
        if(KEY_val[SYS_KEY_INDEX] == GPIO_LOW){
            SYS_KEY_DOWN_Time_Count++;
            KEY_MODE[SYS_KEY_INDEX] |= KEY_UP_DOWN_Bit;
        }else{
            SYS_KEY_DOWN_Time_Count = 0;
            KEY_MODE[SYS_KEY_INDEX] &= ~KEY_UP_DOWN_Bit;
        }
        KEY_old_val[SYS_KEY_INDEX] = KEY_val[SYS_KEY_INDEX];
    }
    // USR KEY
    KEY_val[USR_KEY_INDEX] = Pin_state(User_KEY_Pin);
    if(KEY_val[USR_KEY_INDEX] != KEY_old_val[USR_KEY_INDEX]){
        KEY_MODE[USR_KEY_INDEX] |= KEY_NEW_DATA_Bit;
        if(KEY_val[USR_KEY_INDEX] == GPIO_LOW){
            KEY_MODE[USR_KEY_INDEX] |= KEY_UP_DOWN_Bit;
        }else{
            KEY_MODE[USR_KEY_INDEX] &= ~KEY_UP_DOWN_Bit;
        }
        KEY_old_val[USR_KEY_INDEX] = KEY_val[USR_KEY_INDEX];
    }
    // SYS LONG TIME PRESS
    if(SYS_KEY_DOWN_Time_Count >= 250){  // 5s
        SYS_KEY_DOWN_Time_Count = 0;
    }
}

// Ĭ�������SYS LED����ָ������
void SYS_STATE_to_SYS_LED(void)
{
    if(LED_MODE[SYS_LED_INDEX] & LED_COM_CTRL_Bit){
        return;
    } else if ((GPIO_FUNC[0] == 1) || (GPIO_FUNC[1] == 1) || \
               (GPIO_FUNC[2] == 1) || (GPIO_FUNC[3] == 1) || \
               (GPIO_FUNC[4] == 1) || (GPIO_FUNC[5] == 1) || (GPIO_FUNC[6] == 1)){
        return;
    }
    //  ȷ��SYS LED����Default״̬������UART���ƣ�����GPIO���ƣ�
    if(SYS_STATE_BITMAP & SYS_RUNNING_BREAK_STATE_Bit){  
        // ϵͳ����
        LED_MODE[SYS_LED_INDEX] &= ~LED_ON_OFF_Bit;     // ����
    } else {   // ϵͳ��������
        // SYS KEY
        if((KEY_MODE[SYS_KEY_INDEX] & KEY_UP_DOWN_Bit) || (KEY_MODE[SYS_KEY_INDEX] & KEY_SOFT_UP_DOWN_Bit)){
            LED_MODE[SYS_LED_INDEX] &= ~LED_ON_OFF_Bit;         // ����SYS������LEDϨ��ESP32 EN = 0
            // Pin_Ctrl(BOOT_EN_Pin, GPIO_LOW);
        } else {
            LED_MODE[SYS_LED_INDEX] |= LED_ON_OFF_Bit;
            // Pin_Ctrl(BOOT_EN_Pin, GPIO_LOW);
        }
        // CONNECT STATE
        if(SYS_STATE_BITMAP & SYS_USB_NEW_STATE_Bit) {
            // USB ����/�Ͽ���������
            SYS_STATE_BITMAP &= ~SYS_USB_NEW_STATE_Bit;
            LED_MODE[SYS_LED_INDEX] |= LED_BLINK_EN_Bit;
            LED_MODE[SYS_LED_INDEX] |= LED_BLINK_Times_Bit;     
        }
        if(SYS_STATE_BITMAP & SYS_BLE_NEW_STATE_Bit) {
            // BLE ����/�Ͽ���������
            SYS_STATE_BITMAP &= ~SYS_BLE_NEW_STATE_Bit;
            LED_MODE[SYS_LED_INDEX] |= LED_BLINK_EN_Bit;
            LED_MODE[SYS_LED_INDEX] |= LED_BLINK_Times_Bit;     
        }
        // UART ACTION
        if(SYS_STATE_BITMAP & SYS_UART_FLASHING_STATE_Bit){
            // ��¼�У�ÿ����5��
            LED_MODE[SYS_LED_INDEX] |= LED_BLINK_EN_Bit;
            LED_MODE[SYS_LED_INDEX] &= ~LED_BLINK_Times_Bit;
        }
        if((SYS_STATE_BITMAP & SYS_UART_TX_ACTION_Bit) || (SYS_STATE_BITMAP & SYS_UART_RX_ACTION_Bit)){
            // UART ACTION��ÿ����5��
            LED_MODE[SYS_LED_INDEX] |= LED_BLINK_EN_Bit;
            LED_MODE[SYS_LED_INDEX] &= ~LED_BLINK_Times_Bit;
        }
    }
}

// BOOT ִ��
void BOOT_Execute_GPIO_MODE(void)
{
    uint8_t G0_FUNC_SAVE, G2_FUNC_SAVE;
    if(*APP_BOOT_State & Execute_NOW_Bit){
        switch (*APP_BOOT_State){
            case ESP32_BOOT_PREPARE:
                // *APP_BOOT_State = ESP32_BOOT_NOW;
                G0_FUNC_SAVE = GPIO_FUNC[0];
                G2_FUNC_SAVE = GPIO_FUNC[2];
                GPIO_FUNC[0] = 5;       // BOOT
                GPIO_FUNC[2] = 5;       // BOOT
                KEY_MODE[SYS_KEY_INDEX] |= KEY_SYS_CTRL_Bit;
                KEY_MODE[USR_KEY_INDEX] |= KEY_SYS_CTRL_Bit;
                LED_MODE[SYS_LED_INDEX] |= LED_SYS_CTRL_Bit;
                LED_MODE[USR_LED_INDEX] |= LED_SYS_CTRL_Bit;
                // GPIO OUTPUT
                break;
            case ESP32_BOOT_NOW:
                // *APP_BOOT_State = ESP32_BOOT_END;
                break;
            case ESP32_BOOT_END:
                *APP_BOOT_State = IDLE_Mode;
                GPIO_FUNC[0] = G0_FUNC_SAVE;
                GPIO_FUNC[0] = G2_FUNC_SAVE;
                KEY_MODE[SYS_KEY_INDEX] &= ~KEY_SYS_CTRL_Bit;
                KEY_MODE[USR_KEY_INDEX] &= ~KEY_SYS_CTRL_Bit;
                LED_MODE[SYS_LED_INDEX] &= ~LED_SYS_CTRL_Bit;
                LED_MODE[USR_LED_INDEX] &= ~LED_SYS_CTRL_Bit;
                break;
        
            default:
                break;
        }
    }
}

// ��ESP32���KEY�źţ���SYSǿ��״̬��ִ�У�/BOOT�źţ�SYSǿ��BOOT״̬��ִ�У�
void GPIO_DATA_OUTPUTING(void)
{
    uint8_t i, j;
    GPIOStateTpDef TEMP_IN_STATE, TEMP_OUT_STATE;
    uint8_t SYS_KEY_EN = 1;
    for (i = 0; i < 2; i++){
        // i = 0: SYS; i = 1: USR;
        if (!(KEY_MODE[i] & KEY_SYS_CTRL_Bit)){
            // ִ��ϵͳ�����ȼ�����ʱ����BOOT����ͣ���KEY�ź�
            if (KEY_MODE[i] & KEY_NEW_DATA_Bit){
            // �����������źŴ���
                KEY_MODE[i] &= ~KEY_NEW_DATA_Bit;   // ��λ
                if ((KEY_MODE[i] & KEY_UP_DOWN_Bit) || (KEY_MODE[i] & KEY_SOFT_UP_DOWN_Bit)){
                    // HW PRESS OR SW PRESS
                    TEMP_IN_STATE = GPIO_HIGH;
                } else {
                    TEMP_IN_STATE = GPIO_LOW;
                }
                // ��UART���
                if (KEY_MODE[i] & KEY_UART_OUT_EN_Bit){
                    // OUTPUT to UART
                    if(i == 0){
                        // SYS KEY ������
                        SYS_KEY_EN = 0;
                    }
                }
                // ��ת���
                if (KEY_MODE[i] & KEY_INVER_OUT_Bit){
                    if (TEMP_IN_STATE == GPIO_LOW){
                        TEMP_OUT_STATE = GPIO_HIGH;
                    } else if (TEMP_IN_STATE == GPIO_HIGH){
                        TEMP_OUT_STATE = GPIO_LOW;
                    } else {
                        // �������ȼ����ڷ�ת���ȼ�
                        // TEMP_OUT_STATE = GPIO_INVER;    // LOCK
                    }
                }
                // �������� �������ȼ����ڷ�ת���ȼ�
                if (KEY_MODE[i] & KEY_LOCK_EN_Bit){
                    if (TEMP_IN_STATE == GPIO_LOW){
                        TEMP_OUT_STATE = GPIO_INVER;
                    }
                }
                // �������
                if (KEY_MODE[i] & KEY_STATE_EN_Bit){
                }

                // OUT to GPIO
                for (j = 0; j < GPIO_INDEX_MAX; j++){
                    if (GPIO_FUNC[j] == (i + 3)){
                        Pin_Ctrl(GPIO_Index[j], TEMP_OUT_STATE);
                        if(i == 0){
                            // SYS KEY ������
                            SYS_KEY_EN = 0;
                        }
                    }
                }
                // SYS KEY Default to EN
                if(i == 0){
                    if(SYS_KEY_EN){
                        // SYS KEY δ�����κ���;
                        Pin_Ctrl(BOOT_EN_Pin, TEMP_IN_STATE);
                    }
                }
                // USER KEY Default to GPIO0 
                if(i == 1){
                    // USR KEY
                    if((GPIO_FUNC[0] == 0)){        // == 5 BOOT
                        Pin_Ctrl(GPIO_Index[0], TEMP_IN_STATE);
                    }
                }
            }
        }
    }

    // BOOT
    if (KEY_MODE[SYS_KEY_INDEX] & KEY_SYS_CTRL_Bit){
        switch(*APP_BOOT_State){
            case ESP32_BOOT_PREPARE:
                Pin_Ctrl(BOOT_EN_Pin, GPIO_LOW);
                Pin_Ctrl(BOOT_G0_Pin, GPIO_LOW);
                Pin_Ctrl(BOOT_G2_Pin, GPIO_LOW);
                *APP_BOOT_State = ESP32_BOOT_NOW;
            break;
            case ESP32_BOOT_NOW:
                Pin_Ctrl(BOOT_EN_Pin, GPIO_HIGH);
                *APP_BOOT_State = ESP32_BOOT_END;
            break;
            // case ESP32_BOOT_END:
            //     *APP_BOOT_State = IDLE_Mode;
            //     break;
            case ESP32_Reset_NOW:
                *APP_BOOT_State = ESP32_Reset_END;
                Pin_Ctrl(BOOT_EN_Pin, GPIO_LOW);
                break;
            case ESP32_Reset_END:
                *APP_BOOT_State = IDLE_Mode;
                Pin_Ctrl(BOOT_EN_Pin, GPIO_HIGH);
                break;
            
            default:
                break;
        }
    }
}

// ��ʵ�ķ�ӦLED_MODE
void LED_OUTPUTING(void)
{
    static uint8_t LED_OUTPUT_Count[2] = {0};
    static uint8_t LED_BLINK_Count[2] = {0};
    GPIOStateTpDef TEMP_OUT_STATE;
    uint8_t i;
    for (i = 0; i < 2; i++){
        if (LED_MODE[i] & LED_BLINK_EN_Bit)
        {
            LED_OUTPUT_Count++;
            if (LED_OUTPUT_Count >= 5)
            { // 100ms
                TEMP_OUT_STATE = GPIO_INVER;
            }
        }
        if (LED_MODE[i] & LED_BLINK_Times_Bit){
            LED_BLINK_Count++;
            if(LED_BLINK_Count >= 4){   // 2�κ��Զ��ر�
                LED_MODE[i] &= ~LED_BLINK_EN_Bit;
                LED_MODE[i] &= ~LED_BLINK_Times_Bit;
            }
        }
        // ������ȼ���LEDʹ��
        if (!(LED_MODE[i] & LED_ON_OFF_Bit)){
            TEMP_OUT_STATE = GPIO_LOW
        } else if (TEMP_OUT_STATE != GPIO_INVER) {
            TEMP_OUT_STATE = GPIO_HIGH;
        }
    }
        
    // OUTPUT
    if(i == 0){
        // SYS LED
        Pin_Ctrl(Sys_LED_Pin, TEMP_OUT_STATE);
    }else if(i == 1){
        // USER LED
        Pin_Ctrl(User_LED_Pin, TEMP_OUT_STATE);
    }
}

uint16_t App_ProcessEvent(uint8 task_id, uint16 events)
{
    if(events & APP_Test_EVT)
    {
        DEBUG_PIN(A10_Pin, GPIO_INVER);
        return (events ^ APP_Test_EVT);
    }
    if(events & GPIO_PROCESS_EVT)
    {
        BOOT_Execute_GPIO_MODE();
        GPIO_Mode_SETTING();
        GPIO_GETTING();
        KEY_STATE_GETTING();
        SYS_STATE_to_SYS_LED();
        GPIO_DATA_OUTPUTING();
        LED_OUTPUTING();
        return (events ^ GPIO_PROCESS_EVT);
    }
    if(events & KEY_SHAKE_EVT)
    {
        return (events ^ KEY_SHAKE_EVT);
    }
    return 0;
}

void task_list_fifo_init()
{
    app_drv_fifo_init(&task_list_fifo, task_list, Task_List_LENGTH);
}

void All_FIFO_Init(void)
{
    task_list_fifo_init();
    ble_fifo_init();
//    usb_fifo_init();
    uart_fifo_init();
}

// #define UART0_Start_TX_from_USB_EVT     0x0008
// #define UART0_Start_TX_from_USB_SYB     1
// app_drv_fifo_write_from_same_addr(&task_list_fifo, &TEMP_Task_SYB, 1);  // �ж�������TMOS task�����ⷽʽ
void Task_Launcher(void)
{
    uint8_t Task_Symbol;
    while(!app_drv_fifo_is_empty(&task_list_fifo))
    {
        app_drv_fifo_read_to_same_addr(&task_list_fifo, &Task_Symbol, 1);
        switch(Task_Symbol)
        {
            // case UART0_Start_TX_from_USB_SYB: {
            //     tmos_set_event(UART_TaskID, UART0_Start_TX_from_USB_EVT);
            // }   break;
            default:
                break;
        }
    }
}

void APP_task_Init(void)
{
    APP_TaskID = TMOS_ProcessEventRegister(App_ProcessEvent);
    tmos_start_reload_task(APP_TaskID, GPIO_PROCESS_EVT, 20);    // 12.5ms
}

void BOOT_CTRL_LINE_Check(uint8_t COM_CTRL_LINE_STATE)
{
  if((COM_CTRL_LINE_STATE & COM_DTR_NEW_Edge) || (COM_CTRL_LINE_STATE & COM_RTS_NEW_Edge))
  { // ǰ�᣺����������
    // UART1_SendByte(COM_CTRL_LINE_STATE);
    switch(COM_CTRL_LINE_STATE)
    {
      case ESP32_DLD_STATE:
        // ��¼�źţ����һ���ж�
        *APP_BOOT_State = ESP32_UART_Check;
        // 500ms�ڼ�ⲻ�����ʵĴ�����������Ϊ�鱨�ǼٵΣ��˳�UART_Check״̬
        // �ж��ڲ��ܵ���tmos ������Ⱥ���
        Timer_Task_Start(500, Timer_Task_RST_BOOT_State);
        break;
      // case ESP_Tool_CTRL_STATE:
      //   *APP_BOOT_State = ESP32_UART_Check;
      //   break;
      case ESP32_DLDEND_STATE:
        // ��¼����ź�
        *APP_BOOT_State = ESP32_Reset_NOW;
        SYS_STATE_BITMAP &= ~SYS_UART_FLASHING_STATE_Bit;
        break;
      case ESP32_RST_STATE:
        // ����������ź�
        *APP_BOOT_State = ESP32_Reset_NOW;
        break;
    //   case Arduino_DLD_STATE:
    //     // Arduino��¼�ź� / ���ڴ��ź�
    //     *APP_BOOT_State = Arduino_UART_Check;
    //     break;
      default:
        break;
    }
  }
  else
  {
    // if(!(COM_CTRL_LINE_STATE == ESP_Tool_CTRL_LINE))
    *APP_BOOT_State = IDLE_Mode;
  }
}

void BOOT_Serial_Chack(uint8_t First_bit)
{
    if(*APP_BOOT_State == ESP32_UART_Check)
    {
        if(R8_USB_RX_LEN == ESP32_BOOT_LEN)
        {
            if(First_bit == ESP32_BOOT_F_Bit) 
            {
                *APP_BOOT_State = ESP32_BOOT_PREPARE;
                SYS_STATE_BITMAP |= SYS_UART_FLASHING_STATE_Bit;
                Timer_Task_Stop();
            }
        }
    }
    // else if(*APP_BOOT_State == Arduino_UART_Check)
    // {
    //     if(R8_USB_RX_LEN == Arduino_BOOT_LEN)
    //     {
    //         if(First_bit == Arduino_BOOT_F_Bit) *APP_BOOT_State = Arduino_BOOT_NOW;
    //     }
    // }
    // else if(*APP_BOOT_State == SSCOM_UART_Check)
    // {
    //     if(R8_USB_RX_LEN == SSCOM_BOOT_LEN)
    //     {
    //         if(First_bit == SSCOM_BOOT_F_Bit) *APP_BOOT_State = STC_BOOT_NOW;
    //     }
    // }
}

/*******************************************************************************
 * @fn      TMR2_IRQHandler
 * @brief   TMR2�жϺ���
 * @return  None
*******************************************************************************/
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void TMR2_IRQHandler(void) // TMR2 ��ʱ�ж�
{
    TMR2_ClearITFlag(TMR0_3_IT_CYC_END); // ����жϱ�־
    switch (Timer_Task_ID)
    {
    case Timer_Task_RST_BOOT_State:
        *APP_BOOT_State = IDLE_Mode;
        break;
    
    default:
        break;
    }
    Timer_Task_ID = 0;
    TMR2_ITCfg(DISABLE, TMR0_3_IT_CYC_END); // �ر��ж�
}
