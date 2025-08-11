/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Double-sided two-way ranging (DS TWR) responder example code
 *
 *           This is a simple code example which acts as the responder in a DS TWR distance measurement exchange. This application waits for a "poll"
 *           message (recording the RX time-stamp of the poll) expected from the "DS TWR initiator" example code (companion to this application), and
 *           then sends a "response" message recording its TX time-stamp, after which it waits for a "final" message from the initiator to complete
 *           the exchange. The final message contains the remote initiator's time-stamps of poll TX, response RX and final TX. With this data and the
 *           local time-stamps, (of poll RX, response TX and final RX), this example application works out a value for the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */



/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "lcd.h"
#include "port.h"
#include "lcd_oled.h"
#include "trilateration.h"
#include <math.h>
#include "kalman.h"
#include "AT24C02.h"
#include "stm32_eval.h"
#include "lib.h"

extern void newRange(void);


extern char dist_str[16];
///extern uint8_t TAG_ID;
///extern uint8_t MASTER_TAG;
///extern uint8_t SLAVE_TAG_START_INDEX;
///extern uint8_t ANCHOR_IND; 
////extern uint8_t ANCHOR_IND; 
extern uint8 Semaphore[MAX_SLAVE_TAG];

vec3d AnchorList[ANCHOR_MAX_NUM];
vec3d tag_best_solution;
int Anthordistance[ANCHOR_MAX_NUM];

int Anthordistance_count[ANCHOR_MAX_NUM];

int ANCHOR_REFRESH_COUNT_set=5;
#define ANCHOR_REFRESH_COUNT ANCHOR_REFRESH_COUNT_set


/* Private macro ---------- ---------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//==============================================================================
// printf() 출력으로 사용할 uart초기화 후 아래 코드 추가 
/************ 프로젝트 Option Target Use MicroLIB 체크해야함 *******************/
//==============================================================================
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */ 
//==============================================================================


void dwt_dumpregisters(char *str, size_t strSize)
{
    uint32 reg = 0;
    uint8 buff[5];
    int i;
    int cnt ;

#if (0)
    //first print all single registers
    for(i=0; i<0x3F; i++)
    {
        dwt_readfromdevice(i, 0, 5, buff) ;
        str += cnt = sprintf(str,"reg[%02X]=%02X%02X%02X%02X%02X",i,buff[4], buff[3], buff[2], buff[1], buff[0] ) ;
        str += cnt = sprintf(str,"\n") ;
    }
    // test_1
    //reg 0x20
    for(i=0; i<=32; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x20,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x20,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x21
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x21,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x21,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x23
    for(i=0; i<=0x20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x23,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x23,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#else
    //reg 0x24
    for(i=0; i<=2; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x24,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x24,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x27
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x27,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x27,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x28
    for(i=0; i<=64; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x28,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x28,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2A
    for(i=0; i<20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2A,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2A,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2B
    for(i=0; i<24; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2B,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2B,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2f
    for(i=0; i<40; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2f,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2f,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x31
    for(i=0; i<84; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x31,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x31,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x36 = PMSC_ID
    for(i=0; i<=48; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x36,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x36,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#endif
}

void Anchor_Array_Init(void)
{
    int anchor_index = 0;
    for(anchor_index = 0; anchor_index < ANCHOR_MAX_NUM; anchor_index++)
    {
        Anthordistance[anchor_index] = 0;
        Anthordistance_count[anchor_index] = 0;
    }
}
void Semaphore_Init(void)
{
    int tag_index = 0 ;
    for(tag_index = 0; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        Semaphore[tag_index]  = 0;
    }
}

int Sum_Tag_Semaphore_request(void)
{
    int tag_index = 0 ;
    int sum_request = 0;
    for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        sum_request+=Semaphore[tag_index];
    }
    return sum_request;
}


void Tag_Measure_Dis(void)
{
    uint8 dest_anthor = 0,frame_len = 0;
    float final_distance = 0;
	uint32 tick1;
	frame_seq_nb=0;//change by johhn
    for(dest_anthor = 0 ;  dest_anthor<ANCHOR_MAX_NUM; dest_anthor++)
    {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        tx_poll_msg[ALL_MSG_TAG_IDX] = TAG_ID;	// 기지국은 TAG_ID가 포함된 태그 정보를 수신하고, 기지국이 해당 태그에 응답할 때 TAG_ID도 지정해야 하며, 일치하는 경우 TAG_ID만 처리됩니다.

        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);

        //GPIO_SetBits(GPIOA,GPIO_Pin_2);
        //TODO
        dwt_rxenable(0);// 나중에 추가되는 내용인데, 기본적으로 tx 이후에 rx가 자동으로 전환되어야 합니다. 그러나 현재 디버그에서는 자동으로 열리지 않는 것으로 확인되었으며, 여기서는 rx가 강제로 열리게 됩니다.
		tick1=portGetTickCount();
        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        {
			if((portGetTickCount() - tick1) > 350)
			{
				break;
			}
		};
        GPIO_SetBits(GPIOA,GPIO_Pin_1);

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)	// 감지 TAG_ID
                continue;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;

            /* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time;

                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 9 below. */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

                /* Write all timestamps in the final message. See NOTE 10 below. */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                /* Write and send final message. See NOTE 7 below. */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                tx_final_msg[ALL_MSG_TAG_IDX] = TAG_ID;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg), 0);

                //TODO maybe need longer time
                //dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS*2);
                dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );

                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { 
					if((portGetTickCount() - tick1) > 500)
					{
						break;//change by johhn
					}
				};

                /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                if (status_reg & SYS_STATUS_RXFCG)
                {
                    /* Clear good/fail RX frame event in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)
                        continue;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;

                    /*As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                    rx_buffer[ALL_MSG_SN_IDX] = 0;

                    if (memcmp(rx_buffer, distance_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                       // final_distance = rx_buffer[10] + (float)rx_buffer[11]/100;
                        Anthordistance[rx_buffer[12]] +=(rx_buffer[10]*1000 + rx_buffer[11]*10);
                        Anthordistance_count[rx_buffer[12]] ++;
                        {
                            int Anchor_Index = 0;
                            while(Anchor_Index < ANCHOR_MAX_NUM)
                            {
                                if(Anthordistance_count[Anchor_Index] >=ANCHOR_REFRESH_COUNT )
                                {
                                    distance_mange();
                                    Anchor_Index = 0;
									//clear all
                                    while(Anchor_Index < ANCHOR_MAX_NUM)
                                    {
                                        Anthordistance_count[Anchor_Index] = 0;
                                        Anthordistance[Anchor_Index] = 0;
										Anchor_Index++;
                                    }
                                    break;
                                }
								Anchor_Index++;
                            }
                        }
                    }
                }
                else
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            // sprintf(dist_str, "%08x",status_reg);
            // OLED_ShowString(0, 2,"           ");
            // OLED_ShowString(0, 2,dist_str);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
        /* Execute a delay between ranging exchanges. */
        // deca_sleep(RNG_DELAY_MS);
        frame_seq_nb++;
    }

}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
//   /*!< At this stage the microcontroller clock setting is already configured,
//        this is done through SystemInit() function which is called from startup
//        file (startup_stm32f10x_xx.s) before to branch to application main.
//        To reconfigure the default setting of SystemInit() function, refer to
//        system_stm32f10x.c file
//      */

double final_distance =  0;

int main(void)
{
	uint8 anthor_index = 0;
	uint8 tag_index = 0;
	uint8 Semaphore_Enable = 0 ;
	uint8 Waiting_TAG_Release_Semaphore = 0;
	uint8 frame_len = 0;
	int rx_ant_delay =32880;
	int index = 0 ;
	uint8 temp;
	int itemp;
	
	// 이거 추가해야 함(어셈블리파일에서 호출하나 안되는 보드 있음)
	SystemInit();
	
	/* Start with board specific hardware init. */
	peripherals_init();
	printf("hello dwm1000!\r\n");
	// dwt_dumpregisters();
	/* Reset and initialise DW1000.
	* For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation
	SPI rate can be increased for optimum
	* performance. */
	reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
	spi_set_rate_low();
	if(dwt_initialise(DWT_LOADUCODE) == -1)
	{
		printf("dwm1000 init fail!\r\n");
		OLED_ShowString(0,0,"INIT FAIL");
		while (1)
		{
			STM_EVAL_LEDOn(LED1);
			deca_sleep(100);
			STM_EVAL_LEDOff(LED1);
			deca_sleep(100);
		}
	}
	spi_set_rate_high();
	/* Configure DW1000. See NOTE 6 below. */
	dwt_configure(&config);
	dwt_setleds(1);
	/* Apply default antenna delay value. See NOTE 1 below. */
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);
	OLED_ShowString(0,0,"INIT PASS");
	printf("init pass!\r\n");
/*	
	{0.00, 0.00, 0.00},
	{0.95, 0.59, 0.00},
	{-0.01, 0.48, 0.63},
	{0.56, 0.22, 0.18}
*/
	
	AnchorList[0].x =0.00;
	AnchorList[0].y =0.00;
	AnchorList[0].z =0.00;
	
	AnchorList[1].x =0.95;
	AnchorList[1].y =0.59;
	AnchorList[1].z =0.00;
	
	AnchorList[2].x =-0.01;
	AnchorList[2].y =0.48;
	AnchorList[2].z =0.63;
	
	AnchorList[3].x =0.56;
	AnchorList[3].y =0.22;
	AnchorList[3].z =0.18;


///	int rx_ant_delay =32880;
///	int index = 0 ;
	
	
#ifdef ANTHOR
	Anchor_Array_Init();
	/* Loop forever initiating ranging exchanges. */
	sprintf(dist_str, "  - ANTHOR %X", ANCHOR_IND);
	OLED_ShowString(0,0,dist_str);
	
#ifdef ANC_KALMAN_F_ENABLE	
	KalMan_PramInit();
#endif

	while (1)
	{
		/* Clear reception timeout to start next ranging process. */
		dwt_setrxtimeout(0);
		/* Activate reception immediately. */
		dwt_rxenable(0);
		/* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
		{ };
		if (status_reg & SYS_STATUS_RXFCG)
		{
			/* Clear good RX frame event in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
			if (frame_len <= RX_BUFFER_LEN)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}
			/* Check that the frame is a poll sent by "DS TWR initiator" example.
			* As the sequence number field of the frame is not relevant, it is cleared to simplify the
			validation of the frame. */
			if(rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM != ANCHOR_IND) continue;
			
			anthor_index = rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM;
			tag_index = rx_buffer[ALL_MSG_TAG_IDX];
			rx_buffer[ALL_MSG_SN_IDX] = 0;
			rx_buffer[ALL_MSG_TAG_IDX] = 0;
			if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
			{
				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();
				/* Set expected delay and timeout for final message reception. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
				/* Write and send the response message. See NOTE 9 below.*/
				tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				tx_resp_msg[ALL_MSG_TAG_IDX] = tag_index;
				dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
				dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
				{ };
				if (status_reg & SYS_STATUS_RXFCG)
				{
					/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
					/* A frame has been received, read it into the local buffer. */
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
					if (frame_len <= RX_BUF_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}
					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if(tag_index != rx_buffer[ALL_MSG_TAG_IDX]) continue;
					
					rx_buffer[ALL_MSG_TAG_IDX] = 0;
					if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
					{
						uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
						uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
						double Ra, Rb, Da, Db;
						int64 tof_dtu;
						/* Retrieve response transmission and final reception timestamps. */
						resp_tx_ts = get_tx_timestamp_u64();
						final_rx_ts = get_rx_timestamp_u64();
						/* Get timestamps embedded in the final message. */
						final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);
						/* Compute time of flight. 32-bit subtractions give correct answers even if clock has
							wrapped. See NOTE 10 below. */
						poll_rx_ts_32 = (uint32)poll_rx_ts;
						resp_tx_ts_32 = (uint32)resp_tx_ts;
						final_rx_ts_32 = (uint32)final_rx_ts;
						Ra = (double)(resp_rx_ts - poll_tx_ts);
						Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
						Da = (double)(final_tx_ts - resp_rx_ts);
						Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
						tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
						tof = tof_dtu * DWT_TIME_UNITS;
						distance = tof * SPEED_OF_LIGHT;
						distance = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);// 거리 빼기 보정 계수
                        // sprintf(dist_str, "dis: %3.2f m", distance);
                        //printf("before kalman fliter Distance:%3.2f m\r\n",rx_buffer[12],final_distance);
#ifdef ANC_KALMAN_F_ENABLE	
	// kalman filter
	distance =  KalMan_Update(&distance);
#endif	
						//  sprintf(dist_str, "dis: %3.2f m", distance);
						//    printf("after kalman fliter Distance:%3.2f m\r\n",rx_buffer[12],final_distance);
                        // 계산 결과를 TAG로 보내기
						itemp = (int)(distance*100);
						distance_msg[10] = itemp/100;
                        // a=x;  // 정수 부분을 취하는 자동 유형 변환
						distance_msg[11] = itemp%100; 	// 100을 곱한 후 나머지 100을 취하여 소수점 이하 2자리를 구합니다.
						distance_msg[12] = anthor_index;
						distance_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
						distance_msg[ALL_MSG_TAG_IDX] = tag_index;
						dwt_writetxdata(sizeof(distance_msg), distance_msg, 0);
						dwt_writetxfctrl(sizeof(distance_msg), 0);
						/* Start transmission, indicating that a response is expected so that reception is
							enabled automatically after the frame is sent and the delay
							* set by dwt_setrxaftertxdelay() has elapsed. */
						dwt_starttx(DWT_START_TX_IMMEDIATE );
					}
				}
				else
				{
					/* Clear RX error events in the DW1000 status register. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
				}
			}
			else if (memcmp(rx_buffer, angle_msg, ALL_MSG_COMMON_LEN) == 0 && ANCHOR_IND == 0)
			{
				if(rx_buffer[LOCATION_FLAG_IDX] == 1)//location infomartion
				{
					rx_buffer[ALL_MSG_TAG_IDX] = tag_index;
					USART_puts(&rx_buffer[LOCATION_INFO_START_IDX],rx_buffer[LOCATION_INFO_LEN_IDX]);
				}
				else //follow car
				{
					putchar(rx_buffer[10]);
				}
			}
		}
		else
		{
			/* Clear RX error events in the DW1000 status register. */
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		}
	}
#endif

#ifdef TAG
	/* Set expected response's delay and timeout. See NOTE 4 and 5 below.
	* As this example only handles one incoming frame with always the same delay and timeout,
	those values can be set here once for all. */
	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
	if(TAG_ID == MASTER_TAG)
	{
		sprintf(dist_str, "- MASTER TAG %X", TAG_ID);
		OLED_ShowString(0,0,dist_str);
	}
	else
	{
		sprintf(dist_str, "- SLAVE TAG %X", TAG_ID);
		OLED_ShowString(0,0,dist_str);
	}
	if(TAG_ID == MASTER_TAG)
	{
		Semaphore_Enable = 1 ;
		Semaphore_Init();
		Waiting_TAG_Release_Semaphore = 0;
	}
	else
	{
		Semaphore_Enable = 0 ;
	}
	
	//Master TAG0
	while(1)
	{
		if(Semaphore_Enable == 1)
		{
			GPIO_ResetBits(GPIOA,GPIO_Pin_1);
			GPIO_ResetBits(GPIOA,GPIO_Pin_2);
			//send message to anthor,TAG<->ANTHOR
			Tag_Measure_Dis();			//measuer distance between tag and all anthor
			Semaphore_Enable = 0 ;
			if(TAG_ID != MASTER_TAG)
			{
				//send release semaphore to master tag
				Semaphore_Release[ALL_MSG_SN_IDX] = frame_seq_nb;
				Semaphore_Release[ALL_MSG_TAG_IDX] = TAG_ID;
				dwt_writetxdata(sizeof(Semaphore_Release), Semaphore_Release, 0);
				dwt_writetxfctrl(sizeof(Semaphore_Release), 0);
				dwt_starttx(DWT_START_TX_IMMEDIATE );
				while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
				{ };
				GPIO_SetBits(GPIOA,GPIO_Pin_2);
			}
		}
		
		if(TAG_ID == MASTER_TAG)//master tag
		{
			//statistics tag
			if(Sum_Tag_Semaphore_request() == 0)
			{
				for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
				{
					Tag_Statistics[ALL_MSG_SN_IDX] = 0;
					Tag_Statistics[ALL_MSG_TAG_IDX] = tag_index;
					dwt_writetxdata(sizeof(Tag_Statistics), Tag_Statistics, 0);
					dwt_writetxfctrl(sizeof(Tag_Statistics), 0);
					dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
					while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
					{ };
					if (status_reg & SYS_STATUS_RXFCG)
					{
						/* Clear good RX frame event and TX frame sent in the DW1000 status register. */
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
						/* A frame has been received, read it into the local buffer. */
						frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
						if (frame_len <= RX_BUF_LEN)
						{
							dwt_readrxdata(rx_buffer, frame_len, 0);
						}
						rx_buffer[ALL_MSG_SN_IDX] = 0;
						if(rx_buffer[ALL_MSG_TAG_IDX] == tag_index)
						{
							temp = rx_buffer[ALL_MSG_TAG_IDX] ;
							rx_buffer[ALL_MSG_TAG_IDX] =0;
							if (memcmp(rx_buffer, Tag_Statistics_response, ALL_MSG_COMMON_LEN) == 0)
							{
								Semaphore[temp] = 1;
								GPIO_SetBits(GPIOA,GPIO_Pin_2);
							}
						}
					}
					else
					{
						/* Clear RX error events in the DW1000 status register. */
						dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
						//GPIO_SetBits(GPIOA,GPIO_Pin_1);
					}
				}
				//print all the tags in network
				for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
				{
					if(Semaphore[tag_index] == 1)
					{
						// printf("Tag%d In NetWork!\r\n",tag_index);
					}
				}
			}
			//pick one tag ,send Semaphore message
			//release to specific tag(TAG ID)
			//master tag send release signal,and the specific tag send comfirm message
			if(Waiting_TAG_Release_Semaphore == 0 && Sum_Tag_Semaphore_request() != 0)
			{
				Semaphore[0] = 0;//slave tag must not use tag_id = 0x00!!
				for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
				{
					if(Semaphore[tag_index] == 1)
					{
						// printf("Release Semaphore to Tag%d!\r\n",tag_index);
						// dwt_setrxtimeout(0);
						dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
						dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
						Master_Release_Semaphore[ALL_MSG_SN_IDX] = 0;
						Master_Release_Semaphore[ALL_MSG_TAG_IDX] = tag_index;
						dwt_writetxdata(sizeof(Master_Release_Semaphore), Master_Release_Semaphore, 0);
						dwt_writetxfctrl(sizeof(Master_Release_Semaphore), 0);
						dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
						while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
						{ };
						if (status_reg & SYS_STATUS_RXFCG)
						{
							GPIO_SetBits(GPIOA,GPIO_Pin_1);
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
							frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
							if (frame_len <= RX_BUF_LEN)
							{
								dwt_readrxdata(rx_buffer, frame_len, 0);
							}
							rx_buffer[ALL_MSG_SN_IDX] = 0;
							if(rx_buffer[ALL_MSG_TAG_IDX] == tag_index)
							{
								rx_buffer[ALL_MSG_TAG_IDX] = 0;
								GPIO_SetBits(GPIOA,GPIO_Pin_3);
								// USART_puts(rx_buffer,frame_len);
								if (memcmp(rx_buffer, Master_Release_Semaphore_comfirm,	ALL_MSG_COMMON_LEN) == 0)
								{
									//if the tag recive a semaphore, wait release remaphore
									Waiting_TAG_Release_Semaphore ++;
									break;//only release one semphore once
								}
							}
						}
						else//the tag may leave net,clear semaphore
						{
							Semaphore[tag_index] = 0 ;
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
						}
					}
				}
			}
			if(Waiting_TAG_Release_Semaphore == 0 )
			{
				// GPIO_SetBits(GPIOA,GPIO_Pin_2);GPIO_SetBits(GPIOA,GPIO_Pin_1);
			}
			//Master tag waitting for specific tag Semaphore Release message
			if( Waiting_TAG_Release_Semaphore >0)
			{
				// printf("Waiting for Release Semaphore!\r\n");
				dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS*5);//about 10ms,need adjust!!
				dwt_rxenable(0);
				while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
				{ };
				if (status_reg & SYS_STATUS_RXFCG)
				{
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
					frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
					if (frame_len <= RX_BUFFER_LEN)
					{
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}
					rx_buffer[ALL_MSG_SN_IDX] = 0;
					temp=rx_buffer[ALL_MSG_TAG_IDX] ;
					rx_buffer[ALL_MSG_TAG_IDX] = 0;
					if (memcmp(rx_buffer, Semaphore_Release, ALL_MSG_COMMON_LEN) == 0)
					{
						if(Semaphore[temp] == 1)
						{
							Semaphore[temp] = 0 ;
							if(Waiting_TAG_Release_Semaphore > 0 )
							{
								Waiting_TAG_Release_Semaphore --;
							}
						}
					}
				}
				else
				{
					//maybe the tag leave network
					if(Waiting_TAG_Release_Semaphore > 0)
					{
						Waiting_TAG_Release_Semaphore--;
						Semaphore[tag_index] = 0 ;
					}
					/* Clear RX error events in the DW1000 status register. */
					dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
				}
			}
			// 가능한 문제는 TAG가 Semaphore를 수신한 후 이를 해제하지 않고 네트워크를 떠나서 마스터 TAG가 세마포어를 다시 가져오지 못하는 것입니다.
			//	이를 위해서는 타이머를 구현해야 합니다. 일정 시간이 지난 후에도 여전히 TAG가 그렇지 않은 경우 Semaphore를 수신하고 Semaphore를 해제하는 경우 강제로 취소해야 합니다.
			//if all tag have serviced by master tag
			//master tag can measure the distance
			if(Sum_Tag_Semaphore_request() == 0)
			{
				Semaphore_Enable = 1 ;
				Waiting_TAG_Release_Semaphore= 0;
			}
		}
		else //slave tags
		{
			// SLAVE TAG는 기본적으로 MASTER TAG가 통계 정보를 보내고 세마포어를 해제할 때까지 기다리면서 시작됩니다.
			dwt_setrxtimeout(0);
			dwt_rxenable(0);
			/* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
			while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
			{ };
			if (status_reg & SYS_STATUS_RXFCG)
			{
				/* Clear good RX frame event in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID,
				SYS_STATUS_RXFCG|SYS_STATUS_TXFRS);//clear rx & tx flag at the same time
				/* A frame has been received, read it into the local buffer. */
				frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
				if (frame_len <= RX_BUFFER_LEN)
				{
					dwt_readrxdata(rx_buffer, frame_len, 0);
				}
				rx_buffer[ALL_MSG_SN_IDX] = 0;
				if(rx_buffer[ALL_MSG_TAG_IDX] == TAG_ID)
				{
					rx_buffer[ALL_MSG_TAG_IDX] = 0;
					if (memcmp(rx_buffer, Tag_Statistics, ALL_MSG_COMMON_LEN) == 0)
					{
						//GPIO_SetBits(GPIOA,GPIO_Pin_3);
						Tag_Statistics_response[ALL_MSG_SN_IDX] = frame_seq_nb;
						Tag_Statistics_response[ALL_MSG_TAG_IDX] = TAG_ID;
						dwt_writetxdata(sizeof(Tag_Statistics_response), Tag_Statistics_response, 0);
						dwt_writetxfctrl(sizeof(Tag_Statistics_response), 0);
						dwt_starttx(DWT_START_TX_IMMEDIATE );
						GPIO_SetBits(GPIOA,GPIO_Pin_2);
					}
					if (memcmp(rx_buffer, Master_Release_Semaphore, ALL_MSG_COMMON_LEN) == 0)
					{
						Master_Release_Semaphore_comfirm[ALL_MSG_SN_IDX] = frame_seq_nb;
						Master_Release_Semaphore_comfirm[ALL_MSG_TAG_IDX] = TAG_ID;
						dwt_writetxdata(sizeof(Master_Release_Semaphore_comfirm),
						Master_Release_Semaphore_comfirm, 0);
						dwt_writetxfctrl(sizeof(Master_Release_Semaphore_comfirm), 0);
						dwt_starttx(DWT_START_TX_IMMEDIATE);
						while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
						{ };
						Semaphore_Enable = 1;
						GPIO_SetBits(GPIOA,GPIO_Pin_1);
					}
				}
			}
			else
			{
				/* Clear RX error events in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			}
		}
	}
#endif
}




#define Filter_N 5  //max filter use in this system
const int Filter_D_set=5;
#define Filter_D Filter_D_set  //each filter contain "Filter_D" data
int Value_Buf[Filter_N][Filter_D]= {0};
int filter_index[Filter_N] = {0};
int filter(int input, int fliter_idx )
{
    char count = 0;
    int sum = 0;
    if(input > 0)
    {
        Value_Buf[fliter_idx][filter_index[fliter_idx]++]=input;
        if(filter_index[fliter_idx] == Filter_D) filter_index[fliter_idx] = 0;

        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }
    else
    {
        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }

}

static void distance_mange(void)
{
	int j;	
	int Anchor_Index;
	
/*	
    // if(GetLocation(&tag_best_solution, 0, &(AnchorList[0]), &(Anthordistance[0])) != -1)
    // {
    // printf("Tag Location:x=%3.2fm y=%3.2fm z=%3.2fm\r\n",tag_best_solution.x,tag_best_solution.y,tag_best_solution.z);
    // sprintf(dist_str, "x:%3.2f y:%3.2f",tag_best_solution.x,tag_best_solution.y);
    // OLED_ShowString(0,0,dist_str);
    //  }
    //GetLocation2(&tag_best_solution, 0, &(AnchorList[0]), &(Anthordistance[0]));
    //Th_Location( &(AnchorList[0]), &(Anthordistance[0]));
    //Th_Location2( &(AnchorList[0]), &(Anthordistance[0]));
    // OLED_ShowString(0, 2,"pass");

    //printf("Count:%d %d %d \r\n",Anthordistance[0]/Anthordistance_count[0],Anthordistance[1]/Anthordistance_count[1],Anthordistance[2]/Anthordistance_count[2]);
   // Anthordistance[0] =filter((int)(Anthordistance[0]/Anthordistance_count[0]),0);
   // Anthordistance[1] =filter((int)(Anthordistance[1]/Anthordistance_count[1]),1);
   // Anthordistance[2] = filter((int)(Anthordistance[2]/Anthordistance_count[2]),2);
    //printf("Count:%d %d %d \r\n",Anthordistance_count[0],Anthordistance_count[1],Anthordistance_count[2]);
    //printf("Count:%d %d %d \r\n",Anthordistance[0],Anthordistance[1],Anthordistance[2]);
    //printf(" \r\n");
*/


    {
        Anchor_Index = 0;
        while(Anchor_Index < ANCHOR_MAX_NUM)
        {
            if(Anthordistance_count[Anchor_Index] > 0 )
            {
                Anthordistance[Anchor_Index] =filter((int)(Anthordistance[Anchor_Index]/Anthordistance_count[Anchor_Index]),Anchor_Index);
            }
			Anchor_Index++;
        }
    }

    compute_angle_send_to_anthor0(Anthordistance[0], Anthordistance[1],Anthordistance[2]);



	
	
	
	
	// 1. SDK 에서 제공한 위치 구하기
	if(GetLocation(&tag_best_solution, 0, &(AnchorList[0]), &(Anthordistance[0])) != -1)
///	if(GetLocation(&tag_best_solution, 1, &(AnchorList[0]), &(Anthordistance[0])) != -1)
    {
		printf("Tag Location:x=%3.2fm y=%3.2fm z=%3.2fm\r\n",tag_best_solution.x,tag_best_solution.y,tag_best_solution.z);
///		sprintf(dist_str, "x:%3.2f y:%3.2f",tag_best_solution.x,tag_best_solution.y);
		
		
		sprintf(dist_str, "Tag Location");
		OLED_ShowString(0,0,dist_str);
		sprintf(dist_str, "x=%3.2fm 0=%3.2fm",tag_best_solution.x, (float)Anthordistance[0]/1000);
		OLED_ShowString(0,2,dist_str);
		sprintf(dist_str, "y=%3.2fm 1=%3.2fm",tag_best_solution.y, (float)Anthordistance[1]/1000);
		OLED_ShowString(0,4,dist_str);
		sprintf(dist_str, "z=%3.2fm 2=%3.2fm",tag_best_solution.z, (float)Anthordistance[2]/1000);
		OLED_ShowString(0,6,dist_str);
	}


/*		
	GetLocation2(&tag_best_solution, 0, &(AnchorList[0]), &(Anthordistance[0]));
	Th_Location( &(AnchorList[0]), &(Anthordistance[0]));
	Th_Location2( &(AnchorList[0]), &(Anthordistance[0]));
///////////	OLED_ShowString(0, 2,"pass");

	printf("Count:%d %d %d \r\n",Anthordistance[0]/Anthordistance_count[0],Anthordistance[1]/Anthordistance_count[1],Anthordistance[2]/Anthordistance_count[2]);
	Anthordistance[0] =filter((int)(Anthordistance[0]/Anthordistance_count[0]),0);
	Anthordistance[1] =filter((int)(Anthordistance[1]/Anthordistance_count[1]),1);
	Anthordistance[2] = filter((int)(Anthordistance[2]/Anthordistance_count[2]),2);
	printf("Count:%d %d %d \r\n",Anthordistance_count[0],Anthordistance_count[1],Anthordistance_count[2]);
	printf("Count:%d %d %d \r\n",Anthordistance[0],Anthordistance[1],Anthordistance[2]);
	printf(" \r\n");
	
*/	

	
	// 2. 4A3D 소스에서 제공한 위치 구하기
////	newRange();
	
	
	
	
		for(j=0;j<ANCHOR_MAX_NUM;j++)
		{
			if(Anthordistance_count[j]>0)
			{
				 sprintf(dist_str, "an%d:%3.2fm",j,(float)Anthordistance[j]/1000);
					printf("%s\r\n",dist_str);
			}
		
		}
    if(Anthordistance_count[0]>0)
    {
        sprintf(dist_str, "  AN 0: %3.2fm  ", (float)Anthordistance[0]/1000);
				//printf("%s\r\n",dist_str);
///        OLED_ShowString(0, 2," 		   ");
//////////////        OLED_ShowString(0, 0,dist_str);
    }


    if(Anthordistance_count[1]>0)
    {
        sprintf(dist_str, "  AN 1: %3.2fm  ", (float)Anthordistance[1]/1000);
				//printf("%s\r\n",dist_str);
///        OLED_ShowString(0, 4,"		 ");
////////////        OLED_ShowString(0, 2,dist_str);
    }


    if(Anthordistance_count[2]>0)
    {
        sprintf(dist_str, "  AN 2: %3.2fm  ", (float)Anthordistance[2]/1000);
				//AnthordistanceBuff[0]=Anthordistance[0];
				//printf("an2:%s\r\n",dist_str);
			/*
			float get=(float)Anthordistance[2]/1000;
				if(get >0.98 && get<1.20)
				{
					OLED_ShowString(0, 6,"success");
				
				}		
				else
				{
				
					OLED_ShowString(0, 6,"fail");
				
				}*/
					
///        OLED_ShowString(0, 6,"");
 ////////////       OLED_ShowString(0, 4,dist_str);
    }
/*	
    if(Anthordistance_count[3]>0)
    {
        sprintf(dist_str, "  AN 3: %3.2fm  ", (float)Anthordistance[3]/1000);
				//printf("%s\r\n",dist_str);
///        OLED_ShowString(0, 4,"		 ");
 /////////       OLED_ShowString(0, 6,dist_str);
    }
*/	
    // printf("Distance:%d,   %d,    %d mm\r\n",(int)((float)Anthordistance[0]/Anthordistance_count[0]),(int)((float)Anthordistance[1]/Anthordistance_count[1]),(int)((float)Anthordistance[2]/Anthordistance_count[2]));
}





#define DISTANCE3 	0.27

//**************************************************************//
//distance1 anthor0 <--> TAG  mm
//distance2 anthor1 <--> TAG  mm
//distance3 anthor2 <--> TAG  mm
//**************************************************************//
static void compute_angle_send_to_anthor0(int distance1, int distance2,int distance3)
{
    static int framenum = 0 ;

#if 0 //compute angle for smartcar
    float dis3_constans = DISTANCE3;
    float cos = 0;
    float angle = 0 ;
    float dis1 = (float)distance1/1000; //m
    float dis2 = (float)distance2/1000;  //m

    if(dis1 + dis3_constans < dis2 || dis2+dis3_constans < dis1)
    {
        //printf("ERROR!\r\n");
        //return;
    }
    cos = (dis1*dis1 + dis3_constans* dis3_constans - dis2*dis2)/(2*dis1*dis3_constans);
    angle  = acos(cos)*180/3.1415926;
    printf("cos = %f, arccos = %f\r\n",cos,angle);
    sprintf(dist_str, "angle: %3.2f m", angle);
    OLED_ShowString(0, 6,"            ");
    OLED_ShowString(0, 6,dist_str);

    if(dis1 > 1)
    {
        if(angle > 110)
        {
            printf("turn right\r\n");
            angle_msg[10] = 'R';
        }
        else if(angle < 75)
        {
            printf("turn left\r\n");
            angle_msg[10] = 'L';
        }
        else
        {
            printf("forward\r\n");
            angle_msg[10] = 'F';
        }
    }
    else
    {
        printf("stay here\r\n");
        angle_msg[10] = 'S';
    }
    angle_msg[LOCATION_FLAG_IDX] = 0;

#else
    //location
    {
        uint8 len = 0;
        angle_msg[LOCATION_FLAG_IDX] = 1;

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 'm';
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 'r';

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 0x02;
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = TAG_ID;//TAG ID

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)(framenum&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((framenum>>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10 >>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance2/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance2/10 >>8)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance3/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance3/10 >>8)&0xFF);
		
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10)&0xFF);
       angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10 >>8)&0xFF);
		
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = '\n';
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = '\r';


        angle_msg[LOCATION_INFO_LEN_IDX] = len;
        //MAX LEN
        if(LOCATION_INFO_START_IDX + len -2 >ANGLE_MSG_MAX_LEN)
        {
            while(1);//toggle LED
        }
        //USART_puts((char*)dist_str,16);

    }
#endif
	
    //only anthor0 recive angle message
    angle_msg[ALL_MSG_SN_IDX] = framenum;
    angle_msg[ALL_MSG_TAG_IDX] = TAG_ID;

    dwt_writetxdata(sizeof(angle_msg), angle_msg, 0);
    dwt_writetxfctrl(sizeof(angle_msg), 0);

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE );
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
    { };

    framenum++;

}





/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for
 * both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void) 
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readtxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for
 * both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void) 
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}


/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the
 * timestamp fields of the final message, the least significant byte is at the
 * lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8* ts_field, uint32* ts) 
{
  int i;
  *ts = 0;
  for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
    *ts += ts_field[i] << (i * 8);
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}














#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* 헌SR셍닸포櫓돨TC깃羚 */

    USART_ClearFlag(EVAL_COM1,USART_FLAG_TC);
    /* e.g. write a character to the USART */
    USART_SendData(EVAL_COM1, (uint8_t) ch);	// 사용할 포트로 수정
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
    {}
    return ch;
}


/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 8. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()). It is also to be noted that, when using
 *    delayed send, the time set for transmission must be far enough in the future so that the DW1000 IC has the time to process and start the
 *    transmission of the frame at the wanted time. If the transmission command is issued too late compared to when the frame is supposed to be sent,
 *    this is indicated by an error code returned by dwt_starttx() API call. Here it is not tested, as the values of the delays between frames have
 *    been carefully defined to avoid this situation.
 * 10. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/

/**
  * @}
  */
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/


