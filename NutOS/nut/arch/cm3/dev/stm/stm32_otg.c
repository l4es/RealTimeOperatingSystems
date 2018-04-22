/*
 * Copyright (C) 2010 by Nikolaj Zamotaev
 * Copyright (C) 2010 by Ulrich Prinz (uprinz2@netscape.net)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

/*
 * \verbatim
 * $Id: stm32_otg.c 5472 2013-12-06 00:16:28Z olereinhardt $
 * \endverbatim
 */

#include <cfg/os.h>
#include <cfg/clock.h>

#include <sys/device.h>
#include <sys/timer.h>
#include <string.h>
#include <malloc.h>
#include <dev/irqreg.h>
#include <dev/gpio.h>

#include <sys/event.h>
#include <arch/cm3.h>
#include <arch/cm3/stm/stm32_gpio.h>
#include <dev/usb_stm32/stm32_otg.h>
#include <dev/usb_stm32/hw_config.h>
#include <dev/usb_stm32/usb_istr.h>
#include <dev/usb_stm32/usb_init.h>
#include <dev/usb_stm32/usb_prop.h>
#include <dev/usb_stm32/usb_pwr.h>
#include <dev/usb_stm32/usb_sil.h>
#include <dev/usb_stm32/usb_desc.h>
#include <dev/usb_stm32/otgd_fs_dev.h>
#include <dev/usb_stm32/otgd_fs_cal.h>
#include <dev/usb_stm32/otgd_fs_pcd.h>

#define USART_TXBUFSIZ    0
#define USART_TXHIWMARK   0
#define USART_TXLOWMARK   0
#define USART_RXBUFSIZ    256
#define USART_RXHIWMARK   190
#define USART_RXLOWMARK   128

#define TX_BUF_SIZE       64

#define MIN(a,b)    ((a) < (b) ? (a) : (b))

#include <dev/usart.h>
//#define static

uint8_t buffer_out[VIRTUAL_COM_PORT_DATA_SIZE];
extern USB_OTG_PCD_DEV USB_OTG_PCD_dev;

int rx_errors=0;

extern LINE_CODING linecoding;

static RINGBUF* rx_buffer;
static uint8_t tx_buffer[ TX_BUF_SIZE ];

//how many bytes stored in tx_buffer
static volatile uint8_t  tx_buf_cnt = 0;

//"ready for transmit" flag
static volatile uint8_t tx_rdy = 1;

static HANDLE OTGTimer = 0;
static HANDLE OTGEvent = 0;

void Stm32Otg_IRQHandler(void* arg){
    STM32_PCD_OTG_ISR_Handler();
};

static int Stm32OtgUsartInit(void);

static int Stm32OtgUsartDeinit(void)
{
    NVIC_DisableIRQ(OTG_FS_IRQn);
    return 0;
};

static int Stm32OtgUsartSetSpeed(uint32_t rate){//Работает
    linecoding.bitrate=rate;
    return 0;
};

static uint32_t Stm32OtgUsartGetSpeed(void){//Работает
    return linecoding.bitrate;
};

static int Stm32OtgUsartSetDataBits(uint8_t bits){//Должно работать
    linecoding.datatype=bits;
    return 0;
};

static uint8_t Stm32OtgUsartGetDataBits(void){//Должно работать
    return linecoding.datatype;
};

static int Stm32OtgUsartSetFlowControl(uint32_t flags){//FIXME: доделать?
    return 0;
};

static uint32_t Stm32OtgUsartGetFlowControl(void){//FIXME: доделать?
    return 0;
};

static int Stm32OtgUsartSetClockMode(uint8_t mode){//не нужно - синхронный режим неприменим
    return -1;
};

static uint8_t Stm32OtgUsartGetClockMode(void){//работает
    return 0;
};

static int Stm32OtgUsartSetParity(uint8_t mode){
    return 0;
};

static uint8_t Stm32OtgUsartGetParity(void){
    return 0;
};

static int Stm32OtgUsartSetStopBits(uint8_t bits){//Скорее всего правильно
    linecoding.format=bits-1;
    return 0;
};

static uint8_t Stm32OtgUsartGetStopBits(void){//Скорее всего правильно
    return linecoding.format+1;
};

static uint32_t Stm32OtgUsartGetStatus(void){
    return UART_RXENABLED|UART_TXENABLED;
};

static int Stm32OtgUsartSetStatus(uint32_t flags){
    return 0;
};

static void Stm32OtgUsartTxStart(void){
    //Здесь начать передачу по USB
//  EP1_IN_Callback();
};


static void FlushTxBuffer(void* arg)
{
    if(OTGTimer)  {
    NutTimerStop(OTGTimer);
    OTGTimer = 0;
    }
    tx_rdy = 0;
    OTGD_FS_PCD_EP_Write (EP1_IN,tx_buffer,tx_buf_cnt);
    tx_buf_cnt = 0;
}

void OTGTimerCallback(HANDLE timer, void *arg)
{
    NutEventPostAsync(arg);
}

THREAD(OTGTimerEvent, arg)
{
    NutThreadSetPriority(4);
    for (;;) {
    if (NutEventWait(arg, 0) == 0){
        FlushTxBuffer(NULL);
        }
    }
}

static int Stm32OtgWrite(NUTFILE * fp, const void *buffer, int len)
{
    int c = len;
    uint8_t *cp = (uint8_t *) buffer;
    while(c>0){

        //waiting for "ready-for-transmit" flag
        while( !tx_rdy ) {
          NutThreadYield( );
        }

        if(OTGTimer)  {
        NutTimerStop(OTGTimer);
        OTGTimer = 0;
    }
        int cnt = MIN( TX_BUF_SIZE - tx_buf_cnt, c );
        memcpy( &tx_buffer[ tx_buf_cnt ], cp, cnt );
        tx_buf_cnt += cnt;
        c -= cnt;
        cp += cnt;

        //try to flush tx buffer
        if( tx_buf_cnt == TX_BUF_SIZE ) {
            FlushTxBuffer( NULL );
        } else {
            //reset timer counter
            OTGTimer = NutTimerStart(
        16, OTGTimerCallback, &OTGEvent, TM_ONESHOT);
       }
    }
    return len;
}


static void Stm32OtgUsartRxStart(void){
    //Здесь запустить приём данных по нужному EP
      USB_OTG_EP *ep;
      ep = OTGD_FS_PCD_GetOutEP(EP3_OUT & 0x7F);
      /*setup and start the Xfer */
      ep->xfer_buff = buffer_out;
      ep->xfer_len = VIRTUAL_COM_PORT_DATA_SIZE;
      ep->xfer_count = 0;
      ep->is_in = 0;
      ep->num = EP3_OUT & 0x7F;
      if (USB_OTG_PCD_dev.ep0state == 0)
      {
        OTGD_FS_EPStartXfer( ep );
      }
};

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void)
{
    //NutEventPostFromIrq(&tx_handle);
        tx_rdy = 1;
}

/*******************************************************************************
* Function Name  : EP3_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
    //приём данных - данные пришли - по сути RxReady
    register size_t cnt;
        uint32_t i = 0;
        USB_OTG_EP *ep;
        /* Get the structure pointer of the selected Endpoint */
        ep = OTGD_FS_PCD_GetOutEP(EP3_OUT);
        /* Use the PCD interface layer function to read the selected endpoint */
    /* copy received data into application buffer */
    cnt = rx_buffer->rbf_cnt;
//  if (cnt >= rx_buffer->rbf_siz) {
        //  rx_errors |= UART_OVERRUNERROR;
        //FIXME: somehow signall overflow
//          return;
//  }
    for (i = 0 ; i < ep->xfer_len ; i++)
    {
        if (cnt++ == 0){
                NutEventPostFromIrq(&rx_buffer->rbf_que);
        }
        *rx_buffer->rbf_head++ = ep->xfer_buff[i];
        if (rx_buffer->rbf_head == rx_buffer->rbf_last) {
                rx_buffer->rbf_head = rx_buffer->rbf_start;
        }
        /* Update the ring buffer counter. */
        rx_buffer->rbf_cnt = cnt;
    }
}



static USARTDCB dcb_otg0 = {
    0,                          /* dcb_modeflags */
    0,                          /* dcb_statusflags */
    0,                          /* dcb_rtimeout */
    0,                          /* dcb_wtimeout */
    {0, 0, 0, 0, 0, 0, 0, 0},   /* dcb_tx_rbf */
    {0, 0, 0, 0, 0, 0, 0, 0},   /* dcb_rx_rbf */
    0,                          /* dbc_last_eol */
    Stm32OtgUsartInit,              /* dcb_init */
    Stm32OtgUsartDeinit,            /* dcb_deinit */
    Stm32OtgUsartTxStart,           /* dcb_tx_start */
    Stm32OtgUsartRxStart,           /* dcb_rx_start */
    Stm32OtgUsartSetFlowControl,    /* dcb_set_flow_control */
    Stm32OtgUsartGetFlowControl,    /* dcb_get_flow_control */
    Stm32OtgUsartSetSpeed,          /* dcb_set_speed */
    Stm32OtgUsartGetSpeed,          /* dcb_get_speed */
    Stm32OtgUsartSetDataBits,       /* dcb_set_data_bits */
    Stm32OtgUsartGetDataBits,       /* dcb_get_data_bits */
    Stm32OtgUsartSetParity,         /* dcb_set_parity */
    Stm32OtgUsartGetParity,         /* dcb_get_parity */
    Stm32OtgUsartSetStopBits,       /* dcb_set_stop_bits */
    Stm32OtgUsartGetStopBits,       /* dcb_get_stop_bits */
    Stm32OtgUsartSetStatus,         /* dcb_set_status */
    Stm32OtgUsartGetStatus,         /* dcb_get_status */
    Stm32OtgUsartSetClockMode,      /* dcb_set_clock_mode */
    Stm32OtgUsartGetClockMode,      /* dcb_get_clock_mode */
};

/*!
 * \brief Initialize debug device 2.
 *
 * \return Always 0.
 */
static int Stm32OtgUsartInit( void )
{
    //OTG initialization
#if defined(MCU_STM32F2) || defined(MCU_STM32F4)

    GpioPinConfigSet(NUTGPIO_PORTA, 9, GPIO_CFG_OUTPUT|GPIO_CFG_PERIPHAL);
    GpioPinConfigSet(NUTGPIO_PORTA, 10, GPIO_CFG_OUTPUT|GPIO_CFG_PULLUP|GPIO_CFG_MULTIDRIVE|GPIO_CFG_PERIPHAL);
    GpioPinConfigSet(NUTGPIO_PORTA, 11, GPIO_CFG_OUTPUT|GPIO_CFG_PERIPHAL);
    GpioPinConfigSet(NUTGPIO_PORTA, 12, GPIO_CFG_OUTPUT|GPIO_CFG_PERIPHAL);

    GPIO_PinAFConfig(GPIOA,9,GPIO_AF_OTG_FS) ;
    GPIO_PinAFConfig(GPIOA,11,GPIO_AF_OTG_FS) ;
    GPIO_PinAFConfig(GPIOA,12,GPIO_AF_OTG_FS) ;
    GPIO_PinAFConfig(GPIOA,10,GPIO_AF_OTG_FS) ;
#endif

    Set_USBClock();
    rx_buffer=&(dcb_otg0.dcb_rx_rbf);
    //tx_buffer=malloc( TX_BUF_SIZE );
    tx_buf_cnt=0;
    //Register interrupts
    Cortex_RegisterInt(OTG_FS_IRQn, Stm32Otg_IRQHandler);
    NVIC_EnableIRQ(OTG_FS_IRQn);
    USB_Init();

    NutThreadCreate("otgt", OTGTimerEvent, &OTGEvent, 256);

    return 0;
}

/*!
 * \brief Stm32Otg device information structure.
 */
NUTDEVICE devStm32Otg = {
    0,                                  /*!< Pointer to next device, dev_next. */
    {'u', 's', 'b', '_', 'o', 't', 'g', 0, 0},  /*!< Unique device name, dev_name. */
    IFTYP_CHAR,                         /*!< Type of device, dev_type. */
    0,                                  /*!< Base address, dev_base. */
    0,                                  /*!< First interrupt number, dev_irq. */
    0,                                  /*!< Interface control block, dev_icb. */
    &dcb_otg0,                          /*!< Driver control block, dev_dcb. */
    UsartInit,                          /*!< Driver initialization routine, dev_init. */
    UsartIOCtl,                         /*!< Driver specific control function, dev_ioctl. */
    UsartRead,                          /*!< dev_read. */
    Stm32OtgWrite,                      /*!< dev_write. */
    UsartOpen,                          /*!< dev_opem. */
    UsartClose,                         /*!< dev_close. */
    UsartSize,                          /*!< dev_size. */
    UsartSelect,                        /*!< dev_select. */
};

