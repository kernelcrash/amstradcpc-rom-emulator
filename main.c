#include <stdio.h>
#include <string.h>
#include "main.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "stm32f4_discovery_sdio_sd.h"

#include "ff.h"
#include "diskio.h"

GPIO_InitTypeDef  GPIO_InitStructure;

extern uint32_t *romdis;

extern volatile uint8_t *rom_base;
extern volatile uint8_t *disk_image;
extern volatile uint8_t *disk_select_port;
extern volatile uint8_t *disk_select_port_complement;

// Must be volatile to prevent optimiser doing stuff
extern volatile uint32_t main_thread_command;
extern volatile uint32_t main_thread_data;

extern volatile uint32_t fdc_track;
extern volatile uint32_t fdc_prev_track;
extern volatile uint32_t fdc_write_flush_count;

extern volatile uint8_t *track_buffer;
extern volatile uint8_t *disk_info_buffer;

extern volatile uint32_t *sector_ptrs;

extern volatile uint8_t *fdc_log_ptr;

#ifdef ENABLE_SEMIHOSTING
extern void initialise_monitor_handles(void);   /*rtt*/
#endif

FRESULT res;
FILINFO fno;
DIR dir;
FATFS fs32;
char* path;
UINT BytesRead;

#if _USE_LFN
    static char lfn[_MAX_LFN + 1];
        fno.lfname = lfn;
            fno.lfsize = sizeof lfn;
#endif

void delay_ms(const uint16_t ms)
{
   uint32_t i = ms * 27778;
   while (i-- > 0) {
      __asm volatile ("nop");
   }
}

void blink_pa6_pa7(void) {
        while(1) {
                GPIOA->ODR = 0x0040;
                delay_ms(500);
                GPIOA->ODR = 0x0080;
                delay_ms(500);
        }
}

enum sysclk_freq {
    SYSCLK_42_MHZ=0,
    SYSCLK_84_MHZ,
    SYSCLK_168_MHZ,
    SYSCLK_200_MHZ,
    SYSCLK_240_MHZ,
};
 
void rcc_set_frequency(enum sysclk_freq freq)
{
    int freqs[]   = {42, 84, 168, 200, 240};
 
    /* USB freqs: 42MHz, 42Mhz, 48MHz, 50MHz, 48MHz */
    int pll_div[] = {2, 4, 7, 10, 10}; 
 
    /* PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N */
    /* SYSCLK = PLL_VCO / PLL_P */
    /* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
    uint32_t PLL_P = 2;
    uint32_t PLL_N = freqs[freq] * 2;
    uint32_t PLL_M = (HSE_VALUE/1000000);
    uint32_t PLL_Q = pll_div[freq];
 
    RCC_DeInit();
 
    /* Enable HSE osscilator */
    RCC_HSEConfig(RCC_HSE_ON);
 
    if (RCC_WaitForHSEStartUp() == ERROR) {
        return;
    }
 
    /* Configure PLL clock M, N, P, and Q dividers */
    RCC_PLLConfig(RCC_PLLSource_HSE, PLL_M, PLL_N, PLL_P, PLL_Q);
 
    /* Enable PLL clock */
    RCC_PLLCmd(ENABLE);
 
    /* Wait until PLL clock is stable */
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);
 
    /* Set PLL_CLK as system clock source SYSCLK */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
 
    /* Set AHB clock divider */
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
 
    //FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

    /* Set APBx clock dividers */
    switch (freq) {
        /* Max freq APB1: 42MHz APB2: 84MHz */
        case SYSCLK_42_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div1); /* 42MHz */
            RCC_PCLK2Config(RCC_HCLK_Div1); /* 42MHz */
            break;
        case SYSCLK_84_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div2); /* 42MHz */
            RCC_PCLK2Config(RCC_HCLK_Div1); /* 84MHz */
            break;
        case SYSCLK_168_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div4); /* 42MHz */
            RCC_PCLK2Config(RCC_HCLK_Div2); /* 84MHz */
            break;
        case SYSCLK_200_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div4); /* 50MHz */
            RCC_PCLK2Config(RCC_HCLK_Div2); /* 100MHz */
            break;
        case SYSCLK_240_MHZ:
            RCC_PCLK1Config(RCC_HCLK_Div4); /* 60MHz */
            RCC_PCLK2Config(RCC_HCLK_Div2); /* 120MHz */
            break;
    }
 
    /* Update SystemCoreClock variable */
    SystemCoreClockUpdate();
}

void SD_NVIC_Configuration(void)
{
        NVIC_InitTypeDef NVIC_InitStructure;

        /* Configure the NVIC Preemption Priority Bits */
        //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
        //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

        NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
        //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;    // This must be a lower priority (ie. higher number) than the _MREQ and _IORQ interrupts
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}

void SDIO_IRQHandler(void)
{
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
}

// _IORQ interrupt
void config_PC1_int(void) {
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;

        /* Enable clock for SYSCFG */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        /* Tell system that you will use PC2 for EXTI_Line2 */
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1);

        /* PC2 is connected to EXTI_Line2 */
        EXTI_InitStruct.EXTI_Line = EXTI_Line1;
        /* Enable interrupt */
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        /* Interrupt mode */
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        /* Triggers on rising and falling edge */
        //EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        /* Add to EXTI */
        EXTI_Init(&EXTI_InitStruct);

        /* Add IRQ vector to NVIC */
        /* PC0 is connected to EXTI_Line0, which has EXTI1_IRQn vector */
        NVIC_InitStruct.NVIC_IRQChannel = EXTI1_IRQn;
        /* Set priority */
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
        /* Set sub priority */
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
        /* Enable interrupt */
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        /* Add to NVIC */
        NVIC_Init(&NVIC_InitStruct);
}



void config_PC4_int(void) {
        EXTI_InitTypeDef EXTI_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;

        /* Enable clock for SYSCFG */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);

        /* PC2 is connected to EXTI_Line2 */
        EXTI_InitStruct.EXTI_Line = EXTI_Line4;
        /* Enable interrupt */
        EXTI_InitStruct.EXTI_LineCmd = ENABLE;
        /* Interrupt mode */
        EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
        /* Triggers on rising and falling edge */
        EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
        //EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
        /* Add to EXTI */
        EXTI_Init(&EXTI_InitStruct);

        /* Add IRQ vector to NVIC */
        /* PC0 is connected to EXTI_Line0, which has EXTI4_IRQn vector */
        NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
        /* Set priority */
        NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x01;
        /* Set sub priority */
        NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
        /* Enable interrupt */
        NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
        /* Add to NVIC */
        NVIC_Init(&NVIC_InitStruct);
}


/* Input Signals GPIO pins on ROMEN -> PC2, IORQ -> PC1, WR -> PC0 */
/* Output Signals GPIO pins on ROMDIS -> PC3. need to make it open collector with a pullup */
/* SD card uses PC10, PC11, PC12 out and PC8 in */
void config_gpio_portc(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOC Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Configure GPIO Settings */
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |GPIO_Pin_4 |GPIO_Pin_8 ;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |GPIO_Pin_4 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3|  GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIOC->ODR = 0x0000;
}

/* Input/Output data GPIO pins on PD{8..15}. Also PD2 is used fo MOSI on the STM32F407VET6 board I have */
void config_gpio_data(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | 
		GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/* Input Address GPIO pins on PE{0..15} */
void config_gpio_addr(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOE Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = 
		GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
		GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | 
		GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | 
		GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/* Debug GPIO pins on PA0 */
void config_gpio_dbg(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,DISABLE);


	/* Configure GPIO Settings */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void config_backup_sram(void) {

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
        PWR_BackupAccessCmd(ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
        PWR_BackupRegulatorCmd(ENABLE);
}


FRESULT load_disk_info(FIL *fil,char *fname, uint8_t *buffer) {
        UINT BytesRead;
        FRESULT res;

#ifdef ENABLE_SEMIHOSTING_DISK_INFO
	printf("BEFORE fopen of %s\n",fname);
#endif
        res = f_open(fil, fname, FA_READ);

        if (res == FR_OK) {
                res = f_read(fil, buffer, 256, &BytesRead);
#ifdef ENABLE_SEMIHOSTING_DISK_INFO
		printf("AFTER f_read. Bytes read %d\n",BytesRead);
#endif
	}
	return res;
}

FRESULT load_track(FIL *fil, uint32_t track_number, uint8_t *track, uint8_t *disk_info) {
        UINT BytesRead;
        FRESULT res;

	uint32_t offset=0x100;
	uint32_t track_size;
	int i;

	if ((disk_info[0]=='E') && (disk_info[1]=='X')) {
	   // Must be EXTENDED CPC DSK
#ifdef ENABLE_SEMIHOSTING_LOAD_TRACK
	if (track_number == 0x14) {
	   printf("EXTENDED DISK\n");
	}
#endif
	   for (i=0;i<track_number;i++) {
	      offset+=(disk_info[0x34+i]<<8);
           }
	   track_size = (disk_info[0x34+i]<<8);
	} else {
	   track_size=0x1300;
	   offset += (track_size * track_number);
        }

	res = f_lseek(fil, offset);
#ifdef ENABLE_SEMIHOSTING_LOAD_TRACK
	if (track_number == 0x14) {
	   printf("AFTER f_lseek of %04x. res = %d\n",offset,res);
	}
#endif
	track_size = (track_size > MAX_TRACK_SIZE)? MAX_TRACK_SIZE:track_size;
        if (res == FR_OK) {
             res = f_read(fil, track, track_size, &BytesRead);
#ifdef ENABLE_SEMIHOSTING_LOAD_TRACK
             printf("AFTER f_read  %04x to %08x , res = %d, BytesRead = %d\n",size_of_track, track, res,BytesRead);
#endif
	}
	return res;
}

FRESULT save_track(FIL *fil, uint32_t track_number, uint8_t *track, uint8_t *disk_info) {
        UINT BytesWritten;
        FRESULT res;

	uint32_t offset=0x100;
	uint32_t track_size;
	int i;

	if ((disk_info[0]=='E') && (disk_info[1]=='X')) {
	   // Must be EXTENDED CPC DSK
	   for (i=0;i<track_number;i++) {
	      offset+=(disk_info[0x34+i]<<8);
           }
	   track_size = (disk_info[0x34+i]<<8);
	} else {
	   track_size=0x1300;
	   offset += (track_size * track_number);
        }

	res = f_lseek(fil, offset);
	track_size = (track_size > MAX_TRACK_SIZE)? MAX_TRACK_SIZE:track_size;
        if (res == FR_OK) {
             res = f_write(fil, track, track_size, &BytesWritten);
#ifdef ENABLE_SEMIHOSTING_SAVE_TRACK
             printf("AFTER f_write %04x to %08x , res = %d, BytesWritten = %d\n",track_size, track, res,BytesWritten);
#endif
	}
	return res;
}




// b needs to point to a buffer at least 12 chars long
void compose_disk_name(char *b, uint8_t num) {
    char src[]= "CPCxxx.DSK";

    uint8_t t;
    uint8_t x = num;
    char *p,*q;
    p  = src;
    q = b;
    while (*p) {
       if (*p != 'x') {
          *q++ = *p++;
       } else {
          for (int y=2;y>=0;y--) {
            t = x%10;
            q[y] = t + '0';
            x =x/10;
          }
          q+=3; p+=3;
       }
    }
    *q=0;
}


void build_sector_pointer_table(uint32_t *sector_table, uint8_t * track, uint8_t *disk_info) {
	uint8_t rec,sectors;
	uint8_t *ptr;	// make it to an 8 bit array to make the pointer arithmetic easier
#ifdef ENABLE_SEMIHOSTING_SECTOR_TABLE
	printf("build_sector_pointer_table sector_table=%08X,track=%08X, disk_info=%08X\n",sector_table, track, disk_info);
#endif
	sectors = track[0x15];
	ptr = track + 0x100;
	//printf("sectors in track = %d\n",sectors);
	for (int s=0; s<sectors;s++) {
	   rec = track[0x18 + (s*8) + 2];
	   //printf("sector id %02x, addr = %08x\n",rec,ptr);
	   sector_table[rec & 0x1f] = (uint32_t)ptr;
	   ptr += 0x200;	
	}


}


// probably dont need to turn the optimiser off, but it kept on annoying me at the time
int __attribute__((optimize("O0")))  main(void) {
	char fname_buffer[20];
	char *diskfname = fname_buffer;
        FIL fil;

#ifdef ENABLE_SEMIHOSTING
        initialise_monitor_handles();   /*rtt*/
	printf("Semi hosting on\n");
#endif

	rcc_set_frequency(SYSCLK_200_MHZ);
	  // switch on compensation cell
	RCC->APB2ENR |= 0 |  RCC_APB2ENR_SYSCFGEN ;
	SYSCFG->CMPCR |= SYSCFG_CMPCR_CMP_PD; // enable compensation cell
	while ((SYSCFG->CMPCR & SYSCFG_CMPCR_READY) == 0);  // wait until ready

	//__disable_irq();
	
	// Enable CCMRAM clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CCMDATARAMEN, ENABLE); 

        config_backup_sram();

	/* PD{8..15}  and PD2 for SD card MOSI*/
	config_gpio_data();
	/* PE{0..15} */
	config_gpio_addr();
	/* PC{0..2} and PC8 and PC{10..12} */
	config_gpio_portc();

        config_gpio_dbg();

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 

	SysTick->CTRL  = 0;
	config_PC1_int();
	config_PC4_int();

        SD_NVIC_Configuration(); 

	uint32_t *p;
	p = (uint32_t *) &romdis;
	*p = 0x0000;
	
	//__enable_irq();


        memset(&fs32, 0, sizeof(FATFS));
        res = f_mount(&fs32, "",0);
        //res = f_mount(0, &fs32);
        if (res != FR_OK) {
                blink_pa6_pa7();
        }



        if ((*disk_select_port ^ *disk_select_port_complement) != 0xff) {
           // Default to CPC000.DSK if cannot find old config in RAM
           *disk_select_port = 0x00;
           *disk_select_port_complement = 0xff;
        }
	// Do not use sprintf here
	compose_disk_name(diskfname, *disk_select_port);

	uint8_t *disk_info_buffer_ptr = (uint8_t *) &disk_info_buffer;

	// load first 256 bytes of disk
        if (load_disk_info(&fil,diskfname, disk_info_buffer_ptr) != FR_OK) {
	   // First try to reset the disk_select_port back to zero
           *disk_select_port = 0x00;
           *disk_select_port_complement = 0xff;
	   compose_disk_name(diskfname, *disk_select_port);
           if (load_disk_info(&fil,diskfname, disk_info_buffer_ptr) != FR_OK) {
	      blink_pa6_pa7();
	   }
	}

	
#ifdef ENABLE_SEMIHOSTING_DISK_INFO
	printf("after load_disk_info. First four letters are %c%c%c%c\n",disk_info_buffer_ptr[0],disk_info_buffer_ptr[1],disk_info_buffer_ptr[2],disk_info_buffer_ptr[3]);
	printf ("main_thread_command addr = %08x\n",&main_thread_command);
#endif

	

#ifdef DISABLE_ALL_BUT_SHOW_MAIN_THREAD_ACTIVITY
	while(1) {
		GPIOA->ODR = 1;
		asm volatile ("nop");
		GPIOA->ODR = 0;
		asm volatile ("nop");
	}
#endif
		
	while(1) {
		if (fdc_write_flush_count) {
			//GPIOA->ODR = 1;
			//asm volatile ("nop");
			//GPIOA->ODR = 0;
			// there must have been a write
			fdc_write_flush_count--;
			if (fdc_write_flush_count == 0) {
				//GPIOA->ODR = 1;
				f_close(&fil);		// fclose the last dsk
				res = f_open(&fil, diskfname, FA_WRITE);
				save_track(&fil, fdc_track, (uint8_t *)  &track_buffer, disk_info_buffer_ptr);
				f_close(&fil);
				res = f_open(&fil, diskfname, FA_READ);
			}
		}
		if (!(main_thread_command & 0xc0000000) && (main_thread_command & 0xff)) {
			switch (main_thread_command) {
			   case (MAIN_THREAD_SEEK_COMMAND): {
				// SEEK
				main_thread_command |= 0x40000000;
				// check if there is a pending write
				if (fdc_write_flush_count) {
					//*fdc_log_ptr++ = 0xff;
					//*fdc_log_ptr++ = 0xfe;
					//GPIOA->ODR = 0;
					f_close(&fil);		// fclose the last dsk
					res = f_open(&fil, diskfname, FA_WRITE);
					
					save_track(&fil, fdc_prev_track, (uint8_t *) &track_buffer, disk_info_buffer_ptr);
					fdc_write_flush_count=0;  // kill the countdown
					f_close(&fil);
					res = f_open(&fil, diskfname, FA_READ);
					//GPIOA->ODR = 1;

				}
				//
				// There is not much point in failing if load_track fails
				// 2nd arg below is actually the cylinder to seek to

				//*fdc_log_ptr++ = 0xef;
				//*fdc_log_ptr++ = 0xee;
				load_track(&fil, main_thread_data, (uint8_t *) &track_buffer, disk_info_buffer_ptr);
				build_sector_pointer_table((uint32_t *)&sector_ptrs, (uint8_t *) &track_buffer,disk_info_buffer_ptr);
				main_thread_command |= 0xc0000000;
				
				break;
                           }
			   case (MAIN_THREAD_CHANGE_DISK_COMMAND): {
				// CHANGE DISK
				main_thread_command |= 0x40000000; // somewhat pointless
				f_close(&fil);		// fclose the last dsk
				compose_disk_name(diskfname, *disk_select_port);
#ifdef ENABLE_SEMIHOSTING_CHANGE_DISK
				printf("  new disk %s, track %d, addr %08X\n",diskfname,fdc_track,&track_buffer);
#endif
        			if (load_disk_info(&fil,diskfname, disk_info_buffer_ptr) != FR_OK) {
#ifdef ENABLE_SEMIHOSTING_CHANGE_DISK
					printf("  failed loading that disk. Try disk 0 instead\n");
#endif
	   				// First try to reset the disk_select_port back to zero
           				*disk_select_port = 0x00;
           				*disk_select_port_complement = 0xff;
	   				compose_disk_name(diskfname, *disk_select_port);
           				if (load_disk_info(&fil,diskfname, disk_info_buffer_ptr) != FR_OK) {
	      				   blink_pa6_pa7();
	   				}
				}
				// reload the current track
			        load_track(&fil, fdc_track, (uint8_t *) &track_buffer, disk_info_buffer_ptr);
				build_sector_pointer_table((uint32_t *)&sector_ptrs, (uint8_t *) &track_buffer,disk_info_buffer_ptr);

				main_thread_command = 0x00000000; 

				break;
			   }
			}
		}
	}
}

