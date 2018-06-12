#include "main.h"

	.syntax unified
	.cpu cortex-m4
	.fpu softvfp
	.thumb

	.global EXTI1_IRQHandler
	.global EXTI4_IRQHandler
	.global romdis
	.global rom_base
	.global disk_image
	.global disk_select_port
	.global disk_select_port_complement
	.global main_thread_command
	.global main_thread_data
	.global disk_info_buffer
	.global track_buffer
	.global fdc_track
	.global fdc_prev_track
	.global fdc_write_flush_count
	.global sector_ptrs
	.global fdc_log_ptr;


.section .rodata
// Can put up to four 16K roms here which will get mapped to rom 12, 13, 14 and 15
rom_base:
// be careful if you add roms and later delete them. The old ones might be still in the STM32 flash
//.incbin "roms/Moon_Buggy.rom"
.incbin "roms/maxam15.rom"
//.incbin "roms/Moon_Buggy.rom"
// NOTE: Parados tries to talk to the floppy chip. If you for some reason comment out the floppy emu code,
//       then the Amstrad will hang on boot
//.incbin "roms/Arkanoid.rom"
.incbin "roms/parados12.rom"
//.incbin "roms/Ahhh.rom"
//.incbin "roms/blank.rom"

        .equ PERIPH_BASE     ,   0x40000000                                                                
        .equ PERIPH_BASE_APB1,   (PERIPH_BASE + 0x00000)
        .equ PERIPH_BASE_APB2,   (PERIPH_BASE + 0x10000)
        .equ PERIPH_BASE_AHB1,   (PERIPH_BASE + 0x20000)
        .equ PERIPH_BASE_AHB2,   0x50000000
        .equ PERIPH_BASE_AHB3,   0x60000000

        .equ GPIOA_BASE,   (PERIPH_BASE_AHB1 + 0x0000)                                           
        .equ GPIOB_BASE,   (PERIPH_BASE_AHB1 + 0x0400)
        .equ GPIOC_BASE,   (PERIPH_BASE_AHB1 + 0x0800)
        .equ GPIOD_BASE,   (PERIPH_BASE_AHB1 + 0x0C00)
        .equ GPIOE_BASE,   (PERIPH_BASE_AHB1 + 0x1000)
        .equ GPIOF_BASE,   (PERIPH_BASE_AHB1 + 0x1400)
        .equ GPIOG_BASE,   (PERIPH_BASE_AHB1 + 0x1800)
        .equ GPIOH_BASE,   (PERIPH_BASE_AHB1 + 0x1C00)
        .equ GPIOI_BASE,   (PERIPH_BASE_AHB1 + 0x2000)

	.equ C_TO_D_OFFSET,     (GPIOD_BASE - GPIOC_BASE)
	.equ C_TO_E_OFFSET,     (GPIOE_BASE - GPIOC_BASE)

        .equ GPIOA_MODER     ,   GPIOA_BASE + 0x00
        .equ GPIOA_OTYPER    ,   GPIOA_BASE + 0x04
        .equ GPIOA_OSPEEDR   ,   GPIOA_BASE + 0x08
        .equ GPIOA_PUPDR     ,   GPIOA_BASE + 0x0C
        .equ GPIOA_IDR       ,   GPIOA_BASE + 0x10
        .equ GPIOA_ODR       ,   GPIOA_BASE + 0x14
        .equ GPIOA_BSRR      ,   GPIOA_BASE + 0x18
        .equ GPIOA_LCKR      ,   GPIOA_BASE + 0x1C
        .equ GPIOA_AFRL      ,   GPIOA_BASE + 0x20
        .equ GPIOA_AFRH      ,   GPIOA_BASE + 0x24

        .equ GPIOB_MODER     ,   GPIOB_BASE + 0x00
        .equ GPIOB_OTYPER    ,   GPIOB_BASE + 0x04
        .equ GPIOB_OSPEEDR   ,   GPIOB_BASE + 0x08
        .equ GPIOB_PUPDR     ,   GPIOB_BASE + 0x0C
        .equ GPIOB_IDR       ,   GPIOB_BASE + 0x10
        .equ GPIOB_ODR       ,   GPIOB_BASE + 0x14
        .equ GPIOB_BSRR      ,   GPIOB_BASE + 0x18
        .equ GPIOB_LCKR      ,   GPIOB_BASE + 0x1C
        .equ GPIOB_AFRL      ,   GPIOB_BASE + 0x20
        .equ GPIOB_AFRH      ,   GPIOB_BASE + 0x24

        .equ GPIOC_MODER     ,   GPIOC_BASE + 0x00
        .equ GPIOC_OTYPER    ,   GPIOC_BASE + 0x04
        .equ GPIOC_OSPEEDR   ,   GPIOC_BASE + 0x08
        .equ GPIOC_PUPDR     ,   GPIOC_BASE + 0x0C
        .equ GPIOC_IDR       ,   GPIOC_BASE + 0x10
        .equ GPIOC_ODR       ,   GPIOC_BASE + 0x14
        .equ GPIOC_BSRR      ,   GPIOC_BASE + 0x18
        .equ GPIOC_LCKR      ,   GPIOC_BASE + 0x1C
        .equ GPIOC_AFRL      ,   GPIOC_BASE + 0x20
        .equ GPIOC_AFRH      ,   GPIOC_BASE + 0x24

        .equ GPIOD_MODER     ,   GPIOD_BASE + 0x00
        .equ GPIOD_OTYPER    ,   GPIOD_BASE + 0x04
        .equ GPIOD_OSPEEDR   ,   GPIOD_BASE + 0x08
        .equ GPIOD_PUPDR     ,   GPIOD_BASE + 0x0C
        .equ GPIOD_IDR       ,   GPIOD_BASE + 0x10
        .equ GPIOD_ODR       ,   GPIOD_BASE + 0x14
        .equ GPIOD_BSRR      ,   GPIOD_BASE + 0x18
        .equ GPIOD_LCKR      ,   GPIOD_BASE + 0x1C
        .equ GPIOD_AFRL      ,   GPIOD_BASE + 0x20
        .equ GPIOD_AFRH      ,   GPIOD_BASE + 0x24

        .equ GPIOE_MODER     ,   GPIOE_BASE + 0x00
        .equ GPIOE_OTYPER    ,   GPIOE_BASE + 0x04
        .equ GPIOE_OSPEEDR   ,   GPIOE_BASE + 0x08
        .equ GPIOE_PUPDR     ,   GPIOE_BASE + 0x0C
        .equ GPIOE_IDR       ,   GPIOE_BASE + 0x10
        .equ GPIOE_ODR       ,   GPIOE_BASE + 0x14
        .equ GPIOE_BSRR      ,   GPIOE_BASE + 0x18
        .equ GPIOE_LCKR      ,   GPIOE_BASE + 0x1C
        .equ GPIOE_AFRL      ,   GPIOE_BASE + 0x20
        .equ GPIOE_AFRH      ,   GPIOE_BASE + 0x24

        .equ MODER           ,   0x00
        .equ OTYPER          ,   0x04
        .equ OSPEEDR         ,   0x08
        .equ IDR             ,   0x10
        .equ ODR             ,   0x14

        .equ ROMDIS_HIGH     ,   0x0008

        .equ SCB_AIRCR       , 0xE000ED0C


        .equ    EXTI_Line1      ,       0x00002
        .equ    EXTI_Line2      ,       0x00004
        .equ    EXTI_Line4      ,       0x00010
        .equ    EXTI            ,       0x40013c00
        .equ    EXTI_IMR        ,       EXTI + 0x00
        .equ    EXTI_PR         ,       EXTI + 0x14

	.equ	IMR		,	0x00
	.equ	EMR		,	0x04
	.equ	RTSR		,	0x08
	.equ	PR		,	0x14

	.equ	DSK_TRACKINFO_NUMBEROFSECTORS 	, 0x15
	.equ	DSK_TRACKINFO_SECTORINFOLIST 	, 0x18


    .equ        SYSCFG_BASE,    0x40013800
    .equ        SYSCFG_EXTICR1, SYSCFG_BASE + 0x08


#define	DATA_OUT_MODE	0x55550020
#define	DATA_IN_MODE	0x00000020

#define Z80_WRITE_MASK	0x0001
#define _IORQ_MASK	0x0002
#define _ROMEN_MASK	0x0004
#define _MREQ_MASK	0x0010

#define ROMDIS_HIGH	0x0008

#define LOWEST_ROM_ID	6
#define SUPPORTED_NUMBER_OF_ROMS	2
.section .data
// rom_bank must immediately follow romdis
romdis:
	.word   0x00000000
rom_bank:
        .word   0x00000000

disk_select_port:
        .word   0x40024000
disk_select_port_complement:
        .word   0x40024001

fdc_log_ptr:
        .word   0x10000000

fdc_result_phase_index:
        .word   0x0000
fdc_result_buffer:
        .byte   0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00
fdc_read_data_bytes_left:
        .word   0x0000
fdc_read_data_ptr:
        .word   0x0000


// main_thread_data must immediately follow main_thread_command
main_thread_command:
	.word	0x00000000			// command word
main_thread_data:
	.word	0x00000000			// command arg



fdc_write_flush_count:
	.word	0x00000000

// fdc_prev_track must immediately follow fdc_track
fdc_track:
        .word   0x0000                          // track number
fdc_prev_track:
        .word   0xFFFFFFFF                      // prev track number


.section .bss
        .lcomm  EXTI1_tempvar,4
        .lcomm  EXTI2_tempvar,4
        .lcomm  EXTI4_tempvar,4
fdc_track_base:
        .word   0x00000000
//phases
        .equ    CMD_PHASE,0
        .equ    EXEC_PHASE,1
        .equ    RESULT_PHASE,2
fdc_phase:
        .word   CMD_PHASE
fdc_exec_delay:
        .word   0x00000000

fdc_command_buffer:
        .byte   0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00
fdc_current_bytes_left:
        .word   0x00
fdc_command_phase_index:
        .word   0x00
dsk_offset:
        .word   0x0000
track_offset:
	.word   0x0000

	.lcomm disk_info_buffer,0x100
	.lcomm track_buffer,MAX_TRACK_SIZE    // must be enough for 256 byte Track-Info plus 0x1200 track. Normally 0x1300 long

// The 'R' record id is anded with 0x1f to do a lookup from sector_ptrs, so should work if sectors are C1, C2 ...
//   or 41, 42 ....
	.lcomm sector_ptrs,4*32










.section .text

// _MREQ Handler. Int on +ve edge

.type EXTI4_IRQHandler, %function
EXTI4_IRQHandler:
        ldr     r12,=GPIOA_BASE
        // assume enter on rising edge
#ifdef DEBUG_EXTI4
        movs    r0,#0
        str     r0,[r12,ODR]
#endif

        ldr     r2,=GPIOC_BASE
EXTI4_Start_Polling_For_MREQ_LOW:
        ldr     r3,=romdis
3:      ldr     r1,[r2, C_TO_E_OFFSET + IDR]            // GPIOE->IDR get the addres
        ldr     r0,[r2,IDR]   // GPIOC->IDR
        tst     r0,#_MREQ_MASK                          // check _MREQ
        bne.n   3b
#ifdef DEBUG_EXTI4
        movs    r0,#3
        str     r0,[r12,ODR]
	sub	r0,#1
        str     r0,[r12,ODR]
#endif
        tst     r1,#0x8000              // check A15. If it is high. We are interested
        beq.n   end_EXTI4_IRQHandler
	//nop				// You need at least one NOP here for the delay between _MREQ and _ROMEN
	//nop
        ldr     r0,[r2,IDR]   // GPIOC->IDR             // reread _ROMEN as it goes low marginally after _MREQ
        tst     r0,#_ROMEN_MASK                         // check _ROMEN
        bne.n   end_EXTI4_IRQHandler
        ldr     r0,[r3],#4                      // after reading romdis, bump ptr to rom_bank
        str     r0,[r2,ODR]                     // set romdis high or low
        tst     r0,#ROMDIS_HIGH
        beq.n   end_EXTI4_IRQHandler

        ldr     r0,[r3]                         // r3 will be pointing at rom_bank
        subs    r0,r0,#LOWEST_ROM_ID
        lsls    r0,r0,#14
        bfc     r1,#14,#18              // mask address with 0x3fff
        orr     r1,r0,r1
        ldr     r3,=rom_base
        ldrb    r0,[r3,r1]              // get the byte from a 16K ROM
        lsls    r0,r0, #8
        ldr     r1,=DATA_OUT_MODE
        str     r0,[r2, C_TO_D_OFFSET + ODR] // GPIOD ODR
        str     r1,[r2, C_TO_D_OFFSET + MODER]  // MODER set to outputs

#ifdef DEBUG_EXTI4
        movs    r0,#3
        str     r0,[r12,ODR]
	sub	r0,#1
        str     r0,[r12,ODR]
#endif

// Use WFE to wait until _MREQ goes high
        ldr     r3,=EXTI
        movs     r0,#0
        movs    r1,#EXTI_Line4
        ldr     r0,[r3,IMR]   // EXTI_IMR
        and     r0,#0xffffffef  // turn off  PC4 int
        str     r0,[r3,IMR]   // EXTI_IMR
        str     r1,[r3,PR]   // EXTI_PR  - clear the current PC4 interrupt
        str     r1,[r3,EMR]   // EXTI_EMR - wait for an event rather than an interrupt on PC4

        mov     r1,#DATA_IN_MODE
        movs    r3,#0
        movs    r0,#3   // set PA0/PA1 high again
	dsb             // not entirely sure if this is required
        sev     // set the event register
        wfe     // clear the event register
        wfe     // wait for the positive edge of _MREQ

#ifdef DEBUG_SPURIOUS_EVENT_MREQ
        ldr     r0,[r2,IDR]   // GPIOC->IDR
        tst     r0,#_MREQ_MASK                          // check _MREQ
        bne.n   8f
        movs    r0,#0
        str     r0,[r12,ODR]
        nop
        add     r0,#1
        str     r0,[r12,ODR]
8:
#endif

// _MREQ should have just gone high here
#ifdef DEBUG_EXTI4
        movs    r0,#3
        str     r0,[r12, ODR]
#endif
        str     r1,[r2, C_TO_D_OFFSET + MODER]   // tristate databus
        str     r3,[r2, ODR]                    // Set ROMDIS LOW

        b.n     EXTI4_Start_Polling_For_MREQ_LOW      // loop again instead of exitting the ISR

end_EXTI4_IRQHandler:
#ifdef DEBUG_EXTI4
        movs    r0,#3
        str     r0,[r12, ODR]
#endif
        ldr     r3,=EXTI
        ldr     r0,[r3,IMR]   // EXTI_IMR
        orr     r0,#0x0010      // reenable PC4 interrupts
        //str     r0,[r3,IMR]   // EXTI_IMR

        movs    r1,#EXTI_Line4
        str     r1,[r3,PR]   // EXTI_PR 
        str     r0,[r3,IMR]   // EXTI_IMR	// I think you need to reenable after the clear
        ldr   r0,=EXTI4_tempvar             // You need to read-modify-write to prevent double interrupts
        str   r1,[r0]

        bx      lr




// _IORQ interrupt -ve edge
.type EXTI1_IRQHandler, %function
EXTI1_IRQHandler:
	ldr	r2,=GPIOC_BASE
        ldr     r12,=GPIOA_BASE

	ldr	r1,[r2, C_TO_E_OFFSET + IDR]		// GPIOE->IDR get the address 
	ldr	r0,[r2,IDR]	// get the portc control pins

	// NOTE: Parados wants to see a floppy chip
        tst     r1,#0x0480      // check A10/A8
        beq.n   fdc_chip_selected

	tst	r1,#0x2000	// check A13 is low
	bne.n	end_EXTI1_IRQHandler
	tst	r1,#0x8000	// check A15 is high
	beq.n	end_EXTI1_IRQHandler
	tst	r0,#Z80_WRITE_MASK	// check r/w is low
	bne.n	end_EXTI1_IRQHandler
// Most likely a write to the ROM select register
	ldr	r3,=rom_bank
	ldr	r0,[r2, C_TO_D_OFFSET + IDR] // GPIOD IDR. Get the byte written to the rom select reg
	lsrs	r0,r0, #8
	ands	r0,r0,#0xff
	str	r0,[r3]		// save the rom select register
	movs	r1,#0
	ldr	r3,=romdis
	str	r1,[r3]		// save potential romdis as LOW
	subs	r0,r0,#LOWEST_ROM_ID
	bmi.n	end_EXTI1_IRQHandler
	cmp	r0,#SUPPORTED_NUMBER_OF_ROMS
	bge.n	end_EXTI1_IRQHandler
	movs	r1,#0x0008
	
  	str	r1,[r3]		// save potential romdis as HIGH
#ifdef DEBUG_EXTI1
        movs    r0,#0
        str     r0,[r12,ODR]
#endif
	b.n	end_EXTI1_IRQHandler		// consider repeating the code here instead of jumping

waitIORQHIGH_end_EXTI1_IRQHandler:
#ifdef DEBUG_EXTI1_WFE
	ldr     r12,=GPIOA_BASE
        movs    r0,#0
        str     r0,[r12,ODR]
#endif
// Use WFE to wait until _IORQ goes high
        ldr     r3,=EXTI
        movs    r0,#0
        movs    r1,#EXTI_Line1
        ldr     r0,[r3,IMR]   // EXTI_IMR
        and     r0,#0xfffffffd  // turn off  PC1 int
        str     r0,[r3,IMR]   // EXTI_IMR

        ldr     r0,[r3,RTSR]   // EXTI_RTSR
        orr     r0,#0x0002	// Add in PC1 on rising edge event
        str     r0,[r3,RTSR]   // EXTI_RTSR
	
        str     r1,[r3,PR]   // EXTI_PR  - clear the current PC1 interrupt
        str     r1,[r3,EMR]   // EXTI_EMR - wait for an event rather than an interrupt on PC4

        mov     r1,#DATA_IN_MODE
        movs    r0,#3   // set PA0/PA1 high again
6:      dsb             // not entirely sure if this is required
        sev     // set the event register
        wfe     // clear the event register
        wfe     // wait for the positive edge of _MREQ

// _IORQ should have just gone high here
#ifdef DEBUG_EXTI1_WFE
        str     r0,[r12, ODR]
#endif
        str     r1,[r2, C_TO_D_OFFSET + MODER]   // tristate databus

        ldr     r0,[r3,RTSR]   // EXTI_RTSR
        and     r0,#0xfffffffd	// remove PC1 on rising edge event
        str     r0,[r3,RTSR]   // EXTI_RTSR

        // Don't loop. The chances of two IO requests in a row are very low

end_EXTI1_IRQHandler:
#ifdef DEBUG_EXTI1
        ldr     r12,=GPIOA_BASE
	movs	r0,#1
        str     r0,[r12, ODR]
#endif
        ldr     r3,=EXTI
        ldr     r0,[r3,IMR]   // EXTI_IMR
        orr     r0,#0x0002      // reenable PC1 interrupt
        //str     r0,[r3,IMR]   // EXTI_IMR

        movs    r1,#EXTI_Line1
        str     r1,[r3,PR]   // EXTI_PR 
        str     r0,[r3,IMR]   // EXTI_IMR	// reenable PC1
        ldr   r0,=EXTI1_tempvar             // You need to read-modify-write to prevent double interrupts
        str   r1,[r0]

        bx      lr


fdc_chip_selected:
	and	r3,r1,0xff00		// You need to narrow it down further.
	cmp	r3,#0xfb00
	bne.n	end_EXTI1_IRQHandler
	tst	r0,#Z80_WRITE_MASK
	bne	read_fdc_chip
// Must be a write
	tst	r1,#0x0001		// check A0
	beq	disk_select_port_write		// Write to FB7E
	ldr	r0,[r2, C_TO_D_OFFSET + IDR] // GPIOD IDR   // Main WRITE to FB7F
	lsrs	r0,r0, #8
	ldr	r3,=fdc_phase	
	ldr	r1,[r3]
	cmp	r1,#EXEC_PHASE		// most likely in a WRITE DATA
	beq	exec_write_data
#ifdef FDC_CMD_LOGGING
	ldr	r1,=fdc_log_ptr
	ldr	r12,[r1]
	strb	r0,[r12],#1
	bfc	r12,#16,#2			// make it wrap round
	strb	r0,[r12]		// write twice to make it easier to see the end of a command
	str	r12,[r1]
#endif

	ldr	r1,=fdc_command_phase_index
	ldr	r1,[r1]
	ldr	r3,=fdc_command_buffer
	strb	r0,[r3,r1]   // store the current byte of the command

	ldrb	r0,[r3,#0]	// get the command byte
	and	r0,#0x1f	// mask off the top bits
// jump to the handler
	tbh.w   [pc,r0,lsl #1]
.include "fdc_command_table.S"
cmd_unused:
	b.n	end_EXTI1_IRQHandler

// 20180602. Many of the commands are not implemented

cmd_0x03:
// SPECIFY
	movs	r12,#3
	add	r1,#1
	cmp	r1,r12
	bne.n	1f
	movs	r1,#0
1:
	ldr	r3,=fdc_command_phase_index
	str	r1,[r3]	
	b.n	end_EXTI1_IRQHandler
cmd_0x04:
//SENSE DEVICE STATUS
	movs	r12,#2
	b.n	end_EXTI1_IRQHandler
cmd_0x07:
//RECALIBRATE
	movs	r12,#2
	add	r1,#1
	cmp	r1,r12
	bne.n	1f
	ldr	r3,=fdc_track
	ldr	r1,[r3]			// grab the current track
	str	r1,[r3,0x04]		// save it in fdc_prev_track
	movs	r1,#0
	str	r1,[r3]			// zero the track
	ldr	r3,=main_thread_command
	movs	r0,#MAIN_THREAD_SEEK_COMMAND
	str	r1,[r3, 0x04]		// write cylinder zero
	str	r0,[r3]			// write 'SEEK' command
1:
	ldr	r3,=fdc_command_phase_index
	str	r1,[r3]	
	b.n	end_EXTI1_IRQHandler
cmd_0x08:
//SENSE INTERRUPT STATUS
	movs	r0,#RESULT_PHASE
	ldr	r3,=fdc_phase
	str	r0,[r3]
	movs	r0,#0
	ldr	r3,=fdc_result_phase_index
	str	r0,[r3]

	b.n	end_EXTI1_IRQHandler
cmd_0x0f:
//SEEK
	movs	r12,#3	// num of bytes for cmd
	add	r1,#1
	cmp	r1,r12
	bne.n	1f
	movs	r0,#0		// zero top bytes
	sub	r1,r1,#1
	ldrb	r0,[r3,r1]	// get the cylinder number
	ldr	r3,=fdc_track
	ldr	r12,[r3]
	str	r12,[r3,0x04]	// save the old cylinder in fdc_prev_track
	str	r0,[r3]		// store the cyclinder. will use it in the sense interrupt status cmd
	ldr	r3,=main_thread_command
	movs	r1,#MAIN_THREAD_SEEK_COMMAND

	str	r0,[r3,0x04]	// write the cylinder
	str	r1,[r3]		// write 'SEEK' command

	movs	r1,#0
1:	ldr	r3,=fdc_command_phase_index
	str	r1,[r3]	

	b.n	end_EXTI1_IRQHandler
cmd_0x02:
//READ DIAGNOSTIC
	movs	r12,#9
	b.n	end_EXTI1_IRQHandler



// Putting some of the later commands here, so that they are in range for the tbb
cmd_0x0c:
//READ DELETED DATA
	movs	r12,#9
	b.n	end_EXTI1_IRQHandler
cmd_0x0d:
//WRITE ID
	movs	r12,#6
	b.n	end_EXTI1_IRQHandler
cmd_0x11:
//SCAN EQUAL
	movs	r12,#9
	b.n	end_EXTI1_IRQHandler
cmd_0x19:
//SCAN LOW OR EQUAL
	movs	r12,#9
	b.n	end_EXTI1_IRQHandler
cmd_0x1d:
//SCAN HIGH OR EQUAL
	movs	r12,#9
	b.n	end_EXTI1_IRQHandler



cmd_0x05:
//WRITE DATA
cmd_0x06:
//READ DATA
	movs	r12,#9		// 9 bytes in cmd sequence
	add	r1,#1
	cmp	r1,r12
	bne.n	1f
// Got all the cmd sequence
	// r3 points to to the command buffer.
	ldr	r1,=track_buffer

	// Get the base address of the 512 byte block. The sector_ptr table is built when the track is loaded
	movs	r12,#0
	ldr	r2,=sector_ptrs
	movs	r0,#0
	ldrb	r0,[r3,0x04]		// get the 'R' record number
	and	r12,r0,#0x1f		// mask to make sure we pick a sector ptr
	ldr	r12,[r2,r12,lsl #2]
	ldr	r3,=fdc_read_data_ptr
	str	r12,[r3]

	movs	r12,#0
	ldrb	r12,[r1,DSK_TRACKINFO_NUMBEROFSECTORS]
	add	r1,r1,#DSK_TRACKINFO_SECTORINFOLIST		// offset to sector info list
	movs	r2,#0
2:
	ldrb	r2,[r1,0x02]		// grab next sector id from sector info block
        cmp	r2,r0		// compare it against the 'R' in the request
	beq.n	3f
	add	r1,r1,#8		// bump to next sector info block
	subs	r12,#1
	bne.n	2b
	// If we got to here we could not find the sector. We will just fudge it to return the last sector
	sub	r1,r1,#8
3:
// found sector

	// r1 points to the sector id in a sector info block
	ldr	r2,=fdc_result_buffer
	movs	r0,#0
	strb	r0,[r2],#1		//ST0

	ldrb	r0,[r1,0x04]		//ST1
	strb	r0,[r2],#1		//ST1
	
	ldrb	r0,[r1,0x05]		//ST2
	strb	r0,[r2],#1		//ST2

	ldrb	r0,[r1,0x00]		//C
	strb	r0,[r2],#1		//C

	ldrb	r0,[r1,0x01]		//H
	strb	r0,[r2],#1		//H

	ldrb	r0,[r1,0x02]		//R
	strb	r0,[r2],#1		//R

	ldrb	r0,[r1,0x03]		//N
	strb	r0,[r2],#1		//N

// r0 is the number of 256 byte records. Usually is 02. Shift that <<8 to get 0x0200 for bytes to get
// Even if we end up retrieving more that 0x0200 bytes due to EOT being higher than R, we just put 0x0200 here
	lsl	r0,#8
	ldr	r3,=fdc_read_data_bytes_left
	str	r0,[r3]



	movs	r0,#EXEC_PHASE
	ldr	r3,=fdc_phase
	str	r0,[r3]
// we are about to go to exec phase, but reset the cmd index now too
	movs	r0,#0
	ldr	r3,=fdc_command_phase_index
	str	r0,[r3]
	ldr	r3,=fdc_result_phase_index
	str	r0,[r3]
	movs	r0,#1
	ldr	r3,=fdc_exec_delay
	str	r0,[r3]

	// reset cmd index back to zero for next command
	movs	r1,#0
1:
	ldr	r3,=fdc_command_phase_index
	str	r1,[r3]	

	b.n	end_EXTI1_IRQHandler
cmd_0x09:
//WRITE DELETED DATA
	movs	r12,#9
	b.n	end_EXTI1_IRQHandler
cmd_0x0a:
//READ ID
	movs	r12,#2
	add	r1,#1
	cmp	r1,r12
	bne.n	1f
// last byte
	// r3 points to to the command buffer.
	ldr	r1,=track_buffer
	mov	r12,r1			// save the track base in r12. This points to 'Track-Info'

	
	ldr	r2,=fdc_result_buffer
	movs	r0,#0
	strb	r0,[r2],#1		//ST0

	ldrb	r0,[r1,#0x1c]		//ST1
	strb	r0,[r2],#1		//ST1
	
	ldrb	r0,[r1,#0x1d]		//ST2
	strb	r0,[r2],#1		//ST2

	ldrb	r0,[r1,#0x18]		//C
	strb	r0,[r2],#1		//C

	ldrb	r0,[r1,#0x19]		//H
	strb	r0,[r2],#1		//H

	ldrb	r0,[r1,#0x1a]		//R
	strb	r0,[r2],#1		//R

	ldrb	r0,[r1,#0x1b]		//N
	strb	r0,[r2],#1		//N

	movs	r0,#RESULT_PHASE
	ldr	r3,=fdc_phase
	str	r0,[r3]
// we are about to go to result phase, but reset the cmd index now too
	movs	r0,#0
	ldr	r3,=fdc_command_phase_index
	str	r0,[r3]
	ldr	r3,=fdc_result_phase_index
	str	r0,[r3]


	// reset index back to zero for next command
	movs	r1,#0
1:
	ldr	r3,=fdc_command_phase_index
	str	r1,[r3]	
	b.n	end_EXTI1_IRQHandler





read_fdc_chip:
	tst	r1,#1			// Check A0
	bne.n	read_data_register
// A0 is low so must be STATUS register
	
	ldr	r3,=fdc_phase
	ldr	r0,[r3]
	movs	r1,#0x8000   // always ready
	cmp	r0,#CMD_PHASE
	beq.n	1f
	cmp	r0,#RESULT_PHASE
	bne.n	2f
	movs	r1,#0xd000
	b.n	1f
2:	cmp	r0,#EXEC_PHASE
	bne.n	1f
	ldr	r3,=fdc_command_buffer
	ldrb	r0,[r3,0x00]		// get the command byte
	and	r0,#0x1f
	cmp	r0,#0x05		// write data command
	IT EQ	
	movseq	r1,#0x1000		// In WRITE DATA we return 0x10 or 0xB0
	IT NE	
	movsne	r1,#0x5000		// In READ DATA we return 0x50 or 0xF0
	ldr	r3,=fdc_exec_delay
	ldr	r0,[r3]
	subs	r0,#1
	ITT MI
	orrmi	r1,r1,#0xA000
	movsmi	r0,#0
	str	r0,[r3]			// save the fdc_exec_delay
1:
	ldr     r0,=DATA_OUT_MODE
	str	r1,[r2,C_TO_D_OFFSET +  ODR] // GPIOD ODR
	str	r0,[r2,C_TO_D_OFFSET + MODER]  // MODER set to outputs
#ifdef FDC_STATUS_LOGGING
        ldr     r12,=fdc_log_ptr
        ldr     r0,[r12]
	lsr	r1,#8
        strb    r1,[r0],#1
	bfc     r0,#16,#2                      // make it wrap round
        str     r0,[r12]
#endif
	
        b	waitIORQHIGH_end_EXTI1_IRQHandler
read_data_register:
// This is a read of FB7F
	ldr	r3,=fdc_phase
	ldr	r0,[r3]
	cmp	r0,#RESULT_PHASE
	bne	exec_phase
// RESULT PHASE
	ldr	r3,=fdc_command_buffer
	ldrb	r0,[r3,#0]	// get the command byte
// jump to the handler
	and     r0,#0x1f        // mask off the top bits
	tbh.w   [pc,r0,lsl #1]
.include "fdc_result_table.S"
result_unused:
	b	end_EXTI1_IRQHandler

result_0x03:
// SPECIFY
	b.n	end_EXTI1_IRQHandler
result_0x04:
//SENSE DEVICE STATUS
// only one byte response at all times
	movs	r1,#0		// clear top bytes
	
	ldr     r0,=DATA_OUT_MODE
	// TODO: Writing a 0x00 response is wrong here. Previous code did nothing. Need to research what is normal here
	str	r1,[r2,C_TO_D_OFFSET +  ODR] // GPIOD ODR
	str	r0,[r2,C_TO_D_OFFSET + MODER]  // MODER set to outputs
	ldr	r3,=fdc_phase
	movs	r0,#CMD_PHASE
	str	r0,[r3]

        b	waitIORQHIGH_end_EXTI1_IRQHandler
result_0x07:
//RECALIBRATE
	b.n	end_EXTI1_IRQHandler
result_0x08:
//SENSE INTERRUPT STATUS
// Normally after a seek the result is 0x20 <cylinder_number>
// But if the seek is not complete I think you need to reply with a single 0x80 (ie. only return 1 byte)
	movs	r12,#2
	ldr	r3,=fdc_result_phase_index
	ldr     r0,[r3]
	adds	r0,r0,#1
	str	r0,[r3]		// update the index
	cmp	r0,r12
	bne.n	2f
4:
// last byte
	ldr	r3,=fdc_track
	ldr	r1,[r3]
5:
	movs	r0,#CMD_PHASE
	ldr	r3,=fdc_phase
	str	r0,[r3]
	ldr	r3,=fdc_result_phase_index
	movs	r0,#0
	str	r0,[r3]
	ldr	r3,=fdc_command_phase_index
	str	r0,[r3]
	b.n	1f
2:
	// 1st byte which is the status. NOTE. If a seek is in progress we return 0x80 and dont bother with the 
	// cylinder byte
	ldr	r3,=main_thread_command
	ldr	r0,[r3]			// will be 1 if a seek has been issued, or 0x40000001 if in progress or 0xc0000001 if complete
	movs	r1,#0x80
	tst	r0,#MAIN_THREAD_SEEK_COMMAND	// If no seek queued, just say its completed.
	beq.n	5b
// Seek has been issued if bit 0 is 1
	lsls	r0,r0,#1		// Bit 31 -> C, Bit 30 -> N. If Carry clear then its incomplete
	bcc.n	5b
// Bit 31 must have been a  1. So the seek is complete
	movs	r1,#0
	str	r1,[r3]			// reset the current main thread command
	movs	r1,#0x20
1:
	
	lsls	r1,r1,#8
  
	ldr     r0,=DATA_OUT_MODE
	str	r1,[r2,C_TO_D_OFFSET + ODR] // GPIOD ODR
	str	r0,[r2,C_TO_D_OFFSET + MODER]  // MODER set to outputs

#ifdef FDC_READDATA_LOGGING
        ldr     r12,=fdc_log_ptr
        ldr     r3,[r12]
	lsr	r1,r1,#8
        strb    r1,[r3],#1
	bfc     r3,#16,#2                      // make it wrap round
        str     r3,[r12]
#endif
	b.n	waitIORQHIGH_end_EXTI1_IRQHandler



result_0x0f:
//SEEK
	b.n	end_EXTI1_IRQHandler
result_0x02:
//READ DIAGNOSTIC
	b.n	end_EXTI1_IRQHandler
result_0x05:
//WRITE DATA
result_0x06:
//READ DATA
	movs	r12,#7
	ldr	r3,=fdc_result_phase_index
	ldr	r1,[r3]
	add	r1,r1,#1
	str	r1,[r3]
	cmp	r1,r12
	bne.n	1f
// 	last byte
	movs	r0,#CMD_PHASE
	ldr	r3,=fdc_phase
	str	r0,[r3]
	movs	r0,#0
	ldr	r3,=fdc_result_phase_index
	str	r0,[r3]
	ldr	r3,=fdc_command_phase_index
	str	r0,[r3]

1:	sub	r1,r1,#1
	ldr	r3,=fdc_result_buffer
	ldrb	r0,[r3,r1]
	lsls	r0,r0,#8

	ldr     r1,=DATA_OUT_MODE
	str	r0,[r2,C_TO_D_OFFSET + ODR] // GPIOD ODR
	str	r1,[r2,C_TO_D_OFFSET + MODER]  // MODER set to outputs

#ifdef FDC_LOGGING
        ldr     r12,=fdc_log_ptr
        ldr     r1,[r12]
	lsr	r0,#8
        strb    r0,[r1],#1
	bfc     r1,#16,#2                      // make it wrap round
        str     r1,[r12]
#endif

	b.n	waitIORQHIGH_end_EXTI1_IRQHandler
result_0x09:
//WRITE DELETED DATA
	b.n	end_EXTI1_IRQHandler
result_0x0a:
//READ ID
	movs	r12,#7
	ldr	r3,=fdc_result_phase_index
	ldr	r1,[r3]
	add	r1,r1,#1
	str	r1,[r3]
	cmp	r1,r12
	bne.n	1f
// 	last byte
	movs	r0,#CMD_PHASE
	ldr	r3,=fdc_phase
	str	r0,[r3]
	movs	r0,#0
	ldr	r3,=fdc_result_phase_index
	str	r0,[r3]
	ldr	r3,=fdc_command_phase_index
	str	r0,[r5]

1:	sub	r1,r1,#1
	ldr	r3,=fdc_result_buffer
	ldrb	r0,[r3,r1]
	lsls	r0,r0,#8

	ldr     r1,=DATA_OUT_MODE
	str	r0,[r2,C_TO_D_OFFSET + ODR] // GPIOD ODR
	str	r1,[r2,C_TO_D_OFFSET + MODER]  // MODER set to outputs


	b.n	waitIORQHIGH_end_EXTI1_IRQHandler
result_0x0c:
//READ DELETED DATA
	b.n	end_EXTI1_IRQHandler
result_0x0d:
//WRITE ID
	b.n	end_EXTI1_IRQHandler
result_0x11:
//SCAN EQUAL
	b.n	end_EXTI1_IRQHandler
result_0x19:
//SCAN LOW OR EQUAL
	b.n	end_EXTI1_IRQHandler
result_0x1d:
//SCAN HIGH OR EQUAL
	b.n	end_EXTI1_IRQHandler
exec_phase:
	cmp	r0,#EXEC_PHASE
        bne	end_EXTI1_IRQHandler
	ldr	r3,=fdc_command_buffer
	ldrb	r1,[r3,#0]	// get the command byte
	and     r1,#0x1f        // mask off the top bits
	cmp	r1,#0x06		// only understand one command (read data) in exec phase
	beq.n	exec_read_data
	ldr	r3,=fdc_phase
	movs	r1,#CMD_PHASE
	str	r1,[r3]
	b	end_EXTI1_IRQHandler
// READ DATA EXEC PHASE
exec_read_data:
	ldr	r3,=fdc_read_data_bytes_left
	ldr	r1,[r3]
	subs	r1,r1,#1

	str	r1,[r3]
	ldr	r3,=fdc_read_data_ptr
	ldr	r1,[r3]
	ldrb	r0,[r1],#1
	str	r1,[r3]

#ifdef FDC_LOGGING
	//This will fill up your memory and overwrite stuff if you leave it on
	ldr	r12,=fdc_log_ptr
	ldr	r1,[r12]
	strb	r0,[r1],#1
	bfc     r1,#16,#2                      // make it wrap round
	str	r1,[r12]
#endif

	lsls	r0,r0,#8

// get data on data bus
	ldr     r1,=DATA_OUT_MODE
	str	r0,[r2,C_TO_D_OFFSET + ODR] // GPIOD ODR
	str	r1,[r2,C_TO_D_OFFSET + MODER]  // MODER set to outputs

// Now that we have some data on the data bus, work out whether its the last byte to transfer
	ldr	r3,=fdc_read_data_bytes_left
	ldr	r1,[r3]
	cmp	r1,#0
	bne	waitIORQHIGH_end_EXTI1_IRQHandler
// So we must have sent the last byte of a 512 byte block
	ldr	r12,=fdc_command_buffer
	movs	r0,#0
	ldrb	r0,[r12,0x04]		//get 'R'
	ldrb	r1,[r12,0x06]		// get 'EOT'
	cmp	r0,r1
	bne.n	1f
// it really is the end
        movs    r0,#RESULT_PHASE
        ldr     r3,=fdc_phase
        str     r0,[r3]
        movs    r0,#0
        ldr     r3,=fdc_result_phase_index
        str     r0,[r3]
        ldr     r3,=fdc_command_phase_index
        str     r0,[r3]

	b	waitIORQHIGH_end_EXTI1_IRQHandler
1:	add	r0,r0,#1
	strb	r0,[r12,0x04]		// bump up 'R'
	ldrb	r1,[r12,0x05]		// get 'N' for number of 256 byte records. ie should be 02
	lsl	r1,#8
	str	r1,[r3]			// store 0200 probably for new bytes_left
	and	r0,r0,#0x1f		// mask sector id to get a sort of hash
	ldr	r3,=sector_ptrs
	ldr	r12,=fdc_read_data_ptr
	ldr	r1,[r3,r0,lsl #2]	
	str	r1,[r12]		// store new fdc_read_data_ptr

        b	waitIORQHIGH_end_EXTI1_IRQHandler

exec_write_data:
	// r0 containe byte to write
	ldr	r3,=fdc_read_data_bytes_left
	ldr	r1,[r3]
	subs	r1,r1,#1
	bne.n	1f
// process last byte
	movs	r12,#RESULT_PHASE
	ldr	r3,=fdc_phase
	str	r12,[r3]
	movs	r12,#0
	ldr	r3,=fdc_result_phase_index
	str	r12,[r3]
	ldr	r3,=fdc_command_phase_index
	str	r12,[r3]
1:
	ldr	r3,=fdc_read_data_bytes_left
	str	r1,[r3]
	ldr	r3,=fdc_read_data_ptr
	ldr	r1,[r3]
	strb	r0,[r1],#1
	str	r1,[r3]
	ldr	r3,=fdc_write_flush_count
	ldr	r0,=DISK_FLUSH_COUNT
	str	r0,[r3]
        b	end_EXTI1_IRQHandler


// FB7E write support
disk_select_port_write:
	ldr	r1,[r2,C_TO_D_OFFSET + IDR] // GPIOD IDR
	lsrs	r1,r1, #8
	ldr	r3,=disk_select_port
	ldr	r3,[r3]
	strb	r1,[r3]
	ldr	r0,=main_thread_command
	str	r1,[r0,0x04]	// write the disk id to the main_thread_data
	eor	r1,r1,#0xff
	ldr	r3,=disk_select_port_complement
	ldr	r3,[r3]
	strb	r1,[r3]

        mov	r1,#MAIN_THREAD_CHANGE_DISK_COMMAND
        str     r1,[r0]              // write the 'CHANGE DISK' command

	//dsb
	// reset the board. Yeah, really.
	//ldr	r0,=0x05fa0004
	//ldr	r3,=SCB_AIRCR
	//str	r0,[r3]

	b	end_EXTI1_IRQHandler	


