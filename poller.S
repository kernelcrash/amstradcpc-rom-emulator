
// Note to self. Dont use r10 and r11

    .syntax unified
    .cpu cortex-m4
    .fpu softvfp
    .thumb

.global	rom_base
.global	dsk_base
.global special_port
.global special_port_complement

.section .rodata
// Can put up to two 16K roms here which will get mapped to rom 6 and 7. Since it maps 7 its
// really just for a CPC 464

rom_base:
// be careful if you add roms and later delete them. The old ones might be still in the STM32 flash
//.incbin "roms/Moon_Buggy.rom"
.incbin "roms/maxam15.rom"
.incbin "roms/parados12.rom"
//.incbin "roms/AMSDOS_0.7.ROM"


.section .data
special_port:
	.word	0x40024000
special_port_complement:
	.word	0x40024001

fdc_log_ptr:
	.word	0x20000100

fdc_result_phase_index:	
	.word	0x0000
fdc_result_buffer:
	.byte	0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00
fdc_read_data_bytes_left:
	.word	0x0000
fdc_read_data_ptr:
	.word	0x0000


.section .bss
fdc_track:
	.word	0x0000				// track number
fdc_track_base:
	.word	0x00000000
//phases
	.equ	CMD_PHASE,0
	.equ	EXEC_PHASE,1
	.equ	RESULT_PHASE,2
fdc_phase:
	.word	CMD_PHASE

fdc_current_command:
	.byte	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
fdc_current_bytes_left:
	.word	0x00
fdc_command_phase_index:
	.word	0x00
dsk_offset:
	.word	0x0000
dsk_base:
	.lcomm disk_image, 0x18000    // 96K worth. Could probably make it a little bigger easily
//rom1_base:
//	.skip 16384


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

	.equ MODER	     ,   0x00
	.equ OTYPER	     ,   0x04
	.equ OSPEEDR	     ,   0x08
	.equ IDR	     ,   0x10
	.equ ODR	     ,   0x14

	.equ ROMDIS_HIGH     ,   0x0008

	.equ SCB_AIRCR	     , 0xE000ED0C


#define	DATA_OUT_MODE	0x55550010
#define	DATA_IN_MODE	0x00000010

reg_gpioa_odr	.req	r0
reg_gpioc_base	.req	r1
reg_rom_base	.req	r2  
reg_gpiod_base	.req	r3  // data in
// r4,r5, r6 scratch
reg_rom_bank	.req	r7  // rom bank register
reg_pd2_state	.req	r8
reg_databus_out_mode	.req	r9  // MODER setting for D15-D8 out and D2 out

reg_gpioe_base	.req	r10

// I tried using .data here, but it runs slower than flash on the F4. I think this is due to the ART accelerator
.section .text
.weak asm_poller
.type asm_poller, %function

	
asm_poller:
	// disable all interrupts. We never come out of this. if we did we would use 'cpsie i'
	cpsid i

	// Use as many registers as we can
	ldr	reg_gpioa_odr,=GPIOA_ODR
	ldr	reg_gpioc_base,=GPIOC_BASE
	ldr     reg_gpiod_base,=GPIOD_BASE
	ldr     reg_gpioe_base,=GPIOE_BASE
//	r5 is scratch
//	r6 is scratch
//	r7 is the rom_bank register
	ldr	reg_rom_base,=rom_base
	ldr	reg_databus_out_mode,=DATA_OUT_MODE
	//ldr	reg_databus_in_mode,=DATA_IN_MODE
	movs	reg_pd2_state,#0
	movs	r6,#0
	str	r6,[reg_gpioa_odr]
	
/* Input Signals GPIO pins on _ROMEN -> PC2, _IORQ -> PC1, _WR -> PC0 */
/* Output Signals GPIO pins on ROMDIS -> PC3 */
/* SD card uses PC10, PC11, PC12 and PC8  and PD2*/

// Look for both _ROMEN low and _IORQ low
negative_edge_loop:
	ldr	r5,[reg_gpioe_base,IDR]		// GPIOE->IDR get the addres BEFORE we test __ROMEN. It should be valid 'just before' __ROMEN goes low. NB: top halfword should be 0x0000
	ldr	r6,[reg_gpioc_base,IDR]	// get the portc control pins
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
#ifdef DEBUG_MAIN
	IT	CC
	strcc	r6,[reg_gpioa_odr]		// probbaly zero in bit 0
#endif
	bcc.n	negative_edge_romen
	bpl.n	negative_edge_iorq
	ldr	r5,[reg_gpioe_base,IDR]		// GPIOE->IDR get the addres BEFORE we test __ROMEN. It should be valid 'just before' __ROMEN goes low. NB: top halfword should be 0x0000
	ldr	r6,[reg_gpioc_base,IDR]	// get the portc control pins
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
#ifdef DEBUG_MAIN
	IT	CC
	strcc	r6,[reg_gpioa_odr]		// debug output
#endif
	bcc.n	negative_edge_romen
	bpl.n	negative_edge_iorq
	ldr	r5,[reg_gpioe_base,IDR]		// GPIOE->IDR get the addres BEFORE we test __ROMEN. It should be valid 'just before' __ROMEN goes low. NB: top halfword should be 0x0000
	ldr	r6,[reg_gpioc_base,IDR]	// get the portc control pins
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
#ifdef DEBUG_MAIN
	IT	CC
	strcc	r6,[reg_gpioa_odr]		// debug output
#endif
	bcc.n	negative_edge_romen
	bpl.n	negative_edge_iorq
	ldr	r5,[reg_gpioe_base,#0x10]		// GPIOE->IDR get the addres BEFORE we test __ROMEN. It should be valid 'just before' __ROMEN goes low. NB: top halfword should be 0x0000
	ldr	r6,[reg_gpioc_base,#0x10]	// get the portc control pins
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
#ifdef DEBUG_MAIN
	IT	CC
	strcc	r6,[reg_gpioa_odr]		// debug output
#endif
	bcc.n	negative_edge_romen
	bpl.n	negative_edge_iorq
	ldr	r5,[reg_gpioe_base,#0x10]		// GPIOE->IDR get the addres BEFORE we test __ROMEN. It should be valid 'just before' __ROMEN goes low. NB: top halfword should be 0x0000
	ldr	r6,[reg_gpioc_base,#0x10]	// get the portc control pins
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
#ifdef DEBUG_MAIN
	IT	CC
	strcc	r6,[reg_gpioa_odr]		// debug output
#endif
	bcc.n	negative_edge_romen
	bpl.n	negative_edge_iorq
	b.n	negative_edge_loop


negative_edge_romen:

#ifdef DEBUG__ROMEN
	str	r6,[reg_gpioa_odr]
#endif

// NB: If you wanted to support say 4 ROMS beginning at ROM 4, you need 4 .incbin lines near
//     the beginning, and you  need
// 
//     ....
//     subs    r6,reg_rom_bank,#4
//     ....
//     cmp     r6,#3

// get the address
	tst	r5,#0x8000		// check A15. If it is high. We are interested
	beq.n	positive_edge_romen
	subs	r6,reg_rom_bank,#6	// Only support from ROM 6 upwards
	bmi.n	positive_edge_romen
	cmp	r6,#1			// Only support 2 ROMS
	bgt.n	positive_edge_romen
	lsls	r6,r6,#14
	movs	r4,#ROMDIS_HIGH
	str	r4,[reg_gpioc_base,ODR]	// set ROMDIS high
	bfc   	r5,#14,#18 		// mask with 0x3fff
	orr	r5,r5,r6
	ldrb	r6,[reg_rom_base,r5]
	lsls	r6,r6, #8
	str	r6,[reg_gpiod_base, ODR] // GPIOD ODR
	str	reg_databus_out_mode,[reg_gpiod_base, MODER]  // MODER set to outputs

#ifdef DEBUG__ROMEN
	movs	r5,#1
	str	r5,[reg_gpioa_odr]
#endif
	b.n	positive_edge_romen	
// This next bit is for the end of the cycle. Waiting for _ROMEN to go high
// Technically this comes at the end of the cycle, but putting it in the middle of the codebase for now
// the IT bit is to make the data bus go tristate as fast as we can
register_unused:
positive_edge_romen:
	ldr     r5,=DATA_IN_MODE
1:	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	CS
	strcs	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bcs.n	_romen_high

	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	CS
	strcs	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bcs.n	_romen_high

	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	CS
	strcs	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bcs.n	_romen_high

	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	CS
	strcs	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bcs.n	_romen_high

	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	CS
	strcs	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bcs.n	_romen_high

	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	CS
	strcs	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bcs.n	_romen_high

	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	CS
	strcs	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bcs.n	_romen_high

	b.n	1b
_romen_high:
	movs	r6,#0x0000
	str	r6,[reg_gpioc_base,ODR]	// set ROMDIS low
	//str	r5,[reg_gpiod_base, #0x00]  // MODER set to inputs
#ifdef DEBUG_MAIN
	movs	r6,#1
	str	r6,[reg_gpioa_odr]  // set PA0 HIGH
#endif
	b.n	negative_edge_loop





negative_edge_iorq:
// get the address
	tst	r5,#0x0480	// check A10/A8
	beq.n	fdc_chip_selected
	tst	r5,#0x2000	// check A13
	bne.n	positive_edge_iorq
	tst	r5,#0x8000	// check A15
	beq.n	positive_edge_iorq
	tst	r6,#0x40000000	// see if r/w is low
	bne.n	positive_edge_romen
	ldr	reg_rom_bank,[reg_gpiod_base, IDR] // GPIOD IDR
	lsrs	reg_rom_bank,reg_rom_bank, #8
	ands	reg_rom_bank,reg_rom_bank,#0xff
	movs	r6,#3
	str	r6,[reg_gpioa_odr]

	b.n	positive_edge_iorq

positive_edge_iorq:
	ldr     r5,=DATA_IN_MODE
1:	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	MI
	strmi	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bmi.n	_iorq_high

	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	MI
	strmi	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bmi.n	_iorq_high

	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	MI
	strmi	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bmi.n	_iorq_high

	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	MI
	strmi	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bmi.n	_iorq_high

	ldr	r6,[reg_gpioc_base,IDR]
	lsls	r6,r6,#30			// Move PC2 (_ROMEN) into C and PC1 (_IORQ) into N
	IT	MI
	strmi	r5,[reg_gpiod_base, MODER]  // MODER set to inputs
	bmi.n	_iorq_high

	b.n	1b
_iorq_high:
	//str	r5,[reg_gpiod_base, #0x00]  // MODER set to inputs
	b.n	negative_edge_loop





fdc_chip_selected:
	and	r4,r5,0xff00		// You need to narrow it down further.
	cmp	r4,#0xfb00
	bne	positive_edge_iorq
	tst	r6,#0x40000000
	bne	read_fdc_chip
// Must be a write
	tst	r5,#0x0001
	beq	special_port_write		// Write to FB7E
	ldr	r6,[reg_gpiod_base, IDR] // GPIOD IDR
	lsrs	r6,r6, #8
	and	r6,r6,#0xdf


#ifdef FDC_LOGGING
	ldr	r5,=fdc_log_ptr
	ldr	r4,[r5]
	strb	r6,[r4]
	add	r4,r4,1
	strb	r6,[r4]
	str	r4,[r5]
#endif

	ldr	r4,=fdc_command_phase_index
	ldr	r4,[r4]
	ldr	r5,=fdc_current_command
	strb	r6,[r5,r4]   // store the current byte of the command

	ldrb	r6,[r5,#0]	// get the command byte
// jump to the handler
        tbb.w   [pc,r6]
.include "fdc_command_table.S"
cmd_unused:
	b.n	positive_edge_iorq

// 20180602. Many of the commands are not implemented

cmd_0x03:
// SPECIFY
	movs	r6,#3
	add	r4,#1
	cmp	r4,r6
	bne.n	1f
	movs	r4,#0
1:
	ldr	r5,=fdc_command_phase_index
	str	r4,[r5]	
	b.n	positive_edge_iorq
cmd_0x04:
//SENSE DEVICE STATUS
	movs	r6,#2
	b.n	positive_edge_iorq
cmd_0x07:
//RECALIBRATE
	movs	r6,#2
	add	r4,#1
	cmp	r4,r6
	bne.n	1f
	movs	r4,#0
	ldr	r5,=fdc_track
	str	r4,[r5]
	ldr	r5,=dsk_base
	add	r5,r5,#0x100
	ldr	r6,=fdc_track_base
	str	r5,[r6]
1:
	ldr	r5,=fdc_command_phase_index
	str	r4,[r5]	
	b.n	positive_edge_iorq
cmd_0x08:
//SENSE INTERRUPT STATUS
	movs	r6,#RESULT_PHASE
	ldr	r5,=fdc_phase
	str	r6,[r5]
	movs	r6,#0
	ldr	r5,=fdc_result_phase_index
	str	r6,[r5]

	b.n	positive_edge_iorq
cmd_0x0f:
//SEEK
	movs	r6,#3	// num of bytes for cmd
	add	r4,#1
	cmp	r4,r6
	bne.n	1f
	movs	r6,#0
	sub	r4,r4,#1
	ldrb	r6,[r5,r4]	// get the clylinder number
	ldr	r5,=fdc_track
	str	r6,[r5]		// store the cyclinder. will use it in the sense interrupt status cmd

	movs	r4,#0
1:	ldr	r5,=fdc_command_phase_index
	str	r4,[r5]	

	b.n	positive_edge_iorq
cmd_0x42:
//READ DIAGNOSTIC
	movs	r6,#9
	b.n	positive_edge_iorq
cmd_0x45:
//WRITE DATA
	movs	r6,#9
	b.n	positive_edge_iorq
cmd_0x46:
//READ DATA
	movs	r6,#9		// 9 bytes in cmd sequence
	add	r4,#1
	cmp	r4,r6
	bne.n	1f
// Got all the cmd sequence
	// r5 points to to the command buffer.
	push	{r2,r3}
	movs	r3,#0			// clear upper bytes
	movs	r4,#0			// clear upper bytes
	ldrb	r4,[r5,#2]		// get the 'C' cylinder number
	ldr	r2,=dsk_base
	ldrh	r3,[r2,#0x32]		// get track sie	
	subs	r3,r3,#0
	bne.n	6f
	movs	r3,#0x1300		// default to 1300
6:	mul	r3,r3,r4		// multiple cylinder number times track size
	ldr	r6,=dsk_base
	add	r6,#0x100
	add	r6,r6,r3		// r6 should point to the track
	mov	r2,r6			// save the track base in r2. This points to 'Track-Info'

	ldrb	r4,[r5,#4]		// get the 'R' record number
	add	r6,r6,#0x1a		// offset to sector info sector id . 0x18 + 0x02
	movs	r5,#0
	movs	r3,#0
2:
	ldrb	r5,[r6,r3]		// grab next sector id from sector info block
        cmp	r5,r4
	beq.n	3f
	add	r3,r3,#8		// bump to next sector info block
	b.n	2b
3:
// found sector
	// r6+r3 points to the sector id in a sector info block
	add	r6,r6,r3
	ldr	r5,=fdc_result_buffer
	movs	r4,#0
	strb	r4,[r5],#1		//ST0

	ldrb	r4,[r6,#2]		//ST1
	strb	r4,[r5],#1		//ST1
	
	ldrb	r4,[r6,#3]		//ST2
	strb	r4,[r5],#1		//ST2

	ldrb	r4,[r6,#-2]		//C
	strb	r4,[r5],#1		//C

	ldrb	r4,[r6,#-1]		//H
	strb	r4,[r5],#1		//H

	ldrb	r4,[r6]			//R
	strb	r4,[r5],#1		//R

	ldrb	r4,[r6,#1]		//N
	strb	r4,[r5],#1		//N

// update the fdc_read_data_ptr pointer
	// r3 is the sector number, 0,1,2,3 etc
	lsls	r3,r3,#6		// 8 -> 0x200, 16 -> 0x400
	add	r2,r2,#0x0100
	add	r2,r2,r3		// r5 should point to the track base
	ldr	r6,=fdc_read_data_ptr
	str	r2,[r6]

	
// TODO This needs to change
	ldr	r5,=fdc_read_data_bytes_left
	movs	r4,#0x200
	str	r4,[r5]

	movs	r6,#EXEC_PHASE
	ldr	r5,=fdc_phase
	str	r6,[r5]
// we are about to go to exec phase, but reset the cmd index now too
	movs	r6,#0
	ldr	r5,=fdc_command_phase_index
	str	r6,[r5]
	ldr	r5,=fdc_result_phase_index
	str	r6,[r5]

	pop	{r2,r3}
	// reset cmd index back to zero for next command
	movs	r4,#0
1:
	ldr	r5,=fdc_command_phase_index
	str	r4,[r5]	

	b.n	positive_edge_iorq
cmd_0x49:
//WRITE DELETED DATA
	movs	r6,#9
	b.n	positive_edge_iorq
cmd_0x4a:
//READ ID
	movs	r6,#2
	add	r4,#1
	cmp	r4,r6
	bne.n	1f
// last byte
	// r5 points to to the command buffer.
	push	{r2,r3}
	movs	r3,#0			// clear upper bytes
	movs	r4,#0			// clear upper bytes
	ldr	r5,=fdc_track
	ldr	r4,[r5]
	ldr	r2,=dsk_base
	ldrh	r3,[r2,#0x32]		// get track sie	
	subs	r3,r3,#0
	bne.n	6f
	movs	r3,#0x1300		// default to 1300
6:	mul	r3,r3,r4		// multiple cylinder number times track size
	ldr	r6,=dsk_base
	add	r6,#0x100
	add	r6,r6,r3		// r6 should point to the track
	mov	r2,r6			// save the track base in r2. This points to 'Track-Info'

	
	ldr	r5,=fdc_result_buffer
	movs	r4,#0
	strb	r4,[r5],#1		//ST0

	ldrb	r4,[r6,#0x1c]		//ST1
	strb	r4,[r5],#1		//ST1
	
	ldrb	r4,[r6,#0x1d]		//ST2
	strb	r4,[r5],#1		//ST2

	ldrb	r4,[r6,#0x18]		//C
	strb	r4,[r5],#1		//C

	ldrb	r4,[r6,#0x19]		//H
	strb	r4,[r5],#1		//H

	ldrb	r4,[r6,#0x1a]		//R
	strb	r4,[r5],#1		//R

	ldrb	r4,[r6,#0x1b]		//N
	strb	r4,[r5],#1		//N

	movs	r6,#RESULT_PHASE
	ldr	r5,=fdc_phase
	str	r6,[r5]
// we are about to go to result phase, but reset the cmd index now too
	movs	r6,#0
	ldr	r5,=fdc_command_phase_index
	str	r6,[r5]
	ldr	r5,=fdc_result_phase_index
	str	r6,[r5]

	pop	{r2,r3}

	// reset index back to zero for next command
	movs	r4,#0
1:
	ldr	r5,=fdc_command_phase_index
	str	r4,[r5]	
	b.n	positive_edge_iorq
cmd_0x4c:
//READ DELETED DATA
	movs	r6,#9
	b.n	positive_edge_iorq
cmd_0x4d:
//WRITE ID
	movs	r6,#6
	b.n	positive_edge_iorq
cmd_0x51:
//SCAN EQUAL
	movs	r6,#9
	b.n	positive_edge_iorq
cmd_0x59:
//SCAN LOW OR EQUAL
	movs	r6,#9
	b.n	positive_edge_iorq
cmd_0x5d:
//SCAN HIGH OR EQUAL
	movs	r6,#9
	b.n	positive_edge_iorq

read_fdc_chip:
	tst	r5,#1
	bne.n	read_data_register
// A0 is low so must be status register
	
	ldr	r6,=fdc_phase
	ldr	r5,[r6]
	movs	r6,#0x8000   // always ready
	cmp	r5,#CMD_PHASE
	beq.n	1f
	cmp	r5,#RESULT_PHASE
	bne.n	2f
	movs	r6,#0xd000
	b.n	1f
2:	cmp	r5,#EXEC_PHASE
	bne.n	1f
	movs	r6,#0xf000
1:

	str	r6,[reg_gpiod_base, ODR] // GPIOD ODR
	str	reg_databus_out_mode,[reg_gpiod_base, MODER]  // MODER set to outputs
	
        b	positive_edge_iorq
read_data_register:
	ldr	r5,=fdc_phase
	ldr	r5,[r5]
	cmp	r5,#RESULT_PHASE
	bne	exec_phase
// RESULT PHASE
	ldr	r5,=fdc_current_command
	ldrb	r6,[r5,#0]	// get the command byte
// jump to the handler
        tbb.w   [pc,r6]
.include "fdc_result_table.S"
result_unused:
	b	positive_edge_iorq

result_0x03:
// SPECIFY
	b.n	positive_edge_iorq
result_0x04:
//SENSE DEVICE STATUS
// only one byte response at all times
	movs	r6,#0x0000   
	
	str	reg_databus_out_mode,[reg_gpiod_base, MODER]  // MODER set to outputs
	ldr	r5,=fdc_phase
	movs	r6,#CMD_PHASE
	str	r6,[r5]

        b	positive_edge_iorq
result_0x07:
//RECALIBRATE
	b.n	positive_edge_iorq
result_0x08:
//SENSE INTERRUPT STATUS
	movs	r6,#2
	ldr	r5,=fdc_result_phase_index
	ldr	r4,[r5]
	add	r4,r4,#1
	str	r4,[r5]
	cmp	r4,r6
	beq.n	1f
// 1st byte of result
	movs	r6,#0x2000
	b.n	2f
1:
// on the last byte of the result phase
	movs	r6,#CMD_PHASE
	ldr	r5,=fdc_phase
	str	r6,[r5]
	ldr	r5,=fdc_result_phase_index
	movs	r6,#0
	str	r6,[r5]
	// 2nd byte is the track number
	ldr	r6,=fdc_track
	ldr	r6,[r6]
	lsls	r6,r6,#8
2:
	str	r6,[reg_gpiod_base, ODR] // GPIOD ODR
	str	reg_databus_out_mode,[reg_gpiod_base, MODER]  // MODER set to outputs

	b.n	positive_edge_iorq
result_0x0f:
//SEEK
	b.n	positive_edge_iorq
result_0x42:
//READ DIAGNOSTIC
	b.n	positive_edge_iorq
result_0x45:
//WRITE DATA
	b.n	positive_edge_iorq
result_0x46:
//READ DATA
	movs	r6,#7
	ldr	r5,=fdc_result_phase_index
	ldr	r4,[r5]
	add	r4,r4,#1
	str	r4,[r5]
	cmp	r4,r6
	bne.n	1f
// 	last byte
	movs	r6,#CMD_PHASE
	ldr	r5,=fdc_phase
	str	r6,[r5]
	movs	r6,#0
	ldr	r5,=fdc_result_phase_index
	str	r6,[r5]
	ldr	r5,=fdc_command_phase_index
	str	r6,[r5]

1:	sub	r4,r4,#1
	ldr	r5,=fdc_result_buffer
	ldrb	r6,[r5,r4]
	lsls	r6,r6,#8

	str	r6,[reg_gpiod_base, ODR] // GPIOD ODR
	str	reg_databus_out_mode,[reg_gpiod_base, MODER]  // MODER set to outputs

	b.n	positive_edge_iorq
result_0x49:
//WRITE DELETED DATA
	b.n	positive_edge_iorq
result_0x4a:
//READ ID
	movs	r6,#7
	ldr	r5,=fdc_result_phase_index
	ldr	r4,[r5]
	add	r4,r4,#1
	str	r4,[r5]
	cmp	r4,r6
	bne.n	1f
// 	last byte
	movs	r6,#CMD_PHASE
	ldr	r5,=fdc_phase
	str	r6,[r5]
	movs	r6,#0
	ldr	r5,=fdc_result_phase_index
	str	r6,[r5]
	ldr	r5,=fdc_command_phase_index
	str	r6,[r5]

1:	sub	r4,r4,#1
	ldr	r5,=fdc_result_buffer
	ldrb	r6,[r5,r4]
	lsls	r6,r6,#8

	str	r6,[reg_gpiod_base, ODR] // GPIOD ODR
	str	reg_databus_out_mode,[reg_gpiod_base, MODER]  // MODER set to outputs


	b.n	positive_edge_iorq
result_0x4c:
//READ DELETED DATA
	b.n	positive_edge_iorq
result_0x4d:
//WRITE ID
	b.n	positive_edge_iorq
result_0x51:
//SCAN EQUAL
	b.n	positive_edge_iorq
result_0x59:
//SCAN LOW OR EQUAL
	b.n	positive_edge_iorq
result_0x5d:
//SCAN HIGH OR EQUAL
	b.n	positive_edge_iorq
exec_phase:
	cmp	r5,#EXEC_PHASE
        bne	positive_edge_iorq
	ldr	r5,=fdc_current_command
	ldrb	r6,[r5,#0]	// get the command byte
	cmp	r6,#0x46
	beq.n	exec_read_data
	ldr	r5,=fdc_phase
	movs	r6,#CMD_PHASE
	str	r6,[r5]
	b	positive_edge_iorq
// READ DATA EXEC PHASE
exec_read_data:
	ldr	r5,=fdc_read_data_bytes_left
	ldr	r4,[r5]
	subs	r4,r4,#1
	bne.n	1f
// process last byte
	movs	r6,#RESULT_PHASE
	ldr	r5,=fdc_phase
	str	r6,[r5]
	movs	r6,#0
	ldr	r5,=fdc_result_phase_index
	str	r6,[r5]
	ldr	r5,=fdc_command_phase_index
	str	r6,[r5]


1:
	ldr	r5,=fdc_read_data_bytes_left
	str	r4,[r5]
	ldr	r5,=fdc_read_data_ptr
	ldr	r6,[r5]
	ldrb	r4,[r6],#1
	str	r6,[r5]

	ldr	r5,=fdc_read_data_bytes_left
	ldr	r5,[r5]
	cmp	r5,#0x1f0
	blt.n	3f
#ifdef FDC_LOGGING
	// Sort of log the first 16 bytes or so of a sector transfer
	ldr	r5,=fdc_log_ptr
	ldr	r6,[r5]
	strb	r4,[r6],#1
	str	r6,[r5]
#endif
3:
	lsls	r4,r4,#8

	str	r4,[reg_gpiod_base, ODR] // GPIOD ODR
	str	reg_databus_out_mode,[reg_gpiod_base, MODER]  // MODER set to outputs

        b	positive_edge_iorq

// FB7E write support
special_port_write:
	ldr	r6,[reg_gpiod_base, IDR] // GPIOD IDR
	lsrs	r6,r6, #8
	ldr	r5,=special_port
	ldr	r5,[r5]
	strb	r6,[r5]
	eor	r6,r6,#0xff
	ldr	r5,=special_port_complement
	ldr	r5,[r5]
	strb	r6,[r5]
	dsb
	// reset the board. Yeah, really.
	ldr	r5,=0x05fa0004
	ldr	r6,=SCB_AIRCR
	str	r5,[r6]

	b	positive_edge_iorq


