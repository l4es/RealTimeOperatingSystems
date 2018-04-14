/************************************************************************

  Interrupt vectors section - H8/300H  (16M mode)

  created 2-1-96 :: PBB
************************************************************************/


/***********************************************************************
  Place the prototypes for interrupt service routines here
*/

void start (void);		/* Startup code (in start.s)  */

/***********************************************************************/

typedef void (*fp) (void);

/***********************************************************************
  Place the interrupt service routine symbols in the table 
  to create the vector entry
*/

const fp HardwareVectors[] __attribute__ ((section (".vects"))) = {
  start,        /* Reset vector (hard, NMI high) */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* NMI */
  (fp)0L,       /* TRAPA 0 */
  (fp)0L,       /* TRAPA 1 */
  (fp)0L,       /* TRAPA 2 */
  (fp)0L,       /* TRAPA 3 */
  /*************** IRQs 0-7 */
  (fp)0L,       /* IRQ0 */
  (fp)0L,       /* IRQ1 */
  (fp)0L,       /* IRQ2 */
  (fp)0L,       /* IRQ3 */
  (fp)0L,       /* IRQ4 */
  (fp)0L,       /* IRQ5 */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* WOVI Watchdog */
  (fp)0L,       /* CMI Refresh */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* Reserved */
  /*************** ITU 0 */
  (fp)0L,       /* ITU0-IMIA */
  (fp)0L,       /* ITU0-IMIB */
  (fp)0L,       /* ITU0-OVF */
  (fp)0L,       /* Reserved */
  /*************** ITU 1 */
  (fp)0L,       /* ITU1-IMIA */
  (fp)0L,       /* ITU1-IMIB */
  (fp)0L,       /* ITU1-OVF */
  (fp)0L,       /* Reserved */
  /*************** ITU 2 */
  (fp)0L,       /* ITU2-IMIA */
  (fp)0L,       /* ITU2-IMIB */
  (fp)0L,       /* ITU2-OVF */
  (fp)0L,       /* Reserved */
  /*************** ITU 3 */
  (fp)0L,   	/* ITU3-IMIA */
  (fp)0L,       /* ITU3-IMIB */
  (fp)0L,       /* ITU3-OVF */
  (fp)0L,       /* Reserved */
  /*************** ITU 4 */
  (fp)0L,       /* ITU4-IMIA */
  (fp)0L,       /* ITU4-IMIB */
  (fp)0L,       /* ITU4-OVF */
  (fp)0L,       /* Reserved */
  /*************** DMAC 0-3 */
  (fp)0L,       /* DEND0A */
  (fp)0L,       /* DEND0B */
  (fp)0L,       /* DEND1A */
  (fp)0L,       /* DEND1B */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* Reserved */
  (fp)0L,       /* Reserved */
  /*************** SCI 0 */
  (fp)0L,  		/* ERI0 */
  (fp)0L,      	/* RXI0 */
  (fp)0L,       /* TXI0 */
  (fp)0L,       /* TEI0 */
  /*************** SCI 1 */
  (fp)0L, 		/* ERI1 */
  (fp)0L,      	/* RXI1 */
  (fp)0L,       /* TXI1 */
  (fp)0L,       /* TEI1 */
  (fp)0L        /* ADI A/D */
};

/* Done...
************************************************************************/
