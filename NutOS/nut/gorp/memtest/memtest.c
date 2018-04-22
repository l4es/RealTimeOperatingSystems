/*
 * Copyright (C) 2012 by Ole Reinhardt (ole.reinhardt@embedded-it.de)
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
 *
 ***********************************************************************
 *
 * The test algorithms are taken from the original public domain source
 * code from Michael Barr:
 *
 * Filename:    memtest.c
 *
 * Description: General-purpose memory testing functions.
 *
 * Notes:       This software can be easily ported to systems with
 *              different data bus widths by redefining 'datum'.
 *
 * Copyright (c) 1998 by Michael Barr.  This software is placed into
 * the public domain and may be used for any purpose.  However, this
 * notice must not be changed or removed and no warranty is either
 * expressed or implied by its publication or distribution.
 *
 */

/*
 * \verbatim
 * $Id: $
 * \endverbatim
 */

/*
 * The following functions implement three well known memory test algorithm
 * that can test most common memory problems.
 * These tree algorithms are in the public domain.
 *
 * A nice description of these algorithms can be found at:
 * http://www.esacademy.com/en/library/technical-articles-and-documents/miscellaneous/software-based-memory-testing.html
 *
 * And also at:
 * [1] Barr, Michael. "Software-Based Memory Testing," Embedded Systems Programming, July 2000, pp. 28-40.
 */

/*!
 * \brief       Check data bus wiring on a 8 bit databus
 *
 * Test the data bus wiring in a memory region by performing a walking 1's
 * test at a fixed address within that region. The address (and hence the
 * memory region) is selected by the caller.
 *
 * \param       addr        memory address that shall be used for this test
 *
 * \return      0 on success, a non-zero result is the first pattern that failed
 */

datum MemtestDataBus(volatile datum* addr)
{
    datum pattern;
    /*
     * Perform a walking 1's test at the given address.
     */
    for (pattern = 1; pattern != 0; pattern <<= 1) {
        /*
         * Write the test pattern.
         */
        *addr = pattern;
        /*
         * Read it back (immediately is okay for this test).
         */
        if (*addr != pattern) {
            return (pattern);
        }
    }
    return 0;
}

/*!
 * \brief       Check address bus wiring on a 8 bit databus
 *
 * Test the address bus wiring in a memory region by performing a walking
 * 1's test on the relevant bits of the address and checking for aliasing.
 * This test will find single-bit address failures such as stuck
 * -high, stuck-low, and shorted pins.
 *
 * The base address and size of the region are selected by the caller.
 *
 * \notes       For best results, the selected base address should
 *              have enough LSB 0's to guarantee single address bit
 *              changes.  For example, to test a 64-Kbyte region,
 *              select a base address on a 64-Kbyte boundary.  Also,
 *              select the region size as a power-of-two--if at all
 *              possible.
 *
 * \param       base    memory base address that shall be used for this test
 * \param       size    size of the memory region
 *
 * \return      NULL if the test succeeds.
 *              A non-zero result is the first address at which an
 *              aliasing problem was uncovered.  By examining the
 *              contents of memory, it may be possible to gather
 *              additional information about the problem.
 */

datum *MemtestAddrBus(volatile datum *base, size_t size)
{
    uint32_t addressMask = (size/sizeof(datum) - 1);
    uint32_t offset;
    uint32_t testOffset;
    datum pattern     = (datum) 0xAAAAAAAA;
    datum antipattern = (datum) 0x55555555;

    /*
     * Write the default pattern at each of the power-of-two offsets.
     */
    for (offset = 1; (offset & addressMask) != 0; offset <<= 1) {
        base[offset] = pattern;
    }

    /*
     * Check for address bits stuck high.
     */
    testOffset = 0;
    base[testOffset] = antipattern;

    for (offset = 1; (offset & addressMask) != 0; offset <<= 1) {
        if (base[offset] != pattern) {
            return ((datum *) &base[offset]);
        }
    }

    base[testOffset] = pattern;

    /*
     * Check for address bits stuck low or shorted.
     */
    for (testOffset = 1; (testOffset & addressMask) != 0; testOffset <<= 1) {
        base[testOffset] = antipattern;

        if (base[0] != pattern) {
            return ((datum *) &base[testOffset]);
        }

        for (offset = 1; (offset & addressMask) != 0; offset <<= 1) {
            if ((base[offset] != pattern) && (offset != testOffset)) {
                return ((datum *) &base[testOffset]);
            }
        }
        base[testOffset] = pattern;
    }

    return NULL;
}


/*!
 * \brief       Check address bus wiring on a 8 bit databus
 *
 * Test the integrity of a physical memory device by performing an
 * increment/decrement test over the entire region.  In the process
 * every storage bit in the device is tested as a zero and a one.
 *
 * The base address and size of the region are selected by the caller.
 *
 * \param       base    memory base address that shall be used for this test
 * \param       size    size of the memory region
 *
 * \return      NULL if the test succeeds. Also, in that case, the
 *              entire memory region will be filled with zeros.
 *
 *              A non-zero result is the first address at which an
 *              incorrect value was read back. By examining the
 *              contents of memory, it may be possible to gather
 *              additional information about the problem.
 */

datum * MemtestDevice(volatile datum * base, size_t size)
{
    unsigned long offset;
    size_t nWords = size / sizeof(datum);
    datum pattern;
    datum antipattern;

    /*
     * Fill memory with a known pattern.
     */
    for (pattern = 1, offset = 0; offset < nWords; pattern++, offset++) {
        base[offset] = pattern;
    }

    /*
     * Check each location and invert it for the second pass.
     */
    for (pattern = 1, offset = 0; offset < nWords; pattern++, offset++) {
        if (base[offset] != pattern) {
            return ((datum *) &base[offset]);
        }
        antipattern = ~pattern;
        base[offset] = antipattern;
    }

    /*
     * Check each location for the inverted pattern and zero it.
     */
    for (pattern = 1, offset = 0; offset < nWords; pattern++, offset++) {
        antipattern = ~pattern;
        if (base[offset] != antipattern) {
            return ((datum *) &base[offset]);
        }
    }
    return NULL;
}


