/*
 * Copyright (C) 2003-2006 by egnite Software GmbH
 * Copyright (C) 2011 by egnite GmbH
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

/*!
 * $Id: playmp3.c 4640 2012-09-24 12:05:56Z u_bonnes $
 */

#include <dev/board.h>
#include <dev/vscodec.h>
#include <dev/urom.h>

#include <sys/version.h>

#include <stdio.h>
#include <io.h>
#include <fcntl.h>
#include <errno.h>

#ifndef DEV_FS
#define DEV_FS                  devUrom
#endif

#ifndef DEV_FS_NAME
#define DEV_FS_NAME             "UROM"
#endif

#ifndef DEV_AUDIO_CODEC
#define DEV_AUDIO_CODEC         devSpiVsCodec0
#endif

#ifndef DEV_AUDIO_CODEC_NAME
#define DEV_AUDIO_CODEC_NAME    "audio0"
#endif

static char mp3buf[512];
static void PlayMp3File(char *path, int gain);

/*!
 * \example playmp3/playmp3.c
 *
 * This sample application plays MP3 files from the UROM filesystem. It
 * demonstrates how to use an audio decoder with Nut/OS. It can provide
 * a basis for talking equipment, alarm sound output etc.
 *
 * The code will run out of the box on the Elektor Internet Radio.
 *
 * To run this example code on Ethernut 1 or 2, you need to attach the
 * Medianut Board to the expansion port or use a similar hardware design
 * with a hardware based MP3 decoder.
 *
 * To run on the AT91SAM7X-EK or the AT91SAM9260-EK, you need to attach
 * Calypso Board or any similar hardware design providing an audio DAC.
 * In addition, you must accept the RealNetworks RPSL/RCSL license by
 * enabling this item in the Configurator. Note, that this license is
 * different from the BSD license of Nut/OS.
 *
 * The UROM filesystem is located in program memory. No external file
 * storage device is required. Use the crurom utility to create a C source
 * file named urom.c from the MP3 files located in subdirectory sounds.
 * Here's how to call crurom:
 *
 * crurom -r -ourom.c sounds
 *
 * The created file will then be compiled and linked to the application
 * code.
 */
int main(void)
{
    /* Assign stdout to the default console device. */
    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE.dev_name, "w", stdout);
    printf("\n\nPlay MP3 files on Nut/OS %s\n", NutVersionString());

    /* Initialize the file system driver. */
    if (NutRegisterDevice(&DEV_FS, 0, 0)) {
        puts("File system not available");
    }

    /* Initialize the audio decoder. */
    if (NutRegisterSpiDevice(&DEV_AUDIO_CODEC, &DEV_SPIBUS, 1)) {
        puts("Audio codec not available");
    }

    /* Play audio files in an endless loop. */
    for (;;) {
        PlayMp3File(DEV_FS_NAME ":sound1a.mp3", -40);
        PlayMp3File(DEV_FS_NAME ":sound2a.mp3", -30);
        PlayMp3File(DEV_FS_NAME ":sound3a.mp3", -20);
        PlayMp3File(DEV_FS_NAME ":sound4a.mp3", -10);
    }
    return 0;
}

/*
 * Play audio file with specified volume.
 */
static void PlayMp3File(char *path, int gain)
{
    int fh;
    int dh;
    int got;

    printf("Play %s on %s: ", path, DEV_AUDIO_CODEC_NAME);

    /* Open audio codec. */
    dh = _open(DEV_AUDIO_CODEC_NAME, _O_WRONLY | _O_BINARY);
    if (dh == -1) {
        printf("Error %d, can't open audio codec\n", errno);
        return;
    }

    /* Set volume. */
    _ioctl(dh, AUDIO_SET_PLAYGAIN, &gain);

    /* Open audio file. */
    fh = _open(path, _O_RDONLY | _O_BINARY);
    if (fh == -1) {
        printf("Error %d, can't open audio file\n", errno);
        return;
    } else {
        /* Transfer audio data from file to codec. */
        for (;;) {
            got = _read(fh, mp3buf, sizeof(mp3buf));
            if (got <= 0) {
                break;
            }
            _write(dh, mp3buf, got);
        }
        /* Close audio file. */
        _close(fh);
    }
    /* Close audio codec. Will not return until all buffered data has
       been played. */
    _close(dh);

    puts("Done");
    return;
}
