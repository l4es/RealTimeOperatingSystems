#include <cfg/os.h>

#include <fs/phatfs.h>
#include <sys/timer.h>
#include <dev/board.h>

#include <cfg/arch/gpio.h>

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#include <dirent.h>

#include <dev/blockdev.h>
#include <dev/mmcard.h>

#include <cfg/mmci.h>

#define VERSION     "0.1"

static int ScanFolder(char *root);

static void DumpInfo(int volid)
{
    MMC_CID cid;
    MMC_CSD csd;
    int rc;

    rc = _ioctl(volid, MMCARD_GETCID, &cid);
    if (rc == 0) {
        printf("MID 0x%3d, PNM: %c%c%c%c%c Rev: %d PSN 0x%08lx "
               "MDT: Year %4d Month %2d\n",
               cid.mmcid_mid, cid.mmcid_pnm[0], cid.mmcid_pnm[1],
               cid.mmcid_pnm[2], cid.mmcid_pnm[3], cid.mmcid_pnm[4],
               cid.mmcid_rev, cid.mmcid_psn,
               (cid.mmcid_mdt[0] & 0xf * 16) + (cid.mmcid_mdt[1] >> 4) + 2000,
               cid.mmcid_mdt[1] & 0xf);
    }
    rc = _ioctl(volid, MMCARD_GETCSD, &csd);
    if (rc == 0) {
        int version;
        version = (csd.mmcsd_spec >> 6) + 1;
        printf("CSD Version %d\n", version);
        if (version ==2) {
            uint32_t c_size;

            c_size = (csd.mmcsd_rfld[3]                )
                +    (csd.mmcsd_rfld[2]          <<   8)
                +    ((uint32_t) csd.mmcsd_rfld[1]          <<  16)
                +   ((uint32_t) (csd.mmcsd_rfld[0] &  0x3f) <<  24);
            printf("Capacity %ld MiB\n", (c_size + 1) >> 1);
        } else if (version == 1) {
            uint16_t c_size;
            uint8_t  c_mult;
            uint8_t  read_bl_len;
            uint16_t  mult;
            uint16_t  bl_len;
            uint32_t size;

            c_size  = (csd.mmcsd_rfld[0] & 0x03) << 10; /* Bit 73/72    */
            c_size += (csd.mmcsd_rfld[1]       ) <<  2; /* Bit 71 .. 64 */
            c_size += (csd.mmcsd_rfld[2] & 0xc0) >>  6; /* Bit 63/62    */
            c_mult  = (csd.mmcsd_rfld[3] & 0x03) <<  1; /* Bit 49/48    */
            c_mult += (csd.mmcsd_rfld[4] & 0x80) >>  7; /* Bit 47       */
            read_bl_len  = (csd.mmcsd_ccc_bl[1] & 0xf); /* Bit 83..80   */

            mult  = 1 << (c_mult +2);
            bl_len = 1 << read_bl_len;
            size = (c_size  + 1) * mult * bl_len;
            printf("Capacity %ld MiB\n", size >> 20);
        }
    }
}

int main(void)
{
    u_long baud = 115200;
    int volid;
    int rc;

    NutRegisterDevice(&DEV_CONSOLE, 0, 0);
    freopen(DEV_CONSOLE_NAME, "w", stdout);
    _ioctl(_fileno(stdout), UART_SETSPEED, &baud);
    puts("\nNut/OS MMC Test "__DATE__ " " __TIME__" Ver: " VERSION);

    NutRegisterDevice(&devPhat0, 0, 0);
    if (DEV_MMCARD.dev_type & IF_LAYER_SPI) {
        rc = NutRegisterSpiDevice(&DEV_MMCARD, &DEV_SPIBUS,SPI_MMC_CS);
    } else {
        rc = NutRegisterDevice(&DEV_MMCARD, 0, 0);
    }
    if (rc) {
        while(1) {
            puts("NutRegisterDevice failed\n");
            NutSleep(1000);
        }
    }
    for (;;) {
        for (;;) {
            printf("Mounting Card. This may take some time depending "
                   "card size and SPI speed...");
            if ((volid = _open(DEV_MMCARD_NAME ":1/PHAT0",
                               _O_RDWR | _O_BINARY)) != -1) {
                puts("ready");
                break;
            }
            puts("not found");
            NutSleep(2000);
        }
        DumpInfo(volid);
        ScanFolder("PHAT0:/");
        _close(volid);
    }
    return 0;
}

static int ScanFolder(char *root)
{
    int rc = 0;
    DIR *dir;
    char *path;

    printf("Scanning %s\n", root);
    dir = opendir(root);
    if(dir) {
        struct dirent *dep;

        path = malloc(256);
        while(rc == 0 && (dep = readdir(dir)) != NULL) {
            if (strcmp(dep->d_name, ".") && strcmp(dep->d_name, "..")) {
                if (dep->d_type) {
                    strcpy(path, root);
                    strcat(path, dep->d_name);
                    strcat(path, "/");
                    rc = ScanFolder(path);
                } else {
                    strcpy(path, root);
                    strcat(path, dep->d_name);
                    sprintf(path, "%s%s", root, dep->d_name);
                    printf("File %s\n", path);
                }
            }
        }
        closedir(dir);
        free(path);
    }
    return rc;
}
