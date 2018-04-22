const char crurom_rcsid[] = "@(#) $Id: crurom.c 5351 2013-09-24 10:16:53Z haraldkipp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#ifdef _WIN32
#include <io.h>
#include "dirent.h"
#else
#include <unistd.h>
#include <dirent.h>
#define stricmp strcasecmp
#define strnicmp strncasecmp
#endif

#include "getopt.h"

#ifndef O_BINARY
#define O_BINARY 0
#endif


#define IDENT   "crurom"
#undef VERSION
#define VERSION "2.0.0"

static int entryno = 0;
static int verbose = 0;
static int recursive = 0;
static int prog_types_compat = 0;
static int enc_hex = 0;
static int enc_dec = 0;
static int enc_strings = 0;
static int enc_ugly = 0;
static int max_char_per_line = 16;
static char rootdir[256];
static int rootlen = 0;
static char outname[256];
static FILE *fpout;

int dofile(char *name)
{
    static char *esc_seq = "\aa\bb\ff\nn\rr\tt\vv";
    int rc = 0;
    int fd;
    unsigned char buf[512];
    int i;
    int nl_flag = 1;
    int cpl_cnt = 0;
    int cnt;
    long total = 0;
    char *fsname = name;

    if(strnicmp(fsname, rootdir, rootlen) == 0)
        fsname += rootlen;

    if((fd = open(name, O_RDONLY | O_BINARY)) == -1) {
        perror(name);
        return -1;
    }
    if(verbose)
        fprintf(stderr, IDENT ": Reading %s\n", name);

    for(;;) {
        if((cnt = read(fd, buf, sizeof(buf))) < 0) {
            perror(name);
            rc = -1;
            total = 0;
            break;
        }
        if(total == 0) {
            entryno++;
            if (!enc_ugly) {
                fprintf(fpout, "/*\n * File entry %d: %s\n */\n", entryno, fsname);
            }
            if (prog_types_compat) {
                fprintf(fpout, "prog_char file%ddata[]", entryno);
            } else {
                fprintf(fpout, "const char file%ddata[] PROGMEM", entryno);
            }
            if (enc_strings) {
                fputs(" = ", fpout);
            } else {
                fputs(" = {", fpout);
            }
        }
        if(cnt == 0)
            break;
        if (enc_strings) {
            /*
             * Encode buffer to string.
             *
             * When using this format, then at least text files will
             * become user-editable. However, due to the string terminator
             * each file occupies an additional byte in memory.
             */
            for(i = 0; i < cnt; i++) {
                if (nl_flag) {
                    fputs("\n\"", fpout);
                    nl_flag = 0;
                    cpl_cnt = 0;
                }
                /* Handle non-printable values. */
                if (buf[i] < 32 || buf[i] > 126) {
                    /* Handle a few escape sequences. */
                    char *cp = strchr(esc_seq, buf[i]);
                    if (cp && *cp) {
                        cp++;
                        fputc('\\', fpout);
                        fputc(*cp, fpout);
                        cpl_cnt += 2;
                        /* Terminate line at line-feeds. */
                        if (!enc_ugly) {
                            nl_flag = *cp == 'n';
                        }
                    }
                    else {
                        /* Write all other values as hex. */
                        cpl_cnt += fprintf(fpout, enc_ugly ? "\\x%x" : "\\x%02X", buf[i]);
                        /* ANSI-C doesn't limit hex to 2 characters.
                           Terminate the line if the next character
                           is a hex digit. */
                        if (i + 1 < cnt && isxdigit(buf[i + 1])) {
                            fputc('"', fpout);
                            nl_flag = 1;
                        }
                    }
                }
                /* Handle non-printable values. */
                else {
                    if (buf[i] == '"' || buf[i] == '\\') {
                        fputc('\\', fpout);
                        cpl_cnt++;
                    }
                    fputc((int)buf[i], fpout);
                    cpl_cnt++;
                }
                /* Terminate line if... */
                if(nl_flag == 0 && /* ...not already flagged. */
                   cpl_cnt >= max_char_per_line && /* ...maximum length reached. */
                   i + 1 < cnt) { /* ...more to come. */
                    fputc('"', fpout);
                    nl_flag = 1;
                }
            }
        } else {
            /*
             * Encode buffer to characters.
             *
             * This is the original default encoding, where file contents
             * represented in character arrays.
             */
            for(i = 0; i < cnt; i++) {
                /* Limit characters per line. */
                if((i % max_char_per_line) == 0) {
                    if(total != 0 || i != 0) {
                        fputc(',', fpout);
                    }
                    fputs("\n ", fpout);
                } else {
                    fputc(',', fpout);
                }
                /* Write all characters as hex values. */
                if (enc_hex) {
                    fprintf(fpout, enc_ugly ? "0x%x" : "0x%02X", buf[i]);
                }
                /* Write all characters as decimal values. */
                else if (enc_dec) {
                    fprintf(fpout, enc_ugly ? "%u" : "%3u", buf[i]);
                }
                /* Write characters or decimal values. */
                else {
                    /* Write non-printable values as decimal. */
                    if (buf[i] < 32 || buf[i] > 126 || buf[i] == '\'' || buf[i] == '\\') {
                        fprintf(fpout, enc_ugly ? "%u" : "%3u", buf[i]);
                    }
                    else
                        fprintf(fpout, "'%c'", buf[i]);
                }
            }
        }
        total += cnt;
    }
    close(fd);

    if (enc_strings) {
        if (nl_flag == 0) {
            fputc('"', fpout);
        }
    } else {
        fputs("\n}", fpout);
    }
    fputs(enc_ugly ? ";\n" : ";\n\n", fpout);

    if (!enc_ugly) {
        fputc('\n', fpout);
    }

    if (prog_types_compat) {
        fprintf(fpout, "prog_char file%dname[] = \"%s\";\n", entryno, fsname);
    } else {
        fprintf(fpout, "const char file%dname[] PROGMEM = \"%s\";\n", entryno, fsname);
    }
    if (!enc_ugly) {
        fputc('\n', fpout);
    }

    fprintf(fpout, "static ROMENTRY file%dentry = { ", entryno);

    if(entryno > 1)
        fprintf(fpout, "&file%dentry, ", entryno - 1);
    else
        fprintf(fpout, "0, ");

    if (prog_types_compat) {
        fprintf(fpout, "(prog_char *)file%dname, %ld, (prog_char *)file%ddata };\n", entryno, total, entryno);
    } else {
        fprintf(fpout, "file%dname, %ld, file%ddata };\n", entryno, total, entryno);
    }

    return rc;
}

int dodir(char *dirpath)
{
    int rc = 0;
    char path[256];
    DIR *dir;
    struct dirent *dire;
    struct stat statbuf;

    if((dir = opendir(dirpath)) == NULL) {
        fprintf(stderr, "Failed to scan directory %s\n", dirpath);
        return -1;
    }
    if(verbose)
        fprintf(stderr, "Scan %s\n", dirpath);
    while((dire = readdir(dir)) != NULL && rc == 0) {
        if((dire->d_name[0] == '.') || (stricmp(dire->d_name, "cvs") == 0) || (stricmp(dire->d_name, "svn") == 0))
            continue;
        strcpy(path, dirpath);
        strcat(path, "/");
        strcat(path, dire->d_name);
        stat(path, &statbuf);

        if(statbuf.st_mode & S_IFDIR)
            rc = dodir(path);
        else if(statbuf.st_mode & S_IFREG)
            rc = dofile(path);
    }
    closedir(dir);
    return rc;
}

void usage(void)
{
    fputs("Usage: crurom OPTIONS DIRECTORY\n"
      "OPTIONS:\n"
      "-cp       create __PROG_TYPES_COMPAT__\n"
      "-ed       encode to decimal values\n"
      "-eh       encode to hex values\n"
      "-es       encode to strings\n"
      "-eu       ugly but compact encoding\n"
      "-l <num>  characters per line\n"
      "-o <file> output file\n"
      "-r        recursive\n"
      "-v        verbose\n"
    , stderr);
}

int main(int argc, char **argv)
{
    int option;
    int i;
    int rc = 0;
    char *ocp;

    while((option = getopt(argc, argv, "c:e:l:o:rv?")) != EOF) {
        switch(option) {
        case 'c':
            if (strchr(optarg, 'p')) {
                prog_types_compat++;
            }
            break;
        case 'e':
            for (ocp = optarg; *ocp; ocp++) {
                if (*ocp == 'd') {
                    enc_dec++;
                }
                else if (*ocp == 'h') {
                    enc_hex++;
                }
                else if (*ocp == 's') {
                    enc_strings++;
                    max_char_per_line = 128;
                }
                else if (*ocp == 'u') {
                    enc_ugly++;
                }
                else {
                    usage();
                    return 1;
                }
            }
            break;
        case 'l':
            max_char_per_line = atoi(optarg);
            break;
        case 'o':
            strcpy(outname, optarg);
            break;
        case 'r':
            recursive++;
            break;
        case 'v':
            verbose++;
            break;
        default:
            usage();
            return 1;
        }
    }
    argc -= optind;
    argv += optind;

    if(outname[0]) {
        if((fpout = fopen(outname, "w")) == NULL) {
            perror(outname);
            return 3;
        }
    }
    else
        fpout = stdout;

    fprintf(fpout, "/*\n");
    fprintf(fpout, " * This file is automatically created by " IDENT " " VERSION "\n");
    fprintf(fpout, " */\n");
    fprintf(fpout, "#include <fs/uromfs.h>\n\n");

    if(argc) {
        for(i = 0; i < argc && rc == 0; i++) {
            strcpy(rootdir, argv[i]);
            strcat(rootdir, "/");
            rootlen = strlen(rootdir);
            rc = dodir(argv[i]);
        }
    }
    else {
        strcpy(rootdir, "./");
        rootlen = 2;
        rc = dodir(".");
    }
    fprintf(fpout, "\nROMENTRY *romEntryList = &file%dentry;\n", entryno);
    if(fpout != stdout)
        fclose(fpout);
    return rc;
}
