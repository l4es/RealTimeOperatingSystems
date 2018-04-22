#ifndef _GETOPT_H_
#define _GETOPT_H_

/*
 * $Id: getopt.h 4391 2012-07-24 09:55:48Z haraldkipp $
 */

#ifdef __cplusplus
    extern "C" {
#endif

extern int getopt(int argc, char **argv, char *opts);

extern int optind;
extern char *optarg;

#ifdef __cplusplus
    }
#endif

#endif
