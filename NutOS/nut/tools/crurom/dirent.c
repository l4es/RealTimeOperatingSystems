const char dirent_rcsid[] = "@(#) $Id: dirent.c 4391 2012-07-24 09:55:48Z haraldkipp $";

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#ifdef _WIN32
#include <windows.h>
#endif

#include "dirent.h"

static void free_dircontents(DIRCONT *dp)
{
    DIRCONT *odp;

    while((odp = dp) != NULL) {
        if(dp->_d_entry != NULL)
            free(dp->_d_entry);
        dp = dp->_d_next;
        free(odp);
    }
}

DIR *opendir(const char *name)
{
    struct stat statb;
    DIR  *dirp;
    char *last;
    DIRCONT *dp;
    char *nbuf;
    unsigned long rc;
    WIN32_FIND_DATA ffb;
    HANDLE hdir;
    int len = strlen(name);

    if(!len)
        return NULL;

    if((nbuf = malloc(len + 5)) == NULL)
        return NULL;

    strcpy(nbuf, name);
    last = &nbuf[len - 1];

    if(((*last == '\\') || (*last == '/')) && (len > 1) &&
       (!((len == 3) && (name[1] == ':'))))
        *(last--) = 0;

    rc = stat(nbuf, &statb);

    if(rc) {
        free(nbuf);
        return NULL;
    }

    if((statb.st_mode & _S_IFDIR) != _S_IFDIR) {
        free(nbuf);
        return NULL;
    }

    if((dirp = malloc(sizeof(DIR))) == NULL) {
        free(nbuf);
        return NULL;
    }

    if((*last != '\\') && (*last != '/'))
        strcat (last, "/");

    strcat(last, "*.*");

    dirp->dd_loc = 0;
    dirp->dd_cp = NULL;
    dirp->dd_contents = NULL;

    hdir = FindFirstFile(nbuf, &ffb);
    rc = (hdir == INVALID_HANDLE_VALUE) ? GetLastError () : 0;

    if(rc) {
        free(nbuf);
        free(dirp);
        return NULL;
    }

    do {
        if(((dp = malloc(sizeof(DIRCONT))) == NULL) ||
           ((dp->_d_entry = strdup(ffb.cFileName)) == NULL)) {
            if(dp->_d_entry != NULL)
                free(dp);
            free(nbuf);
            free_dircontents(dirp->dd_contents);
            FindClose(hdir);
            return NULL;
        }

        if(dirp->dd_contents)
            dirp->dd_cp = dirp->dd_cp->_d_next = dp;
        else
            dirp->dd_contents = dirp->dd_cp = dp;

        dp->_d_next = NULL;

    } while(FindNextFile(hdir, &ffb));

    dirp->dd_cp = dirp->dd_contents;
    free(nbuf);

    FindClose(hdir);
    return dirp;
}

int closedir(DIR *dirp)
{
    if(dirp) {
        free_dircontents(dirp->dd_contents);
        free(dirp);
    }
    return 0;
}

struct dirent *readdir(DIR *dirp)
{
    static struct dirent dp;

    if((dirp == NULL) || (dirp->dd_cp == NULL))
        return NULL;

    dp.d_reclen = strlen (strcpy (dp.d_name, dirp->dd_cp->_d_entry));
    dp.d_off = dirp->dd_loc * 32;
    dp.d_ino = (ino_t)++dirp->dd_loc;
    dirp->dd_cp = dirp->dd_cp->_d_next;

    return &dp;
}
