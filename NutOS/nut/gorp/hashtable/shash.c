/*
 * Copyright (c) 1998-1999 Jeremie Miller <jer@jabber.org>
 * Copyright (C) 2013 Ole Reinhardt <ole.reinhardt@embedded-it.de>
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
 */

/* This code is based on
 * Based on BSD licensed mdnsd implementation by Jer <jer@jabber.org>
 * http://dotlocal.org/mdnsd/
 *
 * Unfortunately this site is now longer alive. You can still find it at:
 * http://web.archive.org/web/20080705131510/http://dotlocal.org/mdnsd/
 *
 * mdnsd - embeddable Multicast DNS Daemon
 * =======================================
 *
 * "mdnsd" is a very lightweight, simple, portable, and easy to integrate
 * open source implementation of Multicast DNS (part of Zeroconf, also called
 * Rendezvous by Apple) for developers. It supports both acting as a Query and
 * a Responder, allowing any software to participate fully on the .localnetwork
 * just by including a few files and calling a few functions.  All of the
 * complexity of handling the Multicast DNS retransmit timing, duplicate
 * suppression, probing, conflict detection, and other facets of the DNS
 * protocol is hidden behind a very simple and very easy to use interface,
 * described in the header file. The single small c source file has almost no
 * dependencies, and is portable to almost any embedded platform.
 * Multiple example applications and usages are included in the download,
 * including a simple persistent query browser and a tool to advertise .local
 * web sites.
 *
 * The code is licensed under both the GPL and BSD licenses, for use in any
 * free software or commercial application. If there is a licensing need not
 * covered by either of those, alternative licensing is available upon request.
 *
 */

/*!
 * \file gorp/hash/shash.c
 * \brief Simple hastable implementation of a string ==> void* hashtable
 *        Minimal and efficient
 *
 * \verbatim
 *
 * $Id$
 *
 * \endverbatim
 */

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <gorp/shash.h>

/*!
 * \addtogroup xgSHash
 */
/*@{*/

typedef struct node_struct
{
    char flag;
    struct node_struct *next;
    char *key;
    void *val;
} *SHNODE;

struct string_hash_table_struct
{
    int prime;
    SHNODE zen;
};

/*!
 * \brief Generates a hash code for a string.
 *
 * This function uses the ELF hashing algorithm as reprinted in
 * Andrew Binstock, "Hashing Rehashed," Dr. Dobb's Journal, April 1996.
 *
 * \param s     The string to hash
 *
 * \return      The calculated hash value
 */

static int32_t StrToHash(const char *str)
{
    /* ELF hash uses uint8_ts and unsigned arithmetic for portability */
    const uint8_t *name = (const uint8_t *)str;
    uint32_t hash = 0;
    uint32_t g;

    while (*name) {
        /* do some fancy bit calculations on the string */
        hash = (hash << 4) + (uint32_t)(*name++);
        g = (hash & 0xF0000000UL);
        if (g != 0) {
            hash ^= (g >> 24);
        }
        hash &= ~g;
    }

    return (int32_t)hash;
}


/*!
 * \brief Initialise a new hash
 *
 * \param prime size of the hash (!!! You have to use a _prime_ number (e.g. 5, 7, 11, 13, ...) !!!)
 *
 * \return      pointer to the new hash
 */
SHASH SHashInit(int prime)
{
    SHASH xnew;

    xnew = (SHASH)malloc(sizeof(struct string_hash_table_struct));
    xnew->prime = prime;
    xnew->zen = (SHNODE)malloc(sizeof(struct node_struct)*prime); /* array of SHNODE size of prime */
    memset(xnew->zen, 0, sizeof(struct node_struct)*prime);
    return xnew;
}

/*!
 * \brief Search the node for the given key in the hash table
 *
 * \param node  starting node to search from
 * \param key   The key to get
 *
 * \return      The node or NULL of node was not found
 */

static SHNODE SHashFindNode(SHNODE node, const char *key)
{
    for(; node != 0; node = node->next) {
        if((node->key != 0) && (strcmp(key, node->key) == 0)) {
            return node;
        }
    }
    return 0;
}


/*!
 * \brief Helper function: Add a new entry to the hash table
 *
 * \param hash  The hashtable to insert the new entry in
 * \param key   The key, under which the value should be entered
 * \param val   Void* pointer to be saved under 'key' in the hash table
 * \param flag  If flag != 0 the node memory is freed first and managed by this function
 */

static void SHashDoSet(SHASH hash, char *key, void *val, char flag)
{
    int32_t i;
    SHNODE node;

    /* Get the hash index of the key */
    i = StrToHash(key) % hash->prime;

    /* Does the key just exists? If not, find an empty one */
    node = SHashFindNode(&hash->zen[i], key);
    if (node == 0) {
        for(node = &hash->zen[i]; node != 0; node = node->next) {
            if(node->val == 0) {
                break;
            }
        }
    }

    /* If the key does not exists, create a new node and link into the hash */
    if (node == 0) {
        node = (SHNODE)malloc(sizeof(struct node_struct));
        node->next = hash->zen[i].next;
        hash->zen[i].next = node;
    }

    /* If flag is set, the node memory will be freed first and then reused with the new key / value pair */
    if (node->flag) {
        free(node->key);
        free(node->val);
    }

    node->flag = flag;
    node->key = key;
    node->val = val;
}


/*!
 * \brief Add a new entry to the hash table
 *
 * The caller is responsible for the key storage. No copies are made.
 * Do not free these valued before calling SHashFree()!!!
 *
 * If val is set to NULL, the entry is cleared and the memory is reused but
 * never freed. The number of keys can only grow up to peak usage.
 *
 * \param hash  The hashtable to insert the new entry in
 * \param key   The key, under which the value should be entered
 * \param val   Void* pointer to be saved under 'key' in the hash table
 */

void SHashSet(SHASH hash, char *key, void *val)
{
    if ((hash == 0) || (key == 0)) {
        return;
    }
    SHashDoSet(hash, key, val, 0);
}


/*!
 * \brief Add a new entry to the hash table
 *
 * Unlike SHashSet() where key and values are managed in the callers memory
 * space, here they are copied into the hash table and freed when
 * val is 0 or by calling SHashFree().
 *
 * If val is set to NULL, the entry is cleared and the memory is freed.
 *
 * \param hash  The hashtable to insert the new entry in
 * \param key   The key, under which the value should be entered
 * \param kley  Size of the key
 * \param val   Void* pointer to be saved under 'key' in the hash table
 * \param vlen  Size of the value
 */

void SHashStore(SHASH hash, const char *key, int key_len, void *val, int value_len)
{
    char *ckey;
    char *cval;

    if((hash == 0) || (key == 0) || (key_len == 0)) {
        return;
    }

    ckey = (char*)malloc(key_len+1);
    memcpy(ckey, key, key_len);
    ckey[key_len] = '\0';

    /* Assume the value is a string too and store it with a trailing
       zero for safety reasons
     */
    cval = (void*)malloc(value_len+1);
    memcpy(cval, val, value_len);
    cval[value_len] = '\0';

    SHashDoSet(hash, ckey, cval, 1);
}

/*!
 * \brief Retrive a value associated to key from the hash table
 *
 * Return the value or NULL of the key was not found.
 *
 * \param hash  The hast table to get the value from
 * \param key   Key to get the value for
 * \return      pointer to the value, NULL if no key was not found
 */
void *SHashGet(SHASH hash, const char *key)
{
    SHNODE node;

    if ((hash == 0) || (key == 0)) {
        return 0;
    }

    node = SHashFindNode(&hash->zen[StrToHash(key) % hash->prime], key);
    if (node == NULL) {
        return 0;
    }

    return node->val;
}

/*!
 * \brief Free the hash table and all entries
 *
 * \param hash  The hast table to free
 */

void SHashFree(SHASH hash)
{
    SHNODE node;
    SHNODE temp;
    int i;

    if(hash == 0) {
        return;
    }

    for (i = 0; i < hash->prime; i++) {
        for (node = (&hash->zen[i])->next; node != 0; ) {
            temp = node->next;
            if (node->flag) {
                free(node->key);
                free(node->val);
            }
            free(node);
            node = temp;
        }
    }

    free(hash->zen);
    free(hash);
}

/*!
 * \brief Interate over the hash table and call a callback for each key
 *        that has a value set.
 *
 *
 * \param hash      The hash table to iterate though
 * \param cb        SHASH_CB callback function
 * \param arg       An optional user pointer to pass to the callback function
 */
void SHashForEach(SHASH hash, SHASH_CB cb, void *arg)
{
    int i;
    SHNODE node;

    if ((hash == 0) || (cb == 0)) {
        return;
    }

    for(i = 0; i < hash->prime; i++) {
        for(node = &hash->zen[i]; node != 0; node = node->next) {
            if(node->key != 0 && node->val != 0) {
                (*cb)(hash, node->key, node->val, arg);
            }
        }
    }
}

/*@}*/
