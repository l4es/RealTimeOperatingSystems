/*
 * Copyright (C) 2012-2013 by egnite GmbH
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

#include <pro/uri.h>
#include <pro/uxml.h>
#include <pro/tcphost.h>
#include <pro/upnp.h>

#include <stdlib.h>
#include <string.h>

/* Use for local debugging only. Do NOT include into NUTDEBUG.
#define DEBUG_UPNPC
*/
#ifdef DEBUG_UPNPC
#include <stdio.h>
#endif

typedef struct _UPNP_OBSERVER UPNP_OBSERVER;

struct _UPNP_OBSERVER {
    UPNP_OBSERVER *uobs_next;
    char *uobs_domain;
    char *uobs_type;
    SSDP_OBSERVER_FUNCTION uobs_cb;
};

static UPNP_OBSERVER *upnp_observer_root;

/*!
 * \brief Call all our observers.
 *
 * \param ssvc    Specifies the service.
 * \param removal 0 if a new service has been added or 1 if a service will
 *                be removed.
 *
 * \return 0 on success or -1 on failure.
 */
static int CallObservers(SSDP_SERVICE *ssvc, int_fast8_t removal)
{
    int rc;
    UPNP_OBSERVER *uobs;

    for (rc = 0, uobs = upnp_observer_root; rc == 0 && uobs; uobs = uobs->uobs_next) {
        if (strcmp(ssvc->ssvc_domain, uobs->uobs_domain) == 0 && strcmp(ssvc->ssvc_type, uobs->uobs_type) == 0) {
            rc = (*uobs->uobs_cb)(ssvc, removal);
        }
    }
    return rc;
}

static const UXML_NODE *GetXmlNode(const UXML_NODE *list, const char *name)
{
    const UXML_NODE *node;

    for (node = list; node; node = node->xmln_next) {
        if (strcasecmp(node->xmln_name, name) == 0) {
            break;
        }
    }
    return node;
}

static const UXML_NODE *GetDeviceChildrenNodes(const UXML_NODE *root)
{
    root = GetXmlNode(root, "root");
    if (root) {
        root = GetXmlNode(root->xmln_child, "device");
        if (root) {
            root = root->xmln_child;
        }
    }
    return root;
}

static char *DuplicateXmlNodeContent(const UXML_NODE *node, const char *name)
{
    node = GetXmlNode(node, name);
    if (node && node->xmln_content) {
        return strdup(node->xmln_content);
    }
    return NULL;
}

static int ParseServiceDeviceInfo(const UXML_NODE *root, SSDP_SERVICE *ssvc)
{
    int rc = -1;
    const UXML_NODE *node;
    SSDP_DEVICE *sdev = ssvc->ssvc_dev;
    UPNP_DEVICE_INFO *udev = calloc(1, sizeof(*udev));

    root = GetDeviceChildrenNodes(root);
    if (udev) {
        sdev->sdev_info = udev;
        udev->udev_name = DuplicateXmlNodeContent(root, "friendlyName");
    }
    if (sdev->sdev_domain == NULL || sdev->sdev_type == NULL) {
        char *comp[5];

        SsdpSplitWords(DuplicateXmlNodeContent(root, "deviceType"), ':', comp, 5);
        if (sdev->sdev_domain == NULL) {
            sdev->sdev_domain = strdup(comp[1]);
        }
        if (sdev->sdev_type == NULL) {
            sdev->sdev_type = strdup(comp[3]);
        }
    }
    root = GetXmlNode(root, "serviceList");
    if (root) {
        for (root = root->xmln_child; root && root->xmln_child; root = root->xmln_next) {
            char *comp[5];
            const UXML_NODE *snode = root->xmln_child;

            node = GetXmlNode(snode, "serviceType");
            SsdpSplitWords(strdup(node->xmln_content), ':', comp, 5);
            if (strcasecmp(comp[1], ssvc->ssvc_domain) == 0 && strcasecmp(comp[3], ssvc->ssvc_type) == 0) {
                UPNP_SERVICE_INFO *usvc = calloc(1, sizeof(*usvc));

                if (usvc) {
                    node = GetXmlNode(snode, "SCPDURL");
                    if (node && node->xmln_content) {
                        usvc->usvc_url_scpd = strdup(node->xmln_content);
                    }
                    node = GetXmlNode(snode, "controlURL");
                    if (node && node->xmln_content) {
                        usvc->usvc_url_ctrl = strdup(node->xmln_content);
                    }
                    node = GetXmlNode(snode, "eventSubURL");
                    if (node && node->xmln_content) {
                        usvc->usvc_url_event = strdup(node->xmln_content);
                    }
                    ssvc->ssvc_info = usvc;
                    rc = 0;
                }
            }
        }
    }
    return rc;
}

static int ParseServiceInfo(const UXML_NODE *root, SSDP_SERVICE *ssvc)
{
    int rc = -1;
    const UXML_NODE *node;
    const UXML_NODE *actlist = NULL;
    const UXML_NODE *varlist = NULL;

    root = GetXmlNode(root, "scpd");
    if (root) {
        actlist = GetXmlNode(root->xmln_child, "actionList");
        varlist = GetXmlNode(root->xmln_child, "serviceStateTable");
    }
    if (root == NULL || actlist == NULL || varlist == NULL) {
        return -1;
    }

    /*
     * Read state variables.
     */
    for (varlist = varlist->xmln_child; varlist && varlist->xmln_child; varlist = varlist->xmln_next) {
        UPNP_VARIABLE *var = calloc(1, sizeof(*var));

        if (var) {
            for (node = varlist->xmln_child; node; node = node->xmln_next) {
                if (node->xmln_content) {
                    if (strcasecmp(node->xmln_name, "name") == 0) {
                        var->ustv_name = strdup(node->xmln_content);
                    }
                    else if (strcasecmp(node->xmln_name, "dataType") == 0) {
                        var->ustv_type = UpnpVarTypeIndex(node->xmln_content);
                    }
                    else if (strcasecmp(node->xmln_name, "defaultValue") == 0) {
                        var->ustv_default = strdup(node->xmln_content);
                    }
                    else if (strcasecmp(node->xmln_name, "allowedValueList") == 0) {

                    }
                }
            }
            if (var->ustv_name && var->ustv_type) {
                UPNP_SERVICE_INFO *usvc = ssvc->ssvc_info;

                var->ustv_next = usvc->usvc_stv;
                usvc->usvc_stv = var;
            } else {
                free(var->ustv_name);
                free(var->ustv_default);
                free(var);
            }
        }
    }

    /*
     * Read actions.
     */
    for (actlist = actlist->xmln_child; actlist && actlist->xmln_child; actlist = actlist->xmln_next) {
        SOAP_PROCEDURE *proc;
        const UXML_NODE *actnode = actlist->xmln_child;

        rc = -1;
        proc = calloc(1, sizeof(*proc));
        if (proc) {
            node = GetXmlNode(actnode, "name");
            if (node && node->xmln_content) {
                const UXML_NODE *argnode = GetXmlNode(actnode, "argumentList");

                proc->proc_name = strdup(node->xmln_content);
                if (argnode) {
                    for (argnode = argnode->xmln_child; argnode && argnode->xmln_child; argnode = argnode->xmln_next) {
                        SOAP_ARG *arg = calloc(1, sizeof(*arg));

                        if (arg) {
                            SOAP_ARG **argtyp = NULL;

                            for (node = argnode->xmln_child; node; node = node->xmln_next) {
                                if (node->xmln_content) {
                                    if (strcasecmp(node->xmln_name, "name") == 0) {
                                        arg->arg_name = strdup(node->xmln_content);
                                    }
                                    else if (strcasecmp(node->xmln_name, "direction") == 0) {
                                        if (*node->xmln_content == 'i') {
                                            argtyp = &proc->proc_argi;
                                        }
                                        else if (*node->xmln_content == 'o') {
                                            argtyp = &proc->proc_argo;
                                        }
                                    }
                                    else if (strcasecmp(node->xmln_name, "relatedStateVariable") == 0) {
                                        UPNP_VARIABLE *state;

                                        for (state = ((UPNP_SERVICE_INFO *)ssvc->ssvc_info)->usvc_stv; state; state = state->ustv_next) {
                                            if (strcasecmp(node->xmln_content, state->ustv_name) == 0) {
                                                arg->arg_info = state;
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                            if (arg->arg_name && argtyp && arg->arg_info) {
                                arg->arg_next = *argtyp;
                                *argtyp = arg;
                                rc = 0;
                            } else {
                                free(arg->arg_name);
                                free(arg);
                            }
                        }
                    }
                }
            }
            if (rc == 0) {
                UPNP_SERVICE_INFO *usvc = ssvc->ssvc_info;

                proc->proc_next = usvc->usvc_proc;
                usvc->usvc_proc = proc;
            }
        }
    }
    return rc;
}

static int SendHttpRequest(FILE *stream, const URI_SCHEME *schm)
{
    fputs("GET ", stream);
    fprintf(stream, "/%s HTTP/1.1\r\n", schm->schm_path);
    fprintf(stream, "Host: %s\r\n", schm->schm_host);
    fputs("Connection: close\r\n\r\n", stream);
    fflush(stream);

    return 0;
}

/*!
 * \brief Request XML tree from a specified location.
 *
 * \param url HTTP URL.
 *
 * \return Pointer to the root node or NULL if an error occurred.
 */
static UXML_NODE *RequestXmlTree(const char *url, uint32_t tmo)
{
    UXML_NODE *tree = NULL;

    if (strncasecmp(url, "http://", 7) == 0) {
        URI_SCHEME *schm;

        url += 7;
        schm = UriSchemeSplit(url);
        if (schm) {
            TCPSOCKET *sock = NutTcpCreateSocket();

            if (sock) {
                FILE *stream = TcpHostConnectStream(sock, schm->schm_host, schm->schm_portnum, tmo);

                if (stream) {
                    SendHttpRequest(stream, schm);
                    tree = UxmlParseStream(stream, NULL, NULL);
                    fclose(stream);
                }
                NutTcpCloseSocket(sock);
            }
            UriSchemeRelease(schm);
        }
    }
    return tree;
}

static int AddServiceDeviceInfo(SSDP_SERVICE *ssvc)
{
    int rc = -1;
    UXML_NODE *tree;

    tree = RequestXmlTree(ssvc->ssvc_dev->sdev_url_desc, 2000);
    if (tree) {
        rc = ParseServiceDeviceInfo(tree, ssvc);
        UxmlTreeDestroy(tree);
    }
    return rc;
}

static int AddServiceInfo(SSDP_SERVICE *ssvc)
{
    int rc = -1;
    UPNP_SERVICE_INFO *usvc = ssvc->ssvc_info;
    char *location;

    location = malloc(strlen(ssvc->ssvc_dev->sdev_url_desc) + strlen(usvc->usvc_url_scpd));
    if (location) {
        char *cp;

        strcpy(location, ssvc->ssvc_dev->sdev_url_desc);
        cp = strrchr(location, '/');
        if (cp) {
            UXML_NODE *tree;

            if (*usvc->usvc_url_scpd != '/') {
                cp++;
            }
            strcpy(cp, usvc->usvc_url_scpd);
            tree = RequestXmlTree(location, 2000);
            if (tree) {
                rc = ParseServiceInfo(tree, ssvc);
                UxmlTreeDestroy(tree);
            }
        }
    }
    return rc;
}

/*!
 * \brief Remove UPnP service information from
 */
static void RemoveServiceInfo(SSDP_SERVICE *ssvc)
{
    UPNP_SERVICE_INFO *usvc = ssvc->ssvc_info;

    if (usvc) {
        SOAP_PROCEDURE *proc;
        SOAP_PROCEDURE *proc_next = NULL;
        UPNP_VARIABLE *uvar;
        UPNP_VARIABLE *uvar_next = NULL;

        for (proc = usvc->usvc_proc; proc; proc = proc_next) {
            SOAP_ARG *arg;
            SOAP_ARG *arg_next = NULL;

            proc_next = proc->proc_next;
            for (arg = proc->proc_argi; arg; arg = arg_next) {
                arg_next = arg->arg_next;
                free(arg->arg_name);
                free(arg->arg_val);
                free(arg);
            }
            for (arg = proc->proc_argo; arg; arg = arg_next) {
                arg_next = arg->arg_next;
                free(arg->arg_name);
                free(arg->arg_val);
                free(arg);
            }
            free(proc->proc_name);
            free(proc);
        }

        for (uvar = usvc->usvc_stv; uvar; uvar = uvar_next) {
            uvar_next = uvar->ustv_next;
            free(uvar->ustv_name);
            free(uvar->ustv_default);
            free(uvar);
        }

        free(usvc->usvc_url_scpd);
        free(usvc->usvc_url_ctrl);
        free(usvc->usvc_url_event);
        free(usvc);
    }
}

/*!
 * \brief Service observer callback.
 *
 * This function is called by the SSDP receiver thread after a new service
 * has been added to the cache or before a service will be removed from the
 * cache.
 *
 * \param ssvc    Specifies the service.
 * \param removal 0 if a new service has been added or 1 if a service will
 *                be removed.
 *
 * \return 0 on success or -1 on failure.
 */
static int ServiceObserver(SSDP_SERVICE *ssvc, int_fast8_t removal)
{
    int rc = 0;

    if (removal) {
        CallObservers(ssvc, 1);
        RemoveServiceInfo(ssvc);
    } else {
        if (ssvc->ssvc_dev->sdev_info == NULL) {
            rc = AddServiceDeviceInfo(ssvc);
        }
        if (rc == 0) {
            rc = AddServiceInfo(ssvc);
        }
        if (rc == 0) {
            rc = CallObservers(ssvc, 0);
        }
    }
    return rc;
}

int UpnpRegisterServiceObserver(SSDP_OBSERVER_FUNCTION cb, const char *domain, const char *type, int_fast8_t mxwait)
{
    int rc = -1;
    UPNP_OBSERVER *uobs;

    uobs = malloc(sizeof(*uobs));
    if (uobs) {
        uobs->uobs_domain = strdup(domain);
        uobs->uobs_type = strdup(type);
        if (uobs->uobs_domain && uobs->uobs_type) {
            uobs->uobs_cb = cb;
            uobs->uobs_next = upnp_observer_root;
            upnp_observer_root = uobs;
            rc = SsdpRegisterServiceObserver(ServiceObserver, domain, type, mxwait);
        }
        if (rc) {
            free(uobs->uobs_domain);
            free(uobs->uobs_type);
            free(uobs);
        }
    }
    return rc;
}

SOAP_PROCEDURE *UpnpServiceProcByName(const SSDP_SERVICE *ssvc, const char *name)
{
    return SoapProcByName(((UPNP_SERVICE_INFO *) ssvc->ssvc_info)->usvc_proc, name);
}

int UpnpServiceProcCall(SSDP_SERVICE *ssvc, SOAP_PROCEDURE *proc, uint32_t tmo)
{
    int rc = -1;
    char *urn;
    char *url;
    char *uri;
    UPNP_SERVICE_INFO *usvc = ssvc->ssvc_info;

    urn = malloc(strlen(ssvc->ssvc_domain) + strlen(ssvc->ssvc_type) + 12);
    url = malloc(strlen(ssvc->ssvc_dev->sdev_url_desc) + strlen(usvc->usvc_url_ctrl));
    if (urn && url) {
        sprintf(urn, "%s:service:%s:1", ssvc->ssvc_domain, ssvc->ssvc_type);
        strcpy(url, ssvc->ssvc_dev->sdev_url_desc);
        uri = strrchr(url, '/');
        if (uri) {
            if (*usvc->usvc_url_ctrl != '/') {
                uri++;
            }
            strcpy(uri, usvc->usvc_url_ctrl);
            rc = SoapProcCallResource(proc, url, uri, urn, tmo);
        }
    }
    free(urn);
    free(url);

    return rc;
}
