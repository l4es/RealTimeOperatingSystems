/*
   Copyright (C) 2006 by krasnop@bellsouth.net (Alexei Krasnopolski)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#ifndef _STARTUP_H
#define _STARTUP_H
#ifdef _WATCOM_
/*
***************************************************************************
*                    Watcom start up code
*                     (Compiler Specific)
***************************************************************************
*/
static void fs_root( void )
{
}

extern "C"
struct rt_init
{
  unsigned char  rtn_type;
  unsigned char  priority;
  void ( * rtn ) ( void );
  unsigned short  padding;
} __wcpp_4_data_init_fs_root_= {1,1,fs_root,0};

extern "C"
void __wcpp_4_undefed_cdtor_( void )
{
}

#endif
#endif
