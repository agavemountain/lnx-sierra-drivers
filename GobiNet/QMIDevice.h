/*===========================================================================
FILE:
   QMIDevice.h

DESCRIPTION:
   Functions related to the QMI interface device
   
FUNCTIONS:
   Generic functions
      IsDeviceValid
      PrintHex
      GobiSetDownReason
      GobiClearDownReason
      GobiTestDownReason

   Driver level asynchronous read functions
      ResubmitIntURB
      ReadCallback
      IntCallback
      StartRead
      KillRead

   Internal read/write functions
      ReadAsync
      UpSem
      ReadSync
      WriteSyncCallback
      WriteSync

   Internal memory management functions
      GetClientID
      ReleaseClientID
      FindClientMem
      AddToReadMemList
      PopFromReadMemList
      AddToNotifyList
      NotifyAndPopNotifyList
      AddToURBList
      PopFromURBList

   Internal userspace wrapper functions
      UserspaceunlockedIOCTL

   Userspace wrappers
      UserspaceOpen
      UserspaceIOCTL
      UserspaceClose
      UserspaceRead
      UserspaceWrite
      UserspacePoll

   Initializer and destructor
      RegisterQMIDevice
      DeregisterQMIDevice

   Driver level client management
      QMIReady
      QMIWDSCallback
      SetupQMIWDSCallback
      QMIDMSGetMEID
      QMIDMSSWISetFCCAuth

Copyright (c) 2011, Code Aurora Forum. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Code Aurora Forum nor
      the names of its contributors may be used to endorse or promote
      products derived from this software without specific prior written
      permission.

Alternatively, provided that this notice is retained in full, this software
may be relicensed by the recipient under the terms of the GNU General Public
License version 2 ("GPL") and only version 2, in which case the provisions of
the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
software under the GPL, then the identification text in the MODULE_LICENSE
macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
recipient changes the license terms to the GPL, subsequent recipients shall
not relicense under alternate licensing terms, including the BSD or dual
BSD/GPL terms.  In addition, the following license statement immediately
below and between the words START and END shall also then apply when this
software is relicensed under the GPL:

START

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License version 2 and only version 2 as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.

END

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
===========================================================================*/

//---------------------------------------------------------------------------
// Pragmas
//---------------------------------------------------------------------------
#pragma once

//---------------------------------------------------------------------------
// Include Files
//---------------------------------------------------------------------------
#include "Structs.h"
#include "QMI.h"

#define MAX_QCQMI 255
#define SEMI_INIT_DEFAULT_VALUE 0

extern int qcqmi_table[MAX_QCQMI];

/*=========================================================================*/
// Generic functions
/*=========================================================================*/

// Basic test to see if device memory is valid
bool IsDeviceValid( sGobiUSBNet * pDev );

#ifdef CONFIG_PM
bool bIsSuspend(sGobiUSBNet *pGobiDev);
#endif

// Print Hex data, for debug purposes
void PrintHex(
   void *         pBuffer,
   u16            bufSize );

// Sets mDownReason and turns carrier off
void GobiSetDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason );

// Clear mDownReason and may turn carrier on
void GobiClearDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason );

// Tests mDownReason and returns whether reason is set
bool GobiTestDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason );

/*=========================================================================*/
// Driver level asynchronous read functions
/*=========================================================================*/

// Resubmit interrupt URB, re-using same values
int ResubmitIntURB( struct urb * pIntURB );

// Read callback
//    Put the data in storage and notify anyone waiting for data
void ReadCallback( struct urb * pReadURB );

// Inturrupt callback
//    Data is available, start a read URB
void IntCallback( struct urb * pIntURB );

// Start continuous read "thread"
int StartRead( sGobiUSBNet * pDev );

// Kill continuous read "thread"
void KillRead( sGobiUSBNet * pDev );

/*=========================================================================*/
// Internal read/write functions
/*=========================================================================*/

// Start asynchronous read
//     Reading client's data store, not device
int ReadAsync(
   sGobiUSBNet *    pDev,
   u16                clientID,
   u16                transactionID,
   void               (*pCallback)(sGobiUSBNet *, u16, void *),
   void *             pData );

// Notification function for synchronous read
void UpSem( 
   sGobiUSBNet *    pDev,
   u16                clientID,
   void *             pData );

// Start synchronous read
//     Reading client's data store, not device
int ReadSync(
   sGobiUSBNet *    pDev,
   void **            ppOutBuffer,
   u16                clientID,
   u16                transactionID,
   int                *iID,
   struct semaphore   *pReadSem,
   int                *iIsClosing);

// Write callback
void WriteSyncCallback( struct urb * pWriteURB );

// Start synchronous write
int WriteSync(
   sGobiUSBNet *    pDev,
   char *             pInWriteBuffer,
   int                size,
   u16                clientID );

/*=========================================================================*/
// Internal memory management functions
/*=========================================================================*/

// Create client and allocate memory
int GetClientID( 
   sGobiUSBNet *      pDev,
   u8                 serviceType,
   struct semaphore   *pReadSem);

// Release client and free memory
bool ReleaseClientID(
   sGobiUSBNet *      pDev,
   u16                clientID,
   struct semaphore   *pReadSem);

// Find this client's memory
sClientMemList * FindClientMem(
   sGobiUSBNet *      pDev,
   u16                  clientID );

// Add Data to this client's ReadMem list
bool AddToReadMemList( 
   sGobiUSBNet *      pDev,
   u16                  clientID,
   u16                  transactionID,
   void *               pData,
   u16                  dataSize );

// Remove data from this client's ReadMem list if it matches 
// the specified transaction ID.
bool PopFromReadMemList( 
   sGobiUSBNet *      pDev,
   u16                  clientID,
   u16                  transactionID,
   void **              ppData,
   u16 *                pDataSize );

// Add Notify entry to this client's notify List
bool AddToNotifyList( 
   sGobiUSBNet *      pDev,
   u16                  clientID,
   u16                  transactionID,
   void                 (* pNotifyFunct)(sGobiUSBNet *, u16, void *),
   void *               pData );

int RemoveAndPopNotifyList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID );

// Remove first Notify entry from this client's notify list 
//    and Run function
int NotifyAndPopNotifyList( 
   sGobiUSBNet *      pDev,
   u16                  clientID,
   u16                  transactionID );

// Add URB to this client's URB list
bool AddToURBList( 
   sGobiUSBNet *      pDev,
   u16                  clientID,
   struct urb *         pURB );

// Remove URB from this client's URB list
struct urb * PopFromURBList( 
   sGobiUSBNet *      pDev,
   u16                  clientID );

/*=========================================================================*/
// Internal userspace wrappers
/*=========================================================================*/

// Userspace unlocked ioctl
long UserspaceunlockedIOCTL(
   struct file *     pFilp,
   unsigned int      cmd,
   unsigned long     arg );

/*=========================================================================*/
// Userspace wrappers
/*=========================================================================*/

// Userspace open
int UserspaceOpen( 
   struct inode *   pInode, 
   struct file *    pFilp );

// Userspace ioctl
int UserspaceIOCTL(
   struct inode *    pUnusedInode,
   struct file *     pFilp,
   unsigned int      cmd,
   unsigned long     arg );

// Userspace close
int UserspaceClose( 
   struct file *       pFilp,
   fl_owner_t          unusedFileTable );

// Userspace read (synchronous)
ssize_t UserspaceRead( 
   struct file *        pFilp,
   char __user *        pBuf, 
   size_t               size,
   loff_t *             pUnusedFpos );

// Userspace write (synchronous)
ssize_t UserspaceWrite(
   struct file *        pFilp, 
   const char __user *  pBuf, 
   size_t               size,
   loff_t *             pUnusedFpos );

unsigned int UserspacePoll(
   struct file *                  pFilp,
   struct poll_table_struct *     pPollTable );

/*=========================================================================*/
// Initializer and destructor
/*=========================================================================*/

// QMI Device initialization function
int RegisterQMIDevice( sGobiUSBNet * pDev, int is9x15 );

// QMI Device cleanup function
void DeregisterQMIDevice( sGobiUSBNet * pDev );

/*=========================================================================*/
// Driver level client management
/*=========================================================================*/

// Check if QMI is ready for use
bool QMIReady(
   sGobiUSBNet *    pDev,
   u16                timeout );

// QMI WDS callback function
void QMIWDSCallback(
   sGobiUSBNet *    pDev,
   u16                clientID,
   void *             pData );

// Fire off reqests and start async read for QMI WDS callback
int SetupQMIWDSCallback( sGobiUSBNet * pDev );

int SetupQMIQOSCallback( sGobiUSBNet * pDev );

// Register client, send req and parse MEID response, release client
int QMIDMSGetMEID( sGobiUSBNet * pDev );

// Register client, send req and parse FCC Authentication response, release client
int QMIDMSSWISetFCCAuth( sGobiUSBNet * pDev );

// Register client, send req and parse Data format response, release client
int QMIWDASetDataFormat( sGobiUSBNet * pDev, bool te_flow_control );

// send req and parse Data format response
int QMICTLSetDataFormat( sGobiUSBNet * pDev );

// Initialize Read Sync tasks semaphore
void InitSemID(sGobiUSBNet * pDev);

//Release all Read Sync tasks semaphore(s)
void StopSemID(sGobiUSBNet * pDev);

//Query semaphore slot ID
int iGetSemID(sGobiUSBNet *pDev,int line);

/***************************************************************************/
// wait_ms
/**************************************************************************/
void wait_ms(unsigned int ms) ;

// Userspace Release (synchronous)
int UserspaceRelease(struct inode *inode, struct file *file);

// Userspace Lock (synchronous)
int UserSpaceLock(struct file *filp, unsigned int cmd, struct file_lock *fl, struct file_lock *conf);

// sync memory
void gobi_flush_work(void);

