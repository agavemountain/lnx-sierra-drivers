/*===========================================================================
FILE:
   QMIDevice.c

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
      WriteSync

   Internal memory management functions
      GetClientID
      ReleaseClientID
      FindClientMem
      AddToReadMemList
      PopFromReadMemList
      AddToNotifyList
      NotifyAndPopNotifyList

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
// Include Files
//---------------------------------------------------------------------------
#include <asm/unaligned.h>
#include "QMIDevice.h"
#include <linux/module.h>
#include <linux/proc_fs.h> // for the proc filesystem
#include <linux/device.h>
#include <linux/file.h>
#include <linux/usbdevice_fs.h>

//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------

extern int debug;
extern int is9x15;
extern int interruptible;
const bool clientmemdebug = 0;
enum {
 eNotifyListEmpty=-1,
 eNotifyListNotFound=0,
 eNotifyListFound=1,
};

#define SEND_ENCAPSULATED_COMMAND (0)
#define GET_ENCAPSULATED_RESPONSE (1)
#define USB_WRITE_TIMEOUT 500   // must be less than AM timeout
#define USB_WRITE_RETRY (2)
#define USB_READ_TIMEOUT (500)
#define MAX_RETRY 5
#define ENABLE_MAX_RETRY_LOCK_MSLEEP_TIME 10
/* initially all zero */
int qcqmi_table[MAX_QCQMI];

extern bool isModuleUnload(sGobiUSBNet *pDev);

#define CLIENT_READMEM_SNAPSHOT(clientID, pdev)\
   if( debug == 1 && clientmemdebug )\
   {\
      sClientMemList *pclnt;\
      sReadMemList *plist;\
      pclnt = FindClientMem((pdev), (clientID));\
      plist = pclnt->mpList;\
      if( (pdev) != NULL){\
          while(plist != NULL)\
          {\
             DBG(  "clientID 0x%x, mDataSize = %u, mpData = 0x%p, mTransactionID = %u,  \
                    mpNext = 0x%p\n", (clientID), plist->mDataSize, plist->mpData, \
                    plist->mTransactionID, plist->mpNext  ) \
             /* advance to next entry */\
             plist = plist->mpNext;\
          }\
      }\
   }

#ifdef CONFIG_PM
// Prototype to GobiNetSuspend function
int GobiNetSuspend(
   struct usb_interface *     pIntf,
   pm_message_t               powerEvent );
#endif /* CONFIG_PM */

int weakup_inode_process(struct file *pFilp,struct task_struct * pTask);
// IOCTL to generate a client ID for this service type
#define IOCTL_QMI_GET_SERVICE_FILE 0x8BE0 + 1

// IOCTL to get the VIDPID of the device
#define IOCTL_QMI_GET_DEVICE_VIDPID 0x8BE0 + 2

// IOCTL to get the MEID of the device
#define IOCTL_QMI_GET_DEVICE_MEID 0x8BE0 + 3

#define IOCTL_QMI_RELEASE_SERVICE_FILE_IOCTL  (0x8BE0 + 4)

#define IOCTL_QMI_ADD_MAPPING 0x8BE0 + 5
#define IOCTL_QMI_DEL_MAPPING 0x8BE0 + 6
#define IOCTL_QMI_CLR_MAPPING 0x8BE0 + 7

#define IOCTL_QMI_QOS_SIMULATE 0x8BE0 + 8
#define IOCTL_QMI_GET_TX_Q_LEN 0x8BE0 + 9

#define IOCTL_QMI_EDIT_MAPPING 0x8BE0 + 10
#define IOCTL_QMI_READ_MAPPING 0x8BE0 + 11
#define IOCTL_QMI_DUMP_MAPPING 0x8BE0 + 12
#define IOCTL_QMI_GET_USBNET_STATS 0x8BE0 + 13
#define IOCTL_QMI_SET_DEVICE_MTU 0x8BE0 + 14

// CDC GET_ENCAPSULATED_RESPONSE packet
#define CDC_GET_ENCAPSULATED_RESPONSE_LE 0x01A1ll
#define CDC_GET_ENCAPSULATED_RESPONSE_BE 0xA101000000000000ll
/* The following masks filter the common part of the encapsulated response
 * packet value for Gobi and QMI devices, ie. ignore usb interface number
 */
#define CDC_RSP_MASK_BE 0xFFFFFFFF00FFFFFFll
#define CDC_RSP_MASK_LE 0xFFFFFFE0FFFFFFFFll

const int i = 1;
#define is_bigendian() ( (*(char*)&i) == 0 )
#define CDC_GET_ENCAPSULATED_RESPONSE(pcdcrsp, pmask)\
{\
   *pcdcrsp  = is_bigendian() ? CDC_GET_ENCAPSULATED_RESPONSE_BE \
                          : CDC_GET_ENCAPSULATED_RESPONSE_LE ; \
   *pmask = is_bigendian() ? CDC_RSP_MASK_BE \
                           : CDC_RSP_MASK_LE; \
}

// CDC CONNECTION_SPEED_CHANGE indication packet
#define CDC_CONNECTION_SPEED_CHANGE_LE 0x2AA1ll
#define CDC_CONNECTION_SPEED_CHANGE_BE 0xA12A000000000000ll
/* The following masks filter the common part of the connection speed change
 * packet value for Gobi and QMI devices
 */
#define CDC_CONNSPD_MASK_BE 0xFFFFFFFFFFFF7FFFll
#define CDC_CONNSPD_MASK_LE 0XFFF7FFFFFFFFFFFFll
#define CDC_GET_CONNECTION_SPEED_CHANGE(pcdccscp, pmask)\
{\
   *pcdccscp  = is_bigendian() ? CDC_CONNECTION_SPEED_CHANGE_BE \
                          : CDC_CONNECTION_SPEED_CHANGE_LE ; \
   *pmask = is_bigendian() ? CDC_CONNSPD_MASK_BE \
                           : CDC_CONNSPD_MASK_LE; \
}

#define SPIN_LOCK_DEBUG 0

/*=========================================================================*/
// UserspaceQMIFops
//    QMI device's userspace file operations
/*=========================================================================*/
static const struct file_operations UserspaceQMIFops =
{
   .owner     = THIS_MODULE,
   .read      = UserspaceRead,
   .write     = UserspaceWrite,
#ifdef CONFIG_COMPAT
   .compat_ioctl = UserspaceunlockedIOCTL,
#endif
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,36 ))
   .unlocked_ioctl = UserspaceunlockedIOCTL,
#else
   .ioctl     = UserspaceIOCTL,
#endif
   .open      = UserspaceOpen,
   .flush     = UserspaceClose,
   .poll      = UserspacePoll,
   .release   = UserspaceRelease,
   .lock      = UserSpaceLock
};

int isPreempt(void)
{
   return in_atomic();
}

void GobiSyncRcu(void)
{
   if(isPreempt()!=0)
   {
      printk("preempt_disable");
      preempt_disable();
   }
   synchronize_rcu();
   smp_wmb();
   smp_rmb();
   smp_mb();
}
/*=========================================================================*/
// Generic functions
/*=========================================================================*/
u8 QMIXactionIDGet( sGobiUSBNet *pDev)
{
   u8 transactionID;

   if( 0 == (transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID)) )
   {
      transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
   }
   
   return transactionID;
}

static struct usb_endpoint_descriptor *GetEndpoint(
    struct usb_interface *pintf,
    int type,
    int dir )
{
   int i;
   struct usb_host_interface *iface = pintf->cur_altsetting;
   struct usb_endpoint_descriptor *pendp;

   for( i = 0; i < iface->desc.bNumEndpoints; i++)
   {
      pendp = &iface->endpoint[i].desc;
      if( ((pendp->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == dir)
          &&
          (usb_endpoint_type(pendp) == type) )
      {
         return pendp;
      }
   }

   return NULL;
}

int ForceFilpClose(struct file *pFilp)
{
   int iRet = -1;
   if (file_count(pFilp)>0)
   {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,9,0 ))
      if(file_inode(pFilp)!=NULL)
      {
         iRet = filp_close(pFilp, NULL);
      }
      else
      {
         printk("NULL Inode\n");
      }

#else
      iRet = filp_close(pFilp, NULL);
#endif
   }
   GobiSyncRcu();
   return iRet;
}

int LocalClientMemLockSpinIsLock( sGobiUSBNet * pDev)
{
    if(pDev!=NULL)
    {
        return spin_is_locked(&pDev->mQMIDev.mClientMemLock);
    }
    return 0;
}

unsigned long LocalClientMemLockSpinLockIRQSave( sGobiUSBNet * pDev, int line)
{
   #if SPIN_LOCK_DEBUG
   printk("%s :%d\n",__FUNCTION__,line);
   #endif

   if(pDev!=NULL)
   {
      unsigned long flags = 0;
      spin_lock_irqsave( &pDev->mQMIDev.mClientMemLock, flags);
      smp_wmb();
      #if SPIN_LOCK_DEBUG
      printk("%s :%d Locked\n",__FUNCTION__,line);
      #endif
      pDev->mQMIDev.mFlag = flags;
      return flags;
   }
   return 0;
}

int LocalClientMemUnLockSpinLockIRQRestore( sGobiUSBNet * pDev, unsigned long ulFlags, int line)
{
   if(pDev!=NULL)
   {
      unsigned long flags = pDev->mQMIDev.mFlag;
      if(LocalClientMemLockSpinIsLock(pDev)==0)
      {
         #if SPIN_LOCK_DEBUG
         printk("%s :%d Not Locked\n",__FUNCTION__,line);
         #endif
         return 0;
      }
      #if SPIN_LOCK_DEBUG
      printk("%s %d :%d\n",__FUNCTION__,__LINE__,line);
      #endif
      spin_unlock_irqrestore( &pDev->mQMIDev.mClientMemLock, flags );
   }
   else
   {
      #if SPIN_LOCK_DEBUG
      printk("%s %d :%d\n",__FUNCTION__,__LINE__,line);
      #endif
      local_irq_restore(ulFlags);
   }
   smp_mb();
   return 0;
}

/*===========================================================================
METHOD:
   IsDeviceValid (Public Method)

DESCRIPTION:
   Basic test to see if device memory is valid

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   bool
===========================================================================*/
bool IsDeviceValid( sGobiUSBNet * pDev )
{
   if (pDev == NULL)
   {
      return false;
   }

   if (pDev->mbQMIValid == false)
   {
      return false;
   }

   return true;
}

/*===========================================================================
METHOD:
   PrintHex (Public Method)

DESCRIPTION:
   Print Hex data, for debug purposes

PARAMETERS:
   pBuffer       [ I ] - Data buffer
   bufSize       [ I ] - Size of data buffer

RETURN VALUE:
   None
===========================================================================*/
void PrintHex(
   void *      pBuffer,
   u16         bufSize )
{
   char * pPrintBuf;
   u16 pos;
   int status;

   if (debug != 1)
   {
       return;
   }
   if(bufSize==(u16)(-1))
   {
       DBG( "No Data\n" );
   }
   pPrintBuf = kmalloc( bufSize * 3 + 1, GFP_ATOMIC );
   if (pPrintBuf == NULL)
   {
      DBG( "Unable to allocate buffer\n" );
      return;
   }
   memset( pPrintBuf, 0 , bufSize * 3 + 1 );

   for (pos = 0; pos < bufSize; pos++)
   {
      status = snprintf( (pPrintBuf + (pos * 3)),
                         4,
                         "%02X ",
                         *(u8 *)(pBuffer + pos) );
      if (status != 3)
      {
         DBG( "snprintf error %d\n", status );
         kfree( pPrintBuf );
         return;
      }
   }

   DBG( "   : %s\n", pPrintBuf );

   kfree( pPrintBuf );
   pPrintBuf = NULL;
   return;
}

/*===========================================================================
METHOD:
   GobiSetDownReason (Public Method)

DESCRIPTION:
   Sets mDownReason and turns carrier off

PARAMETERS
   pDev     [ I ] - Device specific memory
   reason   [ I ] - Reason device is down

RETURN VALUE:
   None
===========================================================================*/
void GobiSetDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason )
{
   set_bit( reason, &pDev->mDownReason );

   netif_carrier_off( pDev->mpNetDev->net );
}

/*===========================================================================
METHOD:
   GobiClearDownReason (Public Method)

DESCRIPTION:
   Clear mDownReason and may turn carrier on

PARAMETERS
   pDev     [ I ] - Device specific memory
   reason   [ I ] - Reason device is no longer down

RETURN VALUE:
   None
===========================================================================*/
void GobiClearDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason )
{
   clear_bit( reason, &pDev->mDownReason );

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,11,0 ))
    netif_carrier_on( pDev->mpNetDev->net );
#else
   if (pDev->mDownReason == 0)
   {
      netif_carrier_on( pDev->mpNetDev->net );
   }
#endif
}

/*===========================================================================
METHOD:
   GobiTestDownReason (Public Method)

DESCRIPTION:
   Test mDownReason and returns whether reason is set

PARAMETERS
   pDev     [ I ] - Device specific memory
   reason   [ I ] - Reason device is down

RETURN VALUE:
   bool
===========================================================================*/
bool GobiTestDownReason(
   sGobiUSBNet *    pDev,
   u8                 reason )
{
   return test_bit( reason, &pDev->mDownReason );
}

/*=========================================================================*/
// Driver level asynchronous read functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   ResubmitIntURB (Public Method)

DESCRIPTION:
   Resubmit interrupt URB, re-using same values

PARAMETERS
   pIntURB       [ I ] - Interrupt URB

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int ResubmitIntURB( struct urb * pIntURB )
{
   int status;
   int interval;

   // Sanity test
   if ( (pIntURB == NULL)
   ||   (pIntURB->dev == NULL) )
   {
      return -EINVAL;
   }

   // Interval needs reset after every URB completion
   // QC suggestion, 4ms per poll:
   //   bInterval 6 = 2^5 = 32 frames = 4 ms per poll
   interval = (pIntURB->dev->speed == USB_SPEED_HIGH) ?
                 6 : max((int)(pIntURB->ep->desc.bInterval), 3);

   // Reschedule interrupt URB
   usb_fill_int_urb( pIntURB,
                     pIntURB->dev,
                     pIntURB->pipe,
                     pIntURB->transfer_buffer,
                     pIntURB->transfer_buffer_length,
                     pIntURB->complete,
                     pIntURB->context,
                     interval );
   status = usb_submit_urb( pIntURB, GFP_ATOMIC );
   if (status != 0)
   {
      DBG( "Error re-submitting Int URB %d\n", status );
   }

   return status;
}

/*===========================================================================
METHOD:
   ReadCallback (Public Method)

DESCRIPTION:
   Put the data in storage and notify anyone waiting for data

PARAMETERS
   pReadURB       [ I ] - URB this callback is run for

RETURN VALUE:
   None
===========================================================================*/
void ReadCallback( struct urb * pReadURB )
{
   int result;
   u16 clientID;
   u16 RefclientID = 0;
   sClientMemList * pClientMem;
   void * pData = NULL;
   void * pDataCopy = NULL;
   u16 dataSize;
   sGobiUSBNet * pDev;
   unsigned long flags = 0;
   u16 transactionID;
   int iResult = 0;

   if (pReadURB == NULL)
   {
      DBG( "bad read URB\n" );
      return;
   }

   pDev = pReadURB->context;
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return;
   }

   del_timer(&pDev->read_tmr);

   if (pReadURB->status != 0)
   {
      DBG( "Read status = %d\n", pReadURB->status );
      if ((pReadURB->status == -ECONNRESET) && (pReadURB->actual_length > 0))
      {
          pDev->readTimeoutCnt++;
          // Read URB unlinked after receiving data, send data to client
          DBG( "Read URB timeout/kill after recv data\n" );
          printk(KERN_WARNING "Read URB timeout/kill, recv data len (%d), cnt (%d)\n",
                  pReadURB->actual_length, pDev->readTimeoutCnt);
      }
      else
      {
          // Resubmit the interrupt URB
          if (IsDeviceValid( pDev ) == false)
          {
             DBG( "Invalid device!\n" );
             return;
          }
          ResubmitIntURB( pDev->mQMIDev.mpIntURB );
          return;
      }
   }
   DBG( "Read %d bytes\n", pReadURB->actual_length );

   pData = pReadURB->transfer_buffer;
   dataSize = pReadURB->actual_length;

   PrintHex( pData, dataSize );

   result = ParseQMUX( &clientID,
                       pData,
                       dataSize );
   if (result < 0)
   {
      DBG( "Read error parsing QMUX %d\n", result );
      if (IsDeviceValid( pDev ) == false)
      {
         DBG( "Invalid device!\n" );
         return;
      }
      // Resubmit the interrupt URB
      ResubmitIntURB( pDev->mQMIDev.mpIntURB );

      return;
   }

   // Grab transaction ID

   // Data large enough?
   if (dataSize < result + 3)
   {
      DBG( "Data buffer too small to parse\n" );
      if (IsDeviceValid( pDev ) == false)
      {
         DBG( "Invalid device!\n" );
         return;
      }
      // Resubmit the interrupt URB
      ResubmitIntURB( pDev->mQMIDev.mpIntURB );

      return;
   }

   // Transaction ID size is 1 for QMICTL, 2 for others
   if (clientID == QMICTL)
   {
      transactionID = *(u8*)(pData + result + 1);
   }
   else
   {
      transactionID = le16_to_cpu( get_unaligned((u16*)(pData + result + 1)) );
   }

   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__ );

   // Find memory storage for this service and Client ID
   // Not using FindClientMem because it can't handle broadcasts
   pClientMem = pDev->mQMIDev.mpClientMemList;
   RefclientID = pClientMem->mClientID ;
   while (pClientMem != NULL)
   {
      if (pClientMem->mClientID == clientID
      ||  (pClientMem->mClientID | 0xff00) == clientID)
      {
         // Make copy of pData
         pDataCopy = kmalloc( dataSize, GFP_ATOMIC );
         if (pDataCopy == NULL)
         {
            DBG( "Error allocating client data memory\n" );

            // End critical section
            LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
            if (IsDeviceValid( pDev ) == false)
            {
             DBG( "Invalid device!\n" );
             return;
             }
            // Resubmit the interrupt URB
            ResubmitIntURB( pDev->mQMIDev.mpIntURB );

            return;             
         }

         memcpy( pDataCopy, pData, dataSize );

         if (AddToReadMemList( pDev,
                               pClientMem->mClientID,
                               transactionID,
                               pDataCopy,
                               dataSize ) == false)
         {
            DBG( "Error allocating pReadMemListEntry "
                 "read will be discarded\n" );
            kfree( pDataCopy );

            // End critical section
            if (IsDeviceValid( pDev ) == false)
            {
               DBG( "Invalid device!\n" );
               if(pDev)
               {
                  LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
               }
               else
               {
                  LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               }
               return;
            }
            LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
            // Resubmit the interrupt URB
            ResubmitIntURB( pDev->mQMIDev.mpIntURB );

            return;
         }

         // Success
         CLIENT_READMEM_SNAPSHOT(clientID, pDev);
         DBG( "Creating new readListEntry for client 0x%04X, TID %x\n",
              clientID,
              transactionID );

         // Notify this client data exists
         iResult = NotifyAndPopNotifyList( pDev,
                             pClientMem->mClientID,
                             transactionID );
         if (iResult==eNotifyListFound) 
          {
                DBG("%s:%d Found ClientID:0x%x , TID:0x%x\n",__FUNCTION__,__LINE__,pClientMem->mClientID,transactionID);
          }
          else if (iResult==eNotifyListEmpty) 
          {
                DBG("%s:%d Empty ClientID:0x%x , TID:0x%x\n",__FUNCTION__,__LINE__,pClientMem->mClientID,transactionID);
          }
          else 
          {
            DBG("%s:%d Not Found ClientID:0x%x , TID:0x%x\n",__FUNCTION__,__LINE__,pClientMem->mClientID,transactionID);
            if (IsDeviceValid( pDev ) == false)
            {
               DBG( "Invalid device!\n" );
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               return;
            }
            if(pDev->mbUnload >= eStatUnloading)
            {
               DBG( "Unload:%s\n", __FUNCTION__);
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               return ;
            }
          }
         
         // Not a broadcast
         if (clientID >> 8 != 0xff)
         {
            break;
         }
      }

      // Next element
      pClientMem = pClientMem->mpNext;
   }

   // End critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   if (IsDeviceValid( pDev ) == false)
    {
        DBG( "Invalid device!\n" );
        return;
    }
   // Resubmit the interrupt URB
   ResubmitIntURB( pDev->mQMIDev.mpIntURB );
}

void read_tmr_cb( struct urb * pReadURB )
{
  int result;

  DBG( "%s called (%ld).\n", __func__, jiffies );

  if ((pReadURB != NULL) && (pReadURB->status == -EINPROGRESS))
  {
     // Asynchronously unlink URB. On success, -EINPROGRESS will be returned, 
     // URB status will be set to -ECONNRESET, and ReadCallback() executed
     result = usb_unlink_urb( pReadURB );
     DBG( "%s called usb_unlink_urb, result = %d\n", __func__, result);
  }
}

/*===========================================================================
METHOD:
   IntCallback (Public Method)

DESCRIPTION:
   Data is available, fire off a read URB

PARAMETERS
   pIntURB       [ I ] - URB this callback is run for

RETURN VALUE:
   None
===========================================================================*/
void IntCallback( struct urb * pIntURB )
{
   int status;
   u64 CDCEncResp;
   u64 CDCEncRespMask;

   sGobiUSBNet * pDev = (sGobiUSBNet *)pIntURB->context;
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return;
   }

   // Verify this was a normal interrupt
   if (pIntURB->status != 0)
   {
        DBG( "IntCallback: Int status = %d\n", pIntURB->status );

      // Ignore EOVERFLOW errors
      if (pIntURB->status != -EOVERFLOW)
      {
         // Read 'thread' dies here
         return;
      }
      if(pIntURB->status<0)
      {
         return;
      }
   }
   else
   {
      //TODO cast transfer_buffer to struct usb_cdc_notification
      
      // CDC GET_ENCAPSULATED_RESPONSE
      CDC_GET_ENCAPSULATED_RESPONSE(&CDCEncResp, &CDCEncRespMask)

      DBG( "IntCallback: Encapsulated Response = 0x%llx\n",
          (*(u64*)pIntURB->transfer_buffer));

      //AR7554RD returned interrupt buffer not matching expected mask
      //thus, length check only
      if (pIntURB->actual_length == 8)

      {
         // Time to read
         usb_fill_control_urb( pDev->mQMIDev.mpReadURB,
                               pDev->mpNetDev->udev,
                               usb_rcvctrlpipe( pDev->mpNetDev->udev, 0 ),
                               (unsigned char *)pDev->mQMIDev.mpReadSetupPacket,
                               pDev->mQMIDev.mpReadBuffer,
                               DEFAULT_READ_URB_LENGTH,
                               ReadCallback,
                               pDev );
         setup_timer( &pDev->read_tmr, (void*)read_tmr_cb, (unsigned long)pDev->mQMIDev.mpReadURB );
         mod_timer( &pDev->read_tmr, jiffies + msecs_to_jiffies(USB_READ_TIMEOUT) );
         status = usb_submit_urb( pDev->mQMIDev.mpReadURB, GFP_ATOMIC );
         if (status != 0)
         {
            DBG( "Error submitting Read URB %d\n", status );
            if (IsDeviceValid( pDev ) == false)
            {
               DBG( "Invalid device!\n" );
               return;
            }
            // Resubmit the interrupt urb
            ResubmitIntURB( pIntURB );
            return;
         }

         // Int URB will be resubmitted during ReadCallback
         return;
      }
      // CDC CONNECTION_SPEED_CHANGE
      else if ((pIntURB->actual_length == 16)
           &&  (CDC_GET_CONNECTION_SPEED_CHANGE(&CDCEncResp, &CDCEncRespMask))
           &&  ((*(u64*)pIntURB->transfer_buffer & CDCEncRespMask) == CDCEncResp ) )
      {
         DBG( "IntCallback: Connection Speed Change = 0x%llx\n",
              (*(u64*)pIntURB->transfer_buffer));

         // if upstream or downstream is 0, stop traffic.  Otherwise resume it
         if ((*(u32*)(pIntURB->transfer_buffer + 8) == 0)
         ||  (*(u32*)(pIntURB->transfer_buffer + 12) == 0))
         {
            GobiSetDownReason( pDev, CDC_CONNECTION_SPEED );
            DBG( "traffic stopping due to CONNECTION_SPEED_CHANGE\n" );
         }
         else
         {
            GobiClearDownReason( pDev, CDC_CONNECTION_SPEED );
            DBG( "resuming traffic due to CONNECTION_SPEED_CHANGE\n" );
         }
      }
      else
      {
         DBG( "ignoring invalid interrupt in packet\n" );
         PrintHex( pIntURB->transfer_buffer, pIntURB->actual_length );
      }
   }
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return;
   }
   // Resubmit the interrupt urb
   ResubmitIntURB( pIntURB );

   return;
}

/*===========================================================================
METHOD:
   StartRead (Public Method)

DESCRIPTION:
   Start continuous read "thread" (callback driven)

   Note: In case of error, KillRead() should be run
         to remove urbs and clean up memory.

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int StartRead( sGobiUSBNet * pDev )
{
   int interval;
   struct usb_endpoint_descriptor *pendp;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   // Allocate URB buffers
   pDev->mQMIDev.mpReadURB = usb_alloc_urb( 0, GFP_KERNEL );
   if (pDev->mQMIDev.mpReadURB == NULL)
   {
      DBG( "Error allocating read urb\n" );
      return -ENOMEM;
   }

   pDev->mQMIDev.mpIntURB = usb_alloc_urb( 0, GFP_KERNEL );
   if (pDev->mQMIDev.mpIntURB == NULL)
   {
      DBG( "Error allocating int urb\n" );
      usb_free_urb( pDev->mQMIDev.mpReadURB );
      pDev->mQMIDev.mpReadURB = NULL;
      return -ENOMEM;
   }

   // Create data buffers
   pDev->mQMIDev.mpReadBuffer = kmalloc( DEFAULT_READ_URB_LENGTH, GFP_KERNEL );
   if (pDev->mQMIDev.mpReadBuffer == NULL)
   {
      DBG( "Error allocating read buffer\n" );
      usb_free_urb( pDev->mQMIDev.mpIntURB );
      pDev->mQMIDev.mpIntURB = NULL;
      usb_free_urb( pDev->mQMIDev.mpReadURB );
      pDev->mQMIDev.mpReadURB = NULL;
      return -ENOMEM;
   }

   pDev->mQMIDev.mpIntBuffer = kmalloc( DEFAULT_READ_URB_LENGTH, GFP_KERNEL );
   if (pDev->mQMIDev.mpIntBuffer == NULL)
   {
      DBG( "Error allocating int buffer\n" );
      kfree( pDev->mQMIDev.mpReadBuffer );
      pDev->mQMIDev.mpReadBuffer = NULL;
      usb_free_urb( pDev->mQMIDev.mpIntURB );
      pDev->mQMIDev.mpIntURB = NULL;
      usb_free_urb( pDev->mQMIDev.mpReadURB );
      pDev->mQMIDev.mpReadURB = NULL;
      return -ENOMEM;
   }

   pDev->mQMIDev.mpReadSetupPacket = kmalloc( sizeof( sURBSetupPacket ),
                                              GFP_KERNEL );
   if (pDev->mQMIDev.mpReadSetupPacket == NULL)
   {
      DBG( "Error allocating setup packet buffer\n" );
      kfree( pDev->mQMIDev.mpIntBuffer );
      pDev->mQMIDev.mpIntBuffer = NULL;
      kfree( pDev->mQMIDev.mpReadBuffer );
      pDev->mQMIDev.mpReadBuffer = NULL;
      usb_free_urb( pDev->mQMIDev.mpIntURB );
      pDev->mQMIDev.mpIntURB = NULL;
      usb_free_urb( pDev->mQMIDev.mpReadURB );
      pDev->mQMIDev.mpReadURB = NULL;
      return -ENOMEM;
   }

   // CDC Get Encapsulated Response packet
   pDev->mQMIDev.mpReadSetupPacket->mRequestType = 
       USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
   pDev->mQMIDev.mpReadSetupPacket->mRequestCode = GET_ENCAPSULATED_RESPONSE;
   pDev->mQMIDev.mpReadSetupPacket->mValue = 0;
   pDev->mQMIDev.mpReadSetupPacket->mIndex =
      cpu_to_le16(pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber);  /* interface number */
   pDev->mQMIDev.mpReadSetupPacket->mLength = cpu_to_le16(DEFAULT_READ_URB_LENGTH);

   pendp = GetEndpoint(pDev->mpIntf, USB_ENDPOINT_XFER_INT, USB_DIR_IN);
   if (pendp == NULL)
   {
      DBG( "Invalid interrupt endpoint!\n" );
      kfree(pDev->mQMIDev.mpReadSetupPacket);
      pDev->mQMIDev.mpReadSetupPacket = NULL;
      kfree( pDev->mQMIDev.mpIntBuffer );
      pDev->mQMIDev.mpIntBuffer = NULL;
      kfree( pDev->mQMIDev.mpReadBuffer );
      pDev->mQMIDev.mpReadBuffer = NULL;
      usb_free_urb( pDev->mQMIDev.mpIntURB );
      pDev->mQMIDev.mpIntURB = NULL;
      usb_free_urb( pDev->mQMIDev.mpReadURB );
      pDev->mQMIDev.mpReadURB = NULL;
      return -ENXIO;
   }

   // Interval needs reset after every URB completion
   interval = (pDev->mpNetDev->udev->speed == USB_SPEED_HIGH) ?
                 6 : max((int)(pendp->bInterval), 3);

   // Schedule interrupt URB
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   usb_fill_int_urb( pDev->mQMIDev.mpIntURB,
                     pDev->mpNetDev->udev,
                     /* QMI interrupt endpoint for the following
                      * interface configuration: DM, NMEA, MDM, NET
                      */
                     usb_rcvintpipe( pDev->mpNetDev->udev,
                                     pendp->bEndpointAddress),
                     pDev->mQMIDev.mpIntBuffer,
                     le16_to_cpu(pendp->wMaxPacketSize),
                     IntCallback,
                     pDev,
                     interval );
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   return usb_submit_urb( pDev->mQMIDev.mpIntURB, GFP_ATOMIC );
}

/*===========================================================================
METHOD:
   KillRead (Public Method)

DESCRIPTION:
   Kill continuous read "thread"

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
void KillRead( sGobiUSBNet * pDev )
{

   if(pDev ==NULL)
   {
      DBG( "pDev NULL\n" );
      return ;
   }

   // Stop reading
   if (pDev->mQMIDev.mpReadURB != NULL)
   {
      DBG( "Killng read URB\n" );
      usb_kill_urb( pDev->mQMIDev.mpReadURB );
   }

   if (pDev->mQMIDev.mpIntURB != NULL)
   {
      DBG( "Killng int URB\n" );
      usb_kill_urb( pDev->mQMIDev.mpIntURB );
   }

   // Release buffers
   kfree( pDev->mQMIDev.mpReadSetupPacket );
   pDev->mQMIDev.mpReadSetupPacket = NULL;
   kfree( pDev->mQMIDev.mpReadBuffer );
   pDev->mQMIDev.mpReadBuffer = NULL;
   kfree( pDev->mQMIDev.mpIntBuffer );
   pDev->mQMIDev.mpIntBuffer = NULL;

   // Release URB's
   usb_free_urb( pDev->mQMIDev.mpReadURB );
   pDev->mQMIDev.mpReadURB = NULL;
   usb_free_urb( pDev->mQMIDev.mpIntURB );
   pDev->mQMIDev.mpIntURB = NULL;
}

/*===========================================================================
METHOD:
   InitSemID (Public Method)

DESCRIPTION:
   Initialize Read Sync tasks semaphore

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/

void InitSemID(sGobiUSBNet * pDev)
{
   int i = 0;
   if(pDev==NULL)
   {
      DBG("%s NULL\n",__FUNCTION__);
      return ;
   }
   sema_init( &(pDev->ReadsyncSem), SEMI_INIT_DEFAULT_VALUE );
   up(&(pDev->ReadsyncSem));

   for(i=0;i<MAX_READ_SYNC_TASK_ID;i++)
   {
     pDev->iReasSyncTaskID[i]=-__LINE__;
     sema_init( &(pDev->readSem[i]), SEMI_INIT_DEFAULT_VALUE );
   }
   up(&(pDev->ReadsyncSem));
}

/*===========================================================================
METHOD:
   StopSemID (Public Method)

DESCRIPTION:
   Release all Read Sync tasks semaphore(s)

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
void StopSemID(sGobiUSBNet *pDev)
{
   int i = 0;
   if(pDev==NULL)
   {
      DBG("%s NULL\n",__FUNCTION__);
      return ;
   }

   while(!down_trylock( &(pDev->ReadsyncSem) ))
   {
      if(pDev->iTaskID>0)
      if(signal_pending(current))
      {
            return ;
      }
      i++;
      if(i>MAX_RETRY_LOCK_NUMBER)
      {
         DBG("%s Get ReadSyncID Timeout\n",__FUNCTION__);
         return ;
      }

      set_current_state(TASK_INTERRUPTIBLE);
      wait_ms(MAX_RETRY_LOCK_MSLEEP_TIME);
      if(signal_pending(current))
      {
         set_current_state(TASK_RUNNING);
         return ;
      }

      if(pDev==NULL)
      {
         set_current_state(TASK_RUNNING);
         return ;
      }
   }
   set_current_state(TASK_RUNNING);
   for(i=0;i<MAX_READ_SYNC_TASK_ID;i++)
   {
      if(pDev->iReasSyncTaskID[i]>0)
      {
         up(&(pDev->readSem[i]));
         pDev->iReasSyncTaskID[i]=-__LINE__;
      }
   }
   up(&(pDev->ReadsyncSem));
   DBG("%s DONE\n",__FUNCTION__);
}

/*===========================================================================
METHOD:
   iGetSemID (Public Method)

DESCRIPTION:
   Get free read sync semaphore slot.

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   Free semaphore slot.
===========================================================================*/
int iGetSemID(sGobiUSBNet *pDev,int line)
{
   int i = 0;
   int Ret = -1;
   
   
   while(!down_trylock( &(pDev->ReadsyncSem) ))
   {
      if(pDev->iTaskID>0)
      if(kthread_should_stop() || signal_pending(current))
      {
            return -1;
      }
      i++;
      if(i>MAX_RETRY_LOCK_NUMBER)
      {
         DBG("%s Get ReadSyncID Timeout\n",__FUNCTION__);
         return -1;
      }
      set_current_state(TASK_INTERRUPTIBLE);
      wait_ms(MAX_RETRY_LOCK_MSLEEP_TIME);
      if(signal_pending(current))
      {
         set_current_state(TASK_RUNNING);
         return -1;
      }
      
   }
   set_current_state(TASK_RUNNING);
   for(i=0;i<MAX_READ_SYNC_TASK_ID;i++)
   {
      DBG("%s : iReasSyncTaskID[%d]:%d ret:%d\n",__FUNCTION__,i,pDev->iReasSyncTaskID[i],Ret);
      if(pDev->iReasSyncTaskID[i]<0)
      {
         pDev->iReasSyncTaskID[i]=line;
         Ret = i;
         break;
      }
   }
   up(&(pDev->ReadsyncSem));
   return Ret;
}

/*=========================================================================*/
// Internal read/write functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   ReadAsync (Public Method)

DESCRIPTION:
   Start asynchronous read
   NOTE: Reading client's data store, not device

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any
   pCallback         [ I ] - Callback to be executed when data is available
   pData             [ I ] - Data buffer that willl be passed (unmodified)
                             to callback

RETURN VALUE:
   int - 0 for success
         negative errno for failure
===========================================================================*/
int ReadAsync(
   sGobiUSBNet *    pDev,
   u16                clientID,
   u16                transactionID,
   void               (*pCallback)(sGobiUSBNet*, u16, void *),
   void *             pData )
{
   sClientMemList * pClientMem = NULL;
   sReadMemList ** ppReadMemList = NULL;
   unsigned long flags;
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

   // Find memory storage for this client ID
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find matching client ID 0x%04X\n",
           clientID );

      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      return -ENXIO;
   }

   ppReadMemList = &(pClientMem->mpList);

   // Does data already exist?
   while (*ppReadMemList != NULL)
   {
      // Is this element our data?
      if (transactionID == 0
      ||  transactionID == (*ppReadMemList)->mTransactionID)
      {
         // End critical section
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

         // Run our own callback
         pCallback( pDev, clientID, pData );

         return 0;
      }

      // Next
      ppReadMemList = &(*ppReadMemList)->mpNext;
   }

   // Data not found, add ourself to list of waiters
   if (AddToNotifyList( pDev,
                        clientID,
                        transactionID,
                        pCallback,
                        pData ) == false)
   {
      DBG( "Unable to register for notification\n" );
   }

   // End critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

   // Success
   return 0;
}

/*===========================================================================
METHOD:
   UpSem (Public Method)

DESCRIPTION:
   Notification function for synchronous read

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   pData             [ I ] - Buffer that holds semaphore to be up()-ed

RETURN VALUE:
   None
===========================================================================*/
void UpSem(
   sGobiUSBNet * pDev,
   u16             clientID,
   void *          pData )
{
   DBG( "0x%04X\n", clientID );
   if(pData!=NULL)
   {
      up( (struct semaphore *)pData );
   }
   return;
}

/*===========================================================================
METHOD:
   ReadSync (Public Method)

DESCRIPTION:
   Start synchronous read
   NOTE: Reading client's data store, not device

PARAMETERS:
   pDev              [ I ] - Device specific memory
   ppOutBuffer       [I/O] - On success, will be filled with a
                             pointer to read buffer
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any

RETURN VALUE:
   int - size of data read for success
         negative errno for failure
===========================================================================*/
int ReadSync(
   sGobiUSBNet *    pDev,
   void **            ppOutBuffer,
   u16                clientID,
   u16                transactionID,
   int                *iID,
   struct semaphore   *pReadSem,
   int                *iIsClosing)
{
   int result;
   sClientMemList * pClientMem;
   sNotifyList ** ppNotifyList, * pDelNotifyListEntry;
   void * pData = NULL;
   u16 dataSize;
   struct semaphore *pLocalreadSem = NULL;
   unsigned long flags;
   DBG("\n");
   if(pReadSem==NULL)
   if(*iID<0)
   {
      DBG( "Could not find matching SemID\n");
      return -ENXIO;
   }

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }
   if (pDev->mbUnload >= eStatUnloading)
   {
      DBG( "unloaded\n" );
      return -EFAULT;
   }

   
   if(pReadSem==NULL)
   {
      pLocalreadSem = &(pDev->readSem[*iID]);
   }
   else
   {
     pLocalreadSem = pReadSem;
   }

   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

   // Find memory storage for this Client ID
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find matching client ID 0x%04X\n",
           clientID );

      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      return -ENXIO;
   }

   // Note: in cases where read is interrupted,
   //    this will verify client is still valid
   while (PopFromReadMemList( pDev,
                              clientID,
                              transactionID,
                              &pData,
                              &dataSize ) == false)
   {
      // Data does not yet exist, wait
       
      if(pDev->mbUnload >= eStatUnloading)
      {
         DBG("Unloading");
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         return -ENXIO;
      }
      // Add ourself to list of waiters
      if (AddToNotifyList( pDev,
                           clientID,
                           transactionID,
                           UpSem,
                           pLocalreadSem ) == false)
      {
         DBG( "unable to register for notification\n" );
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         return -EFAULT;
      }

      // End critical section while we block
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

      // Wait for notification
      if (signal_pending(current))
      {
         return -ERESTARTSYS;
      }
      result = down_interruptible( pLocalreadSem);
      DBG("result:%d\n",result);
      if (IsDeviceValid( pDev ) == false)
      {
         DBG( "Invalid device!\n" );
         return -EFAULT;
      }
      if (pDev->mbUnload > eStatUnloading)
      {
         DBG( "unloaded\n" );
          if(pReadSem==NULL)
          pDev->iReasSyncTaskID[*iID] = -__LINE__;
          else
          *iID = -__LINE__;
           return -EFAULT;
      }
      if(iIsClosing!=NULL)
      {
         if(*iIsClosing>0)
         {
           DBG( "filp is closing\n" );
           if(pReadSem==NULL)
           pDev->iReasSyncTaskID[*iID] = -__LINE__;
           else
           *iID = -__LINE__;
           flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
           RemoveAndPopNotifyList(pDev,clientID,transactionID);
           LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
           return result;
         }
      }
      if (result < 0)
      {
         if(pDev)
         {
            if(pReadSem==NULL)
            pDev->iReasSyncTaskID[*iID] = -__LINE__;
            else
            *iID = -__LINE__;
         }
         flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
         RemoveAndPopNotifyList(pDev,clientID,transactionID);
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         return result;//-EFAULT;EINTR resume error
      }
      if(*iID<-1)
      {
         DBG( "%s:%d Interrupted %d iID:%d\n",__FUNCTION__,__LINE__, result,*iID );
         return -EFAULT;
      }
      if (signal_pending(current))
      {
         return -ERESTARTSYS;
      }
      if (result != 0)
      {
         DBG( "Interrupted %d\n", result );

         // readSem will fall out of scope,
         // remove from notify list so it's not referenced
         if(pDev==NULL)
         {
            return -EFAULT;
         }
         if(pDev->iIsClosing)
         {
            DBG( "Closing device!\n" );
            if(pDev)
            {
                if(pReadSem==NULL)
                pDev->iReasSyncTaskID[*iID] = -__LINE__;
                else
                *iID = -__LINE__;
            }
            return -EFAULT;
         }
         if(pDev->mbUnload >= eStatUnloading)
         {
            DBG("Unloading");
            if(pReadSem==NULL)
            pDev->iReasSyncTaskID[*iID] = -__LINE__;
            else
            *iID = -__LINE__;
            return -ENXIO;
         }
         flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
         if(pDev)
         {
            flags = pDev->mQMIDev.mFlag;
         }
         ppNotifyList = &(pClientMem->mpReadNotifyList);
         pDelNotifyListEntry = NULL;

         // Find and delete matching entry
         while (*ppNotifyList != NULL)
         {
            if (IsDeviceValid( pDev ) == false)
            {
               DBG( "Invalid device!\n" );
               /* SWI_START */
               /* workaround to leave the system in cleaner state:
                * must enable interrupts and enable pre-emption.
                * TBD what the correct action should be when the device is gone. 
                */
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               //preempt_enable();
                if(pReadSem==NULL)
                pDev->iReasSyncTaskID[*iID] = -__LINE__;
                else
                *iID = -__LINE__;
               /* SWI_STOP */
               return -EFAULT;
            }
            if(*iID<0)
            {
               DBG( "Invalid device!\n" );
               /* SWI_START */
               /* must restore irq, pre-emption, locks before returning */
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               if(pReadSem==NULL)
               {
                  if(pDev)
                  pDev->iReasSyncTaskID[*iID] = -__LINE__;
               }
               else
               *iID = -__LINE__;
               /* SWI_STOP */
               return -EFAULT;
            }
            if(pDev==NULL)
            {
               DBG( "Invalid device!\n" );
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               return -EFAULT;
            }
            if (pDev->mbUnload > eStatUnloading)
            {
               DBG( "UNLOADING!\n" );
               LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
               if(pReadSem==NULL)
               pDev->iReasSyncTaskID[*iID] = -__LINE__;
               else
               *iID = -__LINE__;
               return -EFAULT;
            }
            if(ppNotifyList==NULL)
            {
                DBG( "UNLOADING!\n" );
                LocalClientMemUnLockSpinLockIRQRestore(pDev,flags,__LINE__);
                if(pReadSem==NULL)
                pDev->iReasSyncTaskID[*iID] = -__LINE__;
                else
                *iID = -__LINE__;
                return -EFAULT;
            }
            if ((*ppNotifyList)->mpData == 
                  pLocalreadSem)
            {
               pDelNotifyListEntry = *ppNotifyList;
               *ppNotifyList = (*ppNotifyList)->mpNext;
               kfree( pDelNotifyListEntry );
               break;
            }

            // Next
            ppNotifyList = &(*ppNotifyList)->mpNext;
         }

         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         if(pDev)
        {
            if(pReadSem==NULL)
            pDev->iReasSyncTaskID[*iID] = -__LINE__;
            else
            *iID = -__LINE__;
        }
         return -EINTR;
      }

      // Verify device is still valid
      if (IsDeviceValid( pDev ) == false)
      {
         DBG( "Invalid device!\n" );
         if(pDev)
         {
             if(pReadSem==NULL)
             pDev->iReasSyncTaskID[*iID] = -__LINE__;
             else
             *iID = -__LINE__;
         }
         return -ENXIO;
      }

      // Restart critical section and continue loop
      flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   }

   // End Critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

   // Success
   *ppOutBuffer = pData;
   if(pDev)
   {
      if(pReadSem==NULL)
      pDev->iReasSyncTaskID[*iID] = -__LINE__;
      else
      *iID = -__LINE__;
   }
   return dataSize;
}

/*===========================================================================
METHOD:
   WriteSync (Public Method)

DESCRIPTION:
   Start synchronous write

PARAMETERS:
   pDev                 [ I ] - Device specific memory
   pWriteBuffer         [ I ] - Data to be written
   writeBufferSize      [ I ] - Size of data to be written
   clientID             [ I ] - Client ID of requester

RETURN VALUE:
   int - write size (includes QMUX)
         negative errno for failure
===========================================================================*/
int WriteSync(
   sGobiUSBNet *          pDev,
   char *                 pWriteBuffer,
   int                    writeBufferSize,
   u16                    clientID )
{
   int i;
   int result;
   int iLockRetry =0;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   if (pDev->mbUnload >= eStatUnloading)
   {
      DBG( "Unloading device!\n" );
      return -ENXIO;
   }
   // Fill writeBuffer with QMUX
   result = FillQMUX( clientID, pWriteBuffer, writeBufferSize );
   if (result < 0)
   {
      return result;
   }

   // Wake device
   result = usb_autopm_get_interface( pDev->mpIntf );
   if (result < 0)
   {
      DBG( "unable to resume interface: %d\n", result );

      // Likely caused by device going from autosuspend -> full suspend
      if (result == -EPERM)
      {
#ifdef CONFIG_PM
#if (LINUX_VERSION_CODE < KERNEL_VERSION( 2,6,33 ))
         pDev->mpNetDev->udev->auto_pm = 0;
#endif
         GobiNetSuspend( pDev->mpIntf, PMSG_SUSPEND );
#endif /* CONFIG_PM */
      }
      return result;
   }

   DBG( "Actual Write:\n" );
   PrintHex( pWriteBuffer, writeBufferSize );

   // Write Control URB, protect with read semaphore to track in-flight USB control writes in case of disconnect
   for(i=0;i<USB_WRITE_RETRY;i++)
   {
       
      if(isModuleUnload(pDev))
      {
         DBG( "unloaded\n" );
         return -EFAULT;
      }
      pDev->iShutdown_read_sem= __LINE__;
      if(signal_pending(current))
      {
         return -ERESTARTSYS;
      }

      iLockRetry = 0;
      while(down_read_trylock(&(pDev->shutdown_rwsem))!=1)
      {
         wait_ms(5);
         if(iLockRetry++>100)
         {
            DBG("down_read_trylock timeout");
            return -EFAULT;
         }
         if(pDev==NULL)
         {
            DBG( "NULL\n" );
            return -EFAULT;
         }
         if (pDev->mbUnload >= eStatUnloading)
         {
            DBG( "unloaded\n" );
            return -EFAULT;
         }
      }
      smp_mb();
      result = usb_control_msg( pDev->mpNetDev->udev, usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
             SEND_ENCAPSULATED_COMMAND,
             USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE,
             0, pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
             (void*)pWriteBuffer, writeBufferSize,
             USB_WRITE_TIMEOUT );
       if(signal_pending(current))
       {
          return -ERESTARTSYS;
       }
       if(pDev==NULL)
       {
          return -EFAULT;
       }
       up_read(&pDev->shutdown_rwsem);
       pDev->iShutdown_read_sem=- __LINE__;
       
       if (pDev->mbUnload >= eStatUnloading)
       {
          DBG( "unloaded\n" );
          return -EFAULT;
       }

       if (signal_pending(current))
       {
           return -ERESTARTSYS;
       }

       if (result < 0)
       {
          printk(KERN_WARNING "usb_control_msg failed (%d)", result);
       }
       // Control write transfer may occasionally timeout with certain HCIs, attempt a second time before reporting an error
       if (result == -ETIMEDOUT)
       {
           pDev->writeTimeoutCnt++;
           printk(KERN_WARNING "Write URB timeout, cnt(%d)\n", pDev->writeTimeoutCnt);
       }
       else if(result < 0 )
       {
          DBG( "%s no device!\n" ,__FUNCTION__);
           return result;
       }
       else
       {
           break;
       }
       if (IsDeviceValid( pDev ) == false)
       {
          DBG( "%s Invalid device!\n" ,__FUNCTION__);
          return -ENXIO;
       }
       if (pDev->mbUnload > eStatUnloading)
       {
         DBG( "unloaded\n" );
         return -EFAULT;
       }
   }

   // Write is done, release device
   usb_autopm_put_interface( pDev->mpIntf );


   return result;
}

/*=========================================================================*/
// Internal memory management functions
/*=========================================================================*/

/*===========================================================================
METHOD:
   GetClientID (Public Method)

DESCRIPTION:
   Request a QMI client for the input service type and initialize memory
   structure

PARAMETERS:
   pDev           [ I ] - Device specific memory
   serviceType    [ I ] - Desired QMI service type

RETURN VALUE:
   int - Client ID for success (positive)
         Negative errno for error
===========================================================================*/
int GetClientID(
   sGobiUSBNet *    pDev,
   u8                 serviceType ,
   struct semaphore   *pReadSem)
{
   u16 clientID;
   sClientMemList ** ppClientMem;
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u8 transactionID;
   int iID = -1;
   unsigned long flags;
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device!\n" );
      return -ENXIO;
   }

   // Run QMI request to be asigned a Client ID
   if (serviceType != 0)
   {
      writeBufferSize = QMICTLGetClientIDReqSize();
      pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
      if (pWriteBuffer == NULL)
      {
         return -ENOMEM;
      }

      /* transactionID cannot be 0 */
      transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
      if (transactionID == 0)
      {
         transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
      }
      if (transactionID != 0)
      {
         result = QMICTLGetClientIDReq( pWriteBuffer,
                                        writeBufferSize,
                                        transactionID,
                                        serviceType );
         if (result < 0)
         {
            kfree( pWriteBuffer );
            return result;
         }
      }
      else
      {
         kfree( pWriteBuffer );
         DBG( "Invalid transaction ID!\n" );
         return EINVAL;
      }
      if(pReadSem==NULL)
      iID = iGetSemID(pDev,__LINE__);
      result = WriteSync( pDev,
                          pWriteBuffer,
                          writeBufferSize,
                          QMICTL );
      kfree( pWriteBuffer );

      if (result < 0)
      {
         if(pReadSem==NULL)
         {
            pDev->iReasSyncTaskID[iID] = -__LINE__;
         }
         return result;
      }

      result = ReadSync( pDev,
                         &pReadBuffer,
                         QMICTL,
                         transactionID,
                         &iID,pReadSem,NULL);
      if (result < 0)
      {
         DBG( "bad read data %d\n", result );
         return result;
      }
      readBufferSize = result;

      result = QMICTLGetClientIDResp( pReadBuffer,
                                      readBufferSize,
                                      &clientID );

     /* Upon return from QMICTLGetClientIDResp, clientID
      * low address contains the Service Number (SN), and
      * clientID high address contains Client Number (CN)
      * For the ReadCallback to function correctly,we swap
      * the SN and CN on a Big Endian architecture.
      */
      clientID = le16_to_cpu(clientID);
      if(pReadBuffer)
      kfree( pReadBuffer );

      if (result < 0)
      {
         return result;
      }
   }
   else
   {
      // QMI CTL will always have client ID 0
      clientID = 0;
   }

   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

   // Verify client is not already allocated
   if (FindClientMem( pDev, clientID ) != NULL)
   {
      DBG( "Client memory already exists\n" );

      // End Critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      return -ETOOMANYREFS;
   }

   // Go to last entry in client mem list
   ppClientMem = &pDev->mQMIDev.mpClientMemList;
   while (*ppClientMem != NULL)
   {
      ppClientMem = &(*ppClientMem)->mpNext;
   }

   // Create locations for read to place data into
   *ppClientMem = kmalloc( sizeof( sClientMemList ), GFP_ATOMIC );
   if (*ppClientMem == NULL)
   {
      DBG( "Error allocating read list\n" );

      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
      return -ENOMEM;
   }

   (*ppClientMem)->mClientID = clientID;
   (*ppClientMem)->mpList = NULL;
   (*ppClientMem)->mpReadNotifyList = NULL;
   (*ppClientMem)->mpURBList = NULL;
   (*ppClientMem)->mpNext = NULL;

   // Initialize workqueue for poll()
   init_waitqueue_head( &(*ppClientMem)->mWaitQueue );

   // End Critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

   return (int)( (*ppClientMem)->mClientID );
}

/*===========================================================================
METHOD:
   ReleaseClientID (Public Method)

DESCRIPTION:
   Release QMI client and free memory

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID

RETURN VALUE:
   None
===========================================================================*/
bool ReleaseClientID(
   sGobiUSBNet *    pDev,
   u16                clientID ,
   struct semaphore   *pReadSem)
{
   int result;
   sClientMemList ** ppDelClientMem;
   sClientMemList * pNextClientMem;
   void * pDelData = NULL;
   u16 dataSize;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer = NULL;
   u16 readBufferSize;
   u8 transactionID;
   unsigned long flags;
   // Is device is still valid?
   DBG("clientID:0x%x\n",clientID);
   if (pDev->mbUnload > eStatUnloaded)
   {
      DBG( "unloaded\n" );
      return false;
   }

   // Run QMI ReleaseClientID if this isn't QMICTL
   if (clientID != QMICTL)
   {
      // Note: all errors are non fatal, as we always want to delete
      //    client memory in latter part of function

      writeBufferSize = QMICTLReleaseClientIDReqSize();
      pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
      if (pWriteBuffer == NULL)
      {
         DBG( "memory error\n" );
         return false;
      }
      else
      {
         transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
         if (transactionID == 0)
         {
            transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
         }
         result = QMICTLReleaseClientIDReq( pWriteBuffer,
                                            writeBufferSize,
                                            transactionID,
                                            clientID );
         if (result < 0)
         {
            kfree( pWriteBuffer );
            DBG( "error %d filling req buffer\n", result );
         }
         else
         {
            {
               int iID = 0;
               struct semaphore *pLocalSem = pReadSem;
               if(pReadSem==NULL)
               {
                  iID = iGetSemID(pDev,__LINE__);
               }
               result = WriteSync( pDev,
                                pWriteBuffer,
                                writeBufferSize,
                                QMICTL );
               kfree( pWriteBuffer );

               if (result < 0)
               {
                  pDev->iReasSyncTaskID[iID] = -__LINE__;
                  DBG( "bad write status %d\n", result );
                  return false;
               }
               else
               {
                  result = ReadSync( pDev,
                                     &pReadBuffer,
                                     QMICTL,
                                     transactionID,
                                     &iID,pLocalSem,NULL);
                  if (result < 0)
                  {
                     if(pLocalSem==NULL)
                     pDev->iReasSyncTaskID[iID] = -__LINE__;
                     if(pReadBuffer)
                     kfree( pReadBuffer );
                  }
                  else
                  {
                     readBufferSize = result;

                     result = QMICTLReleaseClientIDResp( pReadBuffer,
                                                         readBufferSize );
                     if(pReadBuffer)
                     kfree( pReadBuffer );

                     if (result < 0)
                     {
                        DBG( "error %d parsing response\n", result );
                        return false;
                     }
                  }
               }
            }
         } 
      }
   }
   
   // Cleaning up client memory

   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

   // Can't use FindClientMem, I need to keep pointer of previous
   ppDelClientMem = &pDev->mQMIDev.mpClientMemList;
   while (*ppDelClientMem != NULL)
   {
      if ((*ppDelClientMem)->mClientID == clientID)
      {
         pNextClientMem = (*ppDelClientMem)->mpNext;

         // Notify all clients
         while (NotifyAndPopNotifyList( pDev,
                                        clientID,
                                        0 ) == eNotifyListFound );
         // Free any unread data
         while (PopFromReadMemList( pDev,
                                    clientID,
                                    0,
                                    &pDelData,
                                    &dataSize ) == true )
         {
            kfree( pDelData );
            pDelData = NULL;
         }
         DBG("Delete client Mem\r\n");
         // Delete client Mem
         kfree( *ppDelClientMem );
         *ppDelClientMem = NULL;

         // Overwrite the pointer that was to this client mem
         *ppDelClientMem = pNextClientMem;
      }
      else
      {
         // I now point to (a pointer of ((the node I was at)'s mpNext))
          if(*ppDelClientMem==NULL)
          {
              DBG("ppDelClientMem NULL %d\r\n",__LINE__);
              break;
          }
         ppDelClientMem = &(*ppDelClientMem)->mpNext;
      }
   }

   // End Critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   return true;
}

/*===========================================================================
METHOD:
   FindClientMem (Public Method)

DESCRIPTION:
   Find this client's memory

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID

RETURN VALUE:
   sClientMemList - Pointer to requested sClientMemList for success
                    NULL for error
===========================================================================*/
sClientMemList * FindClientMem(
   sGobiUSBNet *      pDev,
   u16              clientID )
{
   sClientMemList * pClientMem;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return NULL;
   }

#ifdef CONFIG_SMP
   // Verify Lock
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   pClientMem = pDev->mQMIDev.mpClientMemList;
   while (pClientMem != NULL)
   {
      if (pClientMem->mClientID == clientID)
      {
         // Success
         DBG("Found client's 0x%x memory\n", clientID);
         return pClientMem;
      }

      pClientMem = pClientMem->mpNext;
   }

   DBG( "Could not find client mem 0x%04X\n", clientID );
   return NULL;
}

/*===========================================================================
METHOD:
   AddToReadMemList (Public Method)

DESCRIPTION:
   Add Data to this client's ReadMem list

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev           [ I ] - Device specific memory
   clientID       [ I ] - Requester's client ID
   transactionID  [ I ] - Transaction ID or 0 for any
   pData          [ I ] - Data to add
   dataSize       [ I ] - Size of data to add

RETURN VALUE:
   bool
===========================================================================*/
bool AddToReadMemList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID,
   void *           pData,
   u16              dataSize )
{
   sClientMemList * pClientMem;
   sReadMemList ** ppThisReadMemList;

#ifdef CONFIG_SMP
   // Verify Lock
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n",
           clientID );

      return false;
   }

   // Go to last ReadMemList entry
   ppThisReadMemList = &pClientMem->mpList;
   while (*ppThisReadMemList != NULL)
   {
      ppThisReadMemList = &(*ppThisReadMemList)->mpNext;
   }

   *ppThisReadMemList = kmalloc( sizeof( sReadMemList ), GFP_ATOMIC );
   if (*ppThisReadMemList == NULL)
   {
      DBG( "Mem error\n" );

      return false;
   }

   (*ppThisReadMemList)->mpNext = NULL;
   (*ppThisReadMemList)->mpData = pData;
   (*ppThisReadMemList)->mDataSize = dataSize;
   (*ppThisReadMemList)->mTransactionID = transactionID;

   return true;
}

/*===========================================================================
METHOD:
   PopFromReadMemList (Public Method)

DESCRIPTION:
   Remove data from this client's ReadMem list if it matches
   the specified transaction ID.

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any
   ppData            [I/O] - On success, will be filled with a
                             pointer to read buffer
   pDataSize         [I/O] - On succces, will be filled with the
                             read buffer's size

RETURN VALUE:
   bool
===========================================================================*/
bool PopFromReadMemList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID,
   void **          ppData,
   u16 *            pDataSize )
{
   sClientMemList * pClientMem;
   sReadMemList * pDelReadMemList, ** ppReadMemList;
   DBG("");
#ifdef CONFIG_SMP
   // Verify Lock
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n",
           clientID );

      return false;
   }

   ppReadMemList = &(pClientMem->mpList);
   pDelReadMemList = NULL;

   // Find first message that matches this transaction ID
   CLIENT_READMEM_SNAPSHOT(clientID, pDev);
   while (*ppReadMemList != NULL)
   {
      // Do we care about transaction ID?
      if (transactionID == 0
      ||  transactionID == (*ppReadMemList)->mTransactionID )
      {
         pDelReadMemList = *ppReadMemList;
         DBG(  "*ppReadMemList = 0x%p pDelReadMemList = 0x%p\n",
               *ppReadMemList, pDelReadMemList );
         break;
      }

      DBG( "skipping 0x%04X data TID = %x\n", clientID, (*ppReadMemList)->mTransactionID );

      // Next
      ppReadMemList = &(*ppReadMemList)->mpNext;
   }
   DBG(  "*ppReadMemList = 0x%p pDelReadMemList = 0x%p\n",
         *ppReadMemList, pDelReadMemList );
   if (pDelReadMemList != NULL)
   {
       if(*ppReadMemList==NULL)
       {
           DBG("%d\r\n",__LINE__);
           return false;
       }
      *ppReadMemList = (*ppReadMemList)->mpNext;

      // Copy to output
      *ppData = pDelReadMemList->mpData;
      *pDataSize = pDelReadMemList->mDataSize;
      DBG(  "*ppData = 0x%p pDataSize = %u\n",
            *ppData, *pDataSize );

      // Free memory
      kfree( pDelReadMemList );

      return true;
   }
   else
   {
      DBG( "No read memory to pop, Client 0x%04X, TID = 0x%x\n",
           clientID,
           transactionID );
      return false;
   }
}

/*===========================================================================
METHOD:
   AddToNotifyList (Public Method)

DESCRIPTION:
   Add Notify entry to this client's notify List

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any
   pNotifyFunct      [ I ] - Callback function to be run when data is available
   pData             [ I ] - Data buffer that willl be passed (unmodified)
                             to callback

RETURN VALUE:
   bool
===========================================================================*/
bool AddToNotifyList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID,
   void             (* pNotifyFunct)(sGobiUSBNet *, u16, void *),
   void *           pData )
{
   sClientMemList * pClientMem;
   sNotifyList ** ppThisNotifyList;
   DBG("ClientID:0x%x, TID:0x%x\n",clientID,transactionID);
   if(pDev==NULL)
   {
      DBG("NULL");
      return eNotifyListEmpty;
   }
#ifdef CONFIG_SMP
   // Verify Lock
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n", clientID );
      return false;
   }

   // Go to last URBList entry
   ppThisNotifyList = &pClientMem->mpReadNotifyList;
   while (*ppThisNotifyList != NULL)
   {
      ppThisNotifyList = &(*ppThisNotifyList)->mpNext;
   }

   *ppThisNotifyList = kmalloc( sizeof( sNotifyList ), GFP_ATOMIC );
   if (*ppThisNotifyList == NULL)
   {
      DBG( "Mem error\n" );
      return false;
   }

   (*ppThisNotifyList)->mpNext = NULL;
   (*ppThisNotifyList)->mpNotifyFunct = pNotifyFunct;
   (*ppThisNotifyList)->mpData = pData;
   (*ppThisNotifyList)->mTransactionID = transactionID;

   return true;
}

/*===========================================================================
METHOD:
   NotifyAndPopNotifyList (Public Method)

DESCRIPTION:
   Remove first Notify entry from this client's notify list
   and Run function

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any

RETURN VALUE:
   bool
===========================================================================*/
int NotifyAndPopNotifyList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID )
{
   sClientMemList * pClientMem;
   sNotifyList * pDelNotifyList, ** ppNotifyList;

   if(pDev==NULL)
   {
      DBG("NULL");
      return eNotifyListEmpty;
   }
#ifdef CONFIG_SMP
   // Verify Lock
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif

   // Get this client's memory location
   pClientMem = FindClientMem( pDev, clientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n", clientID );
      return eNotifyListEmpty;
   }

   ppNotifyList = &(pClientMem->mpReadNotifyList);
   pDelNotifyList = NULL;

   // Remove from list
   CLIENT_READMEM_SNAPSHOT(clientID,pDev);
   while (*ppNotifyList != NULL)
   {
      // Do we care about transaction ID?
      if (transactionID == 0
      ||  (*ppNotifyList)->mTransactionID == 0
      ||  transactionID == (*ppNotifyList)->mTransactionID)
      {
         pDelNotifyList = *ppNotifyList;
         break;
      }

      DBG( "skipping data TID = %x\n", (*ppNotifyList)->mTransactionID );

      // next
      ppNotifyList = &(*ppNotifyList)->mpNext;
   }

   if (pDelNotifyList != NULL)
   {
      // Remove element
      *ppNotifyList = (*ppNotifyList)->mpNext;

      // Run notification function
      if (pDelNotifyList->mpNotifyFunct != NULL)
      {
        if (pDev->mbUnload < eStatUnloading) 
        {
          // Unlock for callback
          if(pDev)
          LocalClientMemUnLockSpinLockIRQRestore(pDev,pDev->mQMIDev.mFlag,__LINE__);

          pDelNotifyList->mpNotifyFunct( pDev,
                                        clientID,
                                        pDelNotifyList->mpData );

          // Restore lock
          LocalClientMemLockSpinLockIRQSave(pDev,__LINE__);
        }
      }

      // Delete memory
      kfree( pDelNotifyList );
      return eNotifyListFound;
   }
   else
   {
      DBG( "no one to notify for Client:0x%x, TID 0x%x\n",clientID, transactionID );
      return eNotifyListNotFound;
   }
}

/*=========================================================================*/
// Internal userspace wrappers
/*=========================================================================*/

/*===========================================================================
METHOD:
   UserspaceunlockedIOCTL (Public Method)

DESCRIPTION:
   Internal wrapper for Userspace IOCTL interface

PARAMETERS
   pFilp        [ I ] - userspace file descriptor
   cmd          [ I ] - IOCTL command
   arg          [ I ] - IOCTL argument

RETURN VALUE:
   long - 0 for success
          Negative errno for failure
===========================================================================*/
long UserspaceunlockedIOCTL(
   struct file *     pFilp,
   unsigned int      cmd,
   unsigned long     arg ) 
{
   int j;
   int result;
   u32 devVIDPID;

   sQMIFilpStorage * pFilpData = (sQMIFilpStorage *)pFilp->private_data;

   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return -EBADF;
   }
   if(cmd==USBDEVFS_RESET)
   {
      DBG( "RESET 0\n" );
      return 0;
   }
   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      return -ENXIO;
   }
   
   if(pFilpData->mpDev->mbUnload)
   {
      DBG( "Unload:%s\n", __FUNCTION__);
      return -ENXIO;
   }

   if(pFilpData->mDeviceInvalid==1)
   {
      DBG( "Clsoing.." );
      return -ENXIO;
   }
   if(pFilpData->iIsClosing==1)
   {
       DBG( "Invalid device! Updating f_ops\n" );
       return -ENXIO;

   }
   if(pFilpData->mpDev->iIsClosing==1)
   {
      DBG( "Device Clsoing.." );
      return -ENXIO;
   }

   switch (cmd)
   {
      case IOCTL_QMI_GET_SERVICE_FILE:
         DBG( "Setting up QMI for service %lu\n", arg );
         if ((u8)arg == 0)
         {
            DBG( "Cannot use QMICTL from userspace\n" );
            return -EINVAL;
         }

         // Connection is already setup
         if (pFilpData->mClientID != (u16)-1)
         {
            DBG( "Close the current connection before opening a new one\n" );
            return -EBADR;
         }

         pFilpData->iSemID = __LINE__;
         result = GetClientID( pFilpData->mpDev, (u8)arg ,&(pFilpData->mReadSem));
         pFilpData->iSemID = -__LINE__;

         if (result < 0)
         {
            pFilpData->mDeviceInvalid = 1;
            return result;
         }
         pFilpData->mClientID = (u16)result;
         DBG("pFilpData->mClientID = 0x%x\n", pFilpData->mClientID );
         return 0;
         break;


      case IOCTL_QMI_GET_DEVICE_VIDPID:
         if (arg == 0)
         {
            DBG( "Bad VIDPID buffer\n" );
            return -EINVAL;
         }

         // Extra verification
         if (pFilpData->mpDev->mpNetDev == 0)
         {
            DBG( "Bad mpNetDev\n" );
            return -ENOMEM;
         }
         if (pFilpData->mpDev->mpNetDev->udev == 0)
         {
            DBG( "Bad udev\n" );
            return -ENOMEM;
         }

         devVIDPID = ((le16_to_cpu( pFilpData->mpDev->mpNetDev->udev->descriptor.idVendor ) << 16)
                     + le16_to_cpu( pFilpData->mpDev->mpNetDev->udev->descriptor.idProduct ) );

         result = copy_to_user( (unsigned int *)arg, &devVIDPID, 4 );
         if (result != 0)
         {
            DBG( "Copy to userspace failure %d\n", result );
         }

         return result;

         break;

      case IOCTL_QMI_GET_DEVICE_MEID:
         if (arg == 0)
         {
            DBG( "Bad MEID buffer\n" );
            return -EINVAL;
         }
         result = copy_to_user( (unsigned int *)arg, &pFilpData->mpDev->mMEID[0], MAX_DEVICE_MEID_SIZE);
         if (result != 0)
         {
            DBG( "Copy to userspace failure %d\n", result );
         }

         return result;

         break;

      case IOCTL_QMI_ADD_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;
             sMapping *pmap = (sMapping*) arg;

             DBG( "add mapping\n" );
             if (arg == 0)
             {
                DBG( "null pointer\n" );
                return -EINVAL;
             }
             DBG( "dscp, qos_id: 0x%x, 0x%x\n", pmap->dscp, pmap->qosId );

             if ((MAX_DSCP_ID < pmap->dscp) && (UNIQUE_DSCP_ID != pmap->dscp))
             {
                 DBG( "Invalid DSCP value\n" );
                 return -EINVAL;
             }

             //check for existing map
             for (j=0;j<MAX_MAP;j++)
             {
                 if (pDev->maps.table[j].dscp == pmap->dscp)
                 {
                     DBG("mapping already exists at slot #%d\n", j);
                     return -EINVAL;
                 }
             }

             //check if this is a request to redirect all IP traffic to default bearer
             if (UNIQUE_DSCP_ID == pmap->dscp)
             {
                 DBG("set slot (%d) to indicate IP packet redirection is needed\n", MAX_MAP-1);
                 pDev->maps.table[MAX_MAP-1].dscp = UNIQUE_DSCP_ID;
                 pDev->maps.table[MAX_MAP-1].qosId = pmap->qosId;
                 pDev->maps.count++; 
                 return 0;
             }

             //find free slot to hold new mapping
             for(j=0;j<MAX_MAP-1;j++)
             {
                 if (pDev->maps.table[j].dscp == 0xff)
                 {
                     pDev->maps.table[j].dscp = pmap->dscp;
                     pDev->maps.table[j].qosId = pmap->qosId;
                     pDev->maps.count++; 
                     return 0;
                 }
             }

             DBG("no free mapping slot\n");
             return -ENOMEM;
         }
         break;

      case IOCTL_QMI_EDIT_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;

             sMapping *pmap = (sMapping*) arg;
             DBG( "edit mapping\n" );
             if (arg == 0)
             {
                DBG( "null pointer\n" );
                return -EINVAL;
             }
             DBG( "dscp, qos_id: 0x%x, 0x%x\n", pmap->dscp, pmap->qosId );

             if ((MAX_DSCP_ID < pmap->dscp) && (UNIQUE_DSCP_ID != pmap->dscp))
             {
                 DBG( "Invalid DSCP value\n" );
                 return -EINVAL;
             }

             for(j=0;j<MAX_MAP;j++)
             {
                 if (pDev->maps.table[j].dscp == pmap->dscp)
                 {
                     pDev->maps.table[j].qosId = pmap->qosId;
                     return 0;
                 }
             }

             DBG("no matching tos for edit mapping\n");
             return -ENOMEM;
         }
         break;

      case IOCTL_QMI_READ_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;

             sMapping *pmap = (sMapping*) arg;
             DBG( "read mapping\n" );
             if (arg == 0)
             {
                DBG( "null pointer\n" );
                return -EINVAL;
             }

             if ((MAX_DSCP_ID < pmap->dscp) && (UNIQUE_DSCP_ID != pmap->dscp))
             {
                 DBG( "Invalid DSCP value\n" );
                 return -EINVAL;
             }

             for(j=0;j<MAX_MAP;j++)
             {
                 if (pDev->maps.table[j].dscp == pmap->dscp)
                 {
                     pmap->qosId = pDev->maps.table[j].qosId;
                     DBG( "dscp, qos_id: 0x%x, 0x%x\n", pmap->dscp, pmap->qosId );

                     result = copy_to_user( (unsigned int *)arg, &pDev->maps.table[j], sizeof(sMapping));
                     if (result != 0)
                     {
                         DBG( "Copy to userspace failure %d\n", result );
                     }

                     return result;
                 }
             }

             DBG("no matching tos for read mapping\n");
             return -ENOMEM;
         }
         break;

      case IOCTL_QMI_DEL_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;
             sMapping *pmap = (sMapping*) arg;
             DBG( "Delete mapping\n" );
             if (arg == 0)
             {
                 DBG( "null pointer\n" );
                 return -EINVAL;
             }
             DBG( "DSCP 0x%x\n", pmap->dscp );

             if ((MAX_DSCP_ID < pmap->dscp) && (UNIQUE_DSCP_ID != pmap->dscp))
             {
                 DBG( "Invalid DSCP value\n" );
                 return -EINVAL;
             }

             for(j=0;j<MAX_MAP;j++)
             {
                 if (pDev->maps.table[j].dscp == pmap->dscp)
                 {
                     // delete mapping table entry
                     memset(&pDev->maps.table[j], 0xff, sizeof(pDev->maps.table[0]));
                     if (pDev->maps.count) pDev->maps.count--; 
                     return 0;
                 }
             }

             DBG("no matching mapping slot\n");
             return -ENOMEM;
         }
         break;

      case IOCTL_QMI_CLR_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;
             DBG( "Clear mapping\n" );
             memset(pDev->maps.table, 0xff, sizeof(pDev->maps.table));
             pDev->maps.count = 0; 
             return 0;
         }
         break;

#ifdef QOS_SIMULATE
      case IOCTL_QMI_QOS_SIMULATE:
         {
             int result;
             u8 supported = (u8)-1;
             DBG( "simulate indication\n" );
             u8 qos_support_ind[] = {
                 0x01,0x15,0x00,0x80,0x04,0xFF,0x04,0x00,0x00,
                 0x27,0x00,0x09,0x00,0x01,0x01,0x00,0x01,0x10,0x02,0x00,0x01,0x80
             };
             u8 qos_flow_activate_ind[] = {
                 0x01,0x15,0x00,0x80,0x04,0xFF,0x04,0x00,0x00,
                 0x26,0x00,0x09,0x00,0x01,0x06,0x00,0xDD,0xCC,0xBB,0xAA,0x01,0x01
             };
             u8 qos_flow_suspend_ind[] = {
                 0x01,0x15,0x00,0x80,0x04,0xFF,0x04,0x00,0x00,
                 0x26,0x00,0x09,0x00,0x01,0x06,0x00,0xDD,0xCC,0xBB,0xAA,0x02,0x02
             };
             u8 qos_flow_gone_ind[] = {
                 0x01,0x15,0x00,0x80,0x04,0xFF,0x04,0x00,0x00,
                 0x26,0x00,0x09,0x00,0x01,0x06,0x00,0xDD,0xCC,0xBB,0xAA,0x03,0x03
             };
             result = QMIQOSEventResp( qos_support_ind,
                     sizeof(qos_support_ind));
             result = QMIQOSEventResp( qos_flow_activate_ind,
                     sizeof(qos_flow_activate_ind));
             result = QMIQOSEventResp( qos_flow_suspend_ind,
                     sizeof(qos_flow_suspend_ind));
             result = QMIQOSEventResp( qos_flow_gone_ind,
                     sizeof(qos_flow_gone_ind));
             return 0;
         }
         break;
#endif

      case IOCTL_QMI_GET_TX_Q_LEN:
         {

             sGobiUSBNet * pDev = pFilpData->mpDev;

             if (arg == 0)
             {
                 DBG( "Bad Tx Queue buffer\n" );
                 return -EINVAL;
             }

             // Extra verification
             if (pFilpData->mpDev->mpNetDev == 0)
             {
                 DBG( "Bad mpNetDev\n" );
                 return -ENOMEM;
             }
             if (pFilpData->mpDev->mpNetDev->udev == 0)
             {
                 DBG( "Bad udev\n" );
                 return -ENOMEM;
             }

             result = copy_to_user( (unsigned int *)arg, &pDev->tx_qlen, sizeof(pDev->tx_qlen) );
             if (result != 0)
             {
                 DBG( "Copy to userspace failure %d\n", result );
             }

             return result;
         }

         break;

      case IOCTL_QMI_DUMP_MAPPING:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;

             DBG( "dump mapping\n" );
             if (arg == 0)
             {
                DBG( "null pointer\n" );
                return -EINVAL;
             }

             result = copy_to_user( (unsigned int *)arg, &pDev->maps.table[0], sizeof(pDev->maps.table));
             if (result != 0)
             {
                 DBG( "Copy to userspace failure %d\n", result );
             }
             return result;
         }

      case IOCTL_QMI_GET_USBNET_STATS:
         {
             sGobiUSBNet * pDev = pFilpData->mpDev;
             struct net_device_stats * pStats = &(pDev->mpNetDev->net->stats);
             sNetStats netStats;

             if (arg == 0)
             {
                 DBG( "Bad usbnet statistic buffer\n" );
                 return -EINVAL;
             }

             // Extra verification
             if (pFilpData->mpDev->mpNetDev == 0)
             {
                 DBG( "Bad mpNetDev\n" );
                 return -ENOMEM;
             }

             /* copy the value from struct net_device_stats to struct sNetStats */
             netStats.rx_packets = pStats->rx_packets;
             netStats.tx_packets = pStats->tx_packets;
             netStats.rx_bytes = pStats->rx_bytes;
             netStats.tx_bytes = pStats->tx_bytes;
             netStats.rx_errors = pStats->rx_errors;
             netStats.tx_errors = pStats->tx_errors;
             netStats.rx_overflows = pStats->rx_fifo_errors;
             netStats.tx_overflows = pStats->tx_fifo_errors;

             result = copy_to_user( (unsigned int *)arg, &netStats, sizeof(sNetStats) );
             if (result != 0)
             {
                 DBG( "Copy to userspace failure %d\n", result );
             }

             return result;
         }

         break;
         case IOCTL_QMI_SET_DEVICE_MTU:
         {
             sGobiUSBNet *pDev = pFilpData->mpDev;
             // struct usbnet * pNet = netdev_priv( pDev->mpNetDev->net );
             int iArgp = (int)arg;
             if (iArgp <= 0)
             {
                 DBG( "Bad MTU buffer\n" );
                 return -EINVAL;
             }
             DBG( "new mtu :%d ,qcqmi:%d\n",iArgp,(int)pDev->mQMIDev.qcqmi );
             pDev->mtu = iArgp;
             usbnet_change_mtu(pDev->mpNetDev->net ,pDev->mtu);
         }
         return 0;
      default:
         return -EBADRQC;
   }
}

/*=========================================================================*/
// Userspace wrappers
/*=========================================================================*/

/*===========================================================================
METHOD:
   UserspaceOpen (Public Method)

DESCRIPTION:
   Userspace open
      IOCTL must be called before reads or writes

PARAMETERS
   pInode       [ I ] - kernel file descriptor
   pFilp        [ I ] - userspace file descriptor

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int UserspaceOpen(
   struct inode *         pInode,
   struct file *          pFilp )
{
   sQMIFilpStorage * pFilpData;
   DBG( "%s\n",__FUNCTION__ );
   // Optain device pointer from pInode
    sQMIDev * pQMIDev = container_of( pInode->i_cdev,
                                     sQMIDev,
                                     mCdev );
   sGobiUSBNet * pDev = container_of( pQMIDev,
                                    sGobiUSBNet,
                                    mQMIDev );
   if(signal_pending(current))
   {
      return -ERESTARTSYS;
   }
   pFilp->private_data = NULL;

   if (IsDeviceValid( pDev ) == false) 
   {
      printk( KERN_INFO "Invalid device\n" );
      return -ENXIO;
   }
   if(pDev->mbUnload)
   {
       printk( KERN_INFO "Unload:%s\n", __FUNCTION__);
      return -ENXIO;
   }
   if(pDev->iIsClosing)
   {
     printk( KERN_INFO "Unload:%s\n", __FUNCTION__);
      return -ENXIO;
   }
   // Setup data in pFilp->private_data
   pFilp->private_data = kmalloc( sizeof( sQMIFilpStorage ), GFP_KERNEL );
   if (pFilp->private_data == NULL)
   {
      printk( KERN_INFO "Mem error\n" );
      return -ENOMEM;
   }

   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   pFilpData->mClientID = (u16)-1;
   pFilpData->mDeviceInvalid = 0;
   pFilpData->mpDev = pDev;
   pFilpData->iSemID = -1;
   pFilpData->iIsClosing = 0;
   pFilpData->iReadSyncResult = -1;
   sema_init(&pFilpData->mReadSem , SEMI_INIT_DEFAULT_VALUE );
   return 0;
}

/*===========================================================================
METHOD:
   UserspaceIOCTL (Public Method)

DESCRIPTION:
   Userspace IOCTL functions

PARAMETERS
   pUnusedInode [ I ] - (unused) kernel file descriptor
   pFilp        [ I ] - userspace file descriptor
   cmd          [ I ] - IOCTL command
   arg          [ I ] - IOCTL argument

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int UserspaceIOCTL(
   struct inode *    pUnusedInode,
   struct file *     pFilp,
   unsigned int      cmd,
   unsigned long     arg ) 
{
   if(signal_pending(current))
   {
      return -ERESTARTSYS;
   }
   // call the internal wrapper function
   return (int)UserspaceunlockedIOCTL( pFilp, cmd, arg );  
}

/*===========================================================================
METHOD:
   UserspaceClose (Public Method)

DESCRIPTION:
   Userspace close
      Release client ID and free memory

PARAMETERS
   pFilp           [ I ] - userspace file descriptor
   unusedFileTable [ I ] - (unused) file table

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int UserspaceClose(
   struct file *       pFilp,
   fl_owner_t          unusedFileTable )
{
   sQMIFilpStorage * pFilpData = NULL;
   DBG( "\n" );

   if(pFilp ==NULL)
   {
      printk( KERN_INFO "bad file data\n" );
      return -EBADF;
   }
   
   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   if (pFilpData == NULL)
   {
      printk( KERN_INFO "bad file data\n" );
      return 0;
   }
   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      printk( KERN_INFO "%s Invalid device! Updating f_ops\n",__FUNCTION__ );
   }

   pFilpData->iIsClosing = 1;
   pFilpData->mDeviceInvalid = 1;
   GobiSyncRcu();
   if(pFilpData->iSemID > 0)
   {
      int iRetry = 0;
      int iReturn = 0;
      int iLockCount = 0;
      if(pFilpData->mpDev->mbUnload)
      {
         iReturn = -EAGAIN;
      }
      
      do 
      {
          GobiSyncRcu();
          if(LocalClientMemLockSpinIsLock(pFilpData->mpDev)!=0)
          {
             if(iLockCount++ > 5)
             {
                unsigned long flags = pFilpData->mpDev->mQMIDev.mFlag;
                printk("Force Unlock!");
                LocalClientMemUnLockSpinLockIRQRestore(pFilpData->mpDev,flags,__LINE__);
             }
             else
             {
                gobi_flush_work();
                continue;
             }
          }
          iLockCount = 0;
          if((pFilpData==NULL) || (pFilp==NULL))
          {
            iReturn = 0;
            break;
          }
          #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,33 ))
          if(do_raw_spin_trylock(&(pFilpData->mReadSem.lock))==0)
          {
             DBG("NOT locked");
             do_raw_spin_unlock(&(pFilpData->mReadSem.lock));
             if(!down_trylock(&(pFilpData->mReadSem)))
             {
                 DBG("NOT locked");
             }
             up(&(pFilpData->mReadSem));
          }
          #else
          if(!down_trylock(&(pFilpData->mReadSem)))
          {
              DBG("NOT locked");
          }
          up(&(pFilpData->mReadSem));
          #endif

          weakup_inode_process(pFilp,NULL);
          gobi_flush_work();
          if((pFilpData==NULL) || (pFilp==NULL))
          {
             iReturn = 0;
             break;
          }
          if(iRetry++>10)
          {
              iReturn = -EAGAIN;
              printk("Timeout!");
              break;
          }
      }while(pFilpData->iSemID > 0);
      GobiSyncRcu();
   } 
  
   if(pFilpData->mpDev->mbUnload)
   {
      return 0;
   }

   if (pFilpData->mpDev->mbUnload > eStatUnloading)
   {
      kfree( pFilp->private_data );
      pFilp->private_data = NULL;
      return -ENXIO;
   }
   
   DBG( "0x%04X\n", pFilpData->mClientID );

   if (pFilpData->mClientID != (u16)-1) 
   {
     pFilpData->iSemID = __LINE__;
     if(pFilpData->iReadSyncResult>=0)
     {
          ReleaseClientID( pFilpData->mpDev,
                      pFilpData->mClientID,
                      &(pFilpData->mReadSem));
     }
     else
     {
         unsigned long flags;
         flags = LocalClientMemLockSpinLockIRQSave( pFilpData->mpDev , __LINE__);
         RemoveAndPopNotifyList(pFilpData->mpDev,
                      pFilpData->mClientID,0);
         LocalClientMemUnLockSpinLockIRQRestore ( pFilpData->mpDev ,flags,__LINE__);
         
     }
     pFilpData->iSemID = -__LINE__;
     pFilpData->mClientID = (u16)-1;
   }

   kfree( pFilp->private_data );

   // Disable pFilpData so they can't keep sending read or write
   //    should this function hang
   // Note: memory pointer is still saved in pFilpData to be deleted later
   pFilp->private_data = NULL;
   GobiSyncRcu();
   return 0;
}

/*===========================================================================
METHOD:
   UserspaceRead (Public Method)

DESCRIPTION:
   Userspace read (synchronous)

PARAMETERS
   pFilp           [ I ] - userspace file descriptor
   pBuf            [ I ] - read buffer
   size            [ I ] - size of read buffer
   pUnusedFpos     [ I ] - (unused) file position

RETURN VALUE:
   ssize_t - Number of bytes read for success
             Negative errno for failure
===========================================================================*/
ssize_t UserspaceRead(
   struct file *          pFilp,
   char __user *          pBuf,
   size_t                 size,
   loff_t *               pUnusedFpos )
{
   int result = -1;
   void * pReadData = NULL;
   void * pSmallReadData = NULL;
   sQMIFilpStorage * pFilpData = NULL;
   DBG("\n"); 
   if(pFilp==NULL)
   {
       return -EBADF;
   }
   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return -EBADF;
   }

   if(signal_pending(current))
   {
      return -ERESTARTSYS;
   }

   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      return -ENXIO;
   }
   if(pFilpData->mpDev->mbUnload)
   {
      DBG( "Unload:%s\n", __FUNCTION__);
      return -ENXIO;
   }

   if(pFilpData->mDeviceInvalid)
   {
      DBG( "mDeviceInvalid\n");
      return -ENXIO;
   }
   
   if (pFilpData->mClientID == (u16)-1)
   {
      DBG( "Client ID must be set before reading 0x%04X\n",
           pFilpData->mClientID );
      return -EBADR;
   }
   if(pFilpData->iIsClosing==1)
   {
      DBG( "filep Clsoing.." );
      return -ENXIO;
   }
   
   if(pFilpData->mpDev->iIsClosing==1)
   {
      DBG( "Device Clsoing.." );
      return -ENXIO;
   }

   pFilpData->iSemID = __LINE__;
   // Perform synchronous read
   result = ReadSync( pFilpData->mpDev,
                      &pReadData,
                      pFilpData->mClientID,
                      0,
                      &(pFilpData->iSemID),&(pFilpData->mReadSem),&(pFilpData->iIsClosing));
   if(pFilp==NULL)
   {
      DBG("%s pFilp NULL\n",__FUNCTION__);
      return -ENXIO;
   }
   if(pFilpData==NULL)
   {
      return -ENXIO;
   }
   pFilpData->iSemID = -__LINE__;
   pFilpData->iReadSyncResult = result;
   GobiSyncRcu();
   if(result<0)
   {
      DBG("Read Error!\n");
   }
   else
   {
      PrintHex((char*)&pReadData,result);
   }
   if(pFilp==NULL)
   {
      DBG("%s pFilp NULL\n",__FUNCTION__);
      return -ENXIO;
   }
   if(pFilpData->mpDev==NULL)
   {
      DBG("%s pFilp NULL\n",__FUNCTION__);
      return -ENXIO;
   }
   if((pFilpData->mpDev->mbUnload)||(pFilpData->iIsClosing))
   {
      return -ENXIO;
   }
   if (result <= 0)
   {
      #ifdef CONFIG_PM
      if(bIsSuspend(pFilpData->mpDev))
      {
         DBG("SUSPEND\n");
      }
      #endif
      if(result == -EINTR)
      {
         DBG("RETRY\n");
      }
      return result;
   }

   // Discard QMUX header
   result -= QMUXHeaderSize();
   pSmallReadData = pReadData + QMUXHeaderSize();

   if (result > size)
   {
      DBG( "Read data is too large for amount user has requested\n" );
      if(pReadData)
      kfree( pReadData );
      return -EOVERFLOW;
   }

   DBG(  "pBuf = 0x%p pSmallReadData = 0x%p, result = %d",
         pBuf, pSmallReadData, result );

   if (copy_to_user( pBuf, pSmallReadData, result ) != 0)
   {
      DBG( "Error copying read data to user\n" );
      result = -EFAULT;
   }
   pSmallReadData = NULL;
   // Reader is responsible for freeing read buffer
   kfree( pReadData );

   return result;
}

/*===========================================================================
METHOD:
   UserspaceWrite (Public Method)

DESCRIPTION:
   Userspace write (synchronous)

PARAMETERS
   pFilp           [ I ] - userspace file descriptor
   pBuf            [ I ] - write buffer
   size            [ I ] - size of write buffer
   pUnusedFpos     [ I ] - (unused) file position

RETURN VALUE:
   ssize_t - Number of bytes read for success
             Negative errno for failure
===========================================================================*/
ssize_t UserspaceWrite(
   struct file *        pFilp,
   const char __user *  pBuf,
   size_t               size,
   loff_t *             pUnusedFpos )
{
   int status;
   void * pWriteBuffer;
   sQMIFilpStorage * pFilpData = (sQMIFilpStorage *)pFilp->private_data;

   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return -EBADF;
   }
   if(signal_pending(current))
   {
      return -ERESTARTSYS;
   }
   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      return -ENXIO;
   }
   if(pFilpData->mpDev->mbUnload)
   {
      DBG( "Unload:%s\n", __FUNCTION__);
      return -ENXIO;
   }

   if(pFilpData->mDeviceInvalid)
   {
      DBG( "mDeviceInvalid\n");
      return -ENXIO;
   }

   if (pFilpData->mClientID == (u16)-1)
   {
      DBG( "Client ID must be set before writing 0x%04X\n",
           pFilpData->mClientID );
      return -EBADR;
   }
   if(pFilpData->iIsClosing==1)
   {
      DBG( "Filep Clsoing.." );
      return -ENXIO;
   }
   if(pFilpData->mpDev->iIsClosing==1)
   {
      DBG( "Device Clsoing.." );
      return -ENXIO;
   }

   // Copy data from user to kernel space
   pWriteBuffer = kmalloc( size + QMUXHeaderSize(), GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }
   status = copy_from_user( pWriteBuffer + QMUXHeaderSize(), pBuf, size );
   if (status != 0)
   {
      DBG( "Unable to copy data from userspace %d\n", status );
      kfree( pWriteBuffer );
      return status;
   }

   status = WriteSync( pFilpData->mpDev,
                       pWriteBuffer,
                       size + QMUXHeaderSize(),
                       pFilpData->mClientID );

   kfree( pWriteBuffer );

   // On success, return requested size, not full QMI reqest size
   if (status == size + QMUXHeaderSize())
   {
      return size;
   }
   else
   {
      pFilpData->mDeviceInvalid = 1;
      if(status<0)
      {
         pFilpData->iIsClosing=1;
         if (pFilp->f_op->flush)
         {
              ForceFilpClose(pFilp);
              GobiSyncRcu();
         }
         return -ENXIO;
      }
      return status;
   }
}

/*===========================================================================
METHOD:
   UserspacePoll (Public Method)

DESCRIPTION:
   Used to determine if read/write operations are possible without blocking

PARAMETERS
   pFilp              [ I ] - userspace file descriptor
   pPollTable         [I/O] - Wait object to notify the kernel when data 
                              is ready

RETURN VALUE:
   unsigned int - bitmask of what operations can be done immediately
===========================================================================*/
unsigned int UserspacePoll(
   struct file *                  pFilp,
   struct poll_table_struct *     pPollTable )
{
   sQMIFilpStorage * pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   sClientMemList * pClientMem;
   unsigned long flags;

   // Always ready to write
   unsigned int status = POLLOUT | POLLWRNORM;

   if (pFilpData == NULL)
   {
      DBG( "Bad file data\n" );
      return POLLERR;
   }

   if (IsDeviceValid( pFilpData->mpDev ) == false)
   {
      DBG( "Invalid device! Updating f_ops\n" );
      return POLLERR;
   }

   if (pFilpData->mClientID == (u16)-1)
   {
      DBG( "Client ID must be set before polling 0x%04X\n",
           pFilpData->mClientID );
      return POLLERR;
   }

   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pFilpData->mpDev , __LINE__);

   // Get this client's memory location
   pClientMem = FindClientMem( pFilpData->mpDev, 
                              pFilpData->mClientID );
   if (pClientMem == NULL)
   {
      DBG( "Could not find this client's memory 0x%04X\n",
           pFilpData->mClientID );

      LocalClientMemUnLockSpinLockIRQRestore ( pFilpData->mpDev ,flags,__LINE__);
      return POLLERR;
   }
   
   poll_wait( pFilp, &pClientMem->mWaitQueue, pPollTable );

   if (pClientMem->mpList != NULL)
   {
      status |= POLLIN | POLLRDNORM;
   }

   // End critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pFilpData->mpDev ,flags,__LINE__);

   // Always ready to write 
   return (status | POLLOUT | POLLWRNORM);
}


int weakup_inode_process(struct file *pFilp,struct task_struct * pTask)
{
   struct task_struct *pEachTask=NULL;
   if(pFilp==NULL)
      return 0;

   if(pTask)
   {
      if(pTask->state != TASK_STOPPED)
      {
         wake_up_process(pTask);
         return 0;
      }
      else
      {
         return 0;
      }
   }
   for_each_process( pEachTask )
   {
      int count = 0;
      struct fdtable * pFDT;
      if (pEachTask == NULL || pEachTask->files == NULL)
      {
         // Some tasks may not have files (e.g. Xsession)
            continue;
      }
      pFDT = files_fdtable( pEachTask->files );
      for (count = 0; count < pFDT->max_fds; count++)
      {
         if (pFDT->fd[count] == pFilp)
         {
            if(pEachTask->state != TASK_STOPPED)
            {
               wake_up_process(pEachTask);
            }
            count = pFDT->max_fds;
         }
      }
   }
   return 0;
}

/*===========================================================================
METHOD:
   UserSpaceLock (Public Method)

DESCRIPTION:
   Used to determine if read/write operations are possible without blocking

PARAMETERS
   pFilp      [ I ] - The file to apply the lock to
   cmd        [ I ] - type of locking operation (F_SETLK, F_GETLK, etc.)
   fl         [I/O] - The lock to be applied
   conf       [I/O] - Place to return a copy of the conflicting lock, if found.

RETURN VALUE:
   unsigned int - bitmask of what operations can be done immediately
===========================================================================*/
int UserSpaceLock(struct file *filp, unsigned int cmd, struct file_lock *fl, struct file_lock *conf)
{
   if((filp!=NULL) && (fl!=NULL) && (conf!=NULL))
   {
      #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,9,0 ))
      if(file_inode(filp)!=NULL)
      {
         return posix_lock_file(filp, fl, conf);
      }
      #else
      return posix_lock_file(filp, fl, conf);
      #endif
   }
   return -ENOLCK;
}

/*===========================================================================
METHOD:
   UserspaceRelease (Public Method)

DESCRIPTION:
   Used to determine if read/write operations are possible without blocking

PARAMETERS
   pInode              [I/O] - userspace file descriptor
   pFilp               [I/O] - userspace file descriptor
   
RETURN VALUE:
   unsigned int - bitmask of what operations can be done immediately
===========================================================================*/
int UserspaceRelease(struct inode *pInode, struct file *pFilp)
{
   sQMIFilpStorage * pFilpData = NULL;

   if(pFilp==NULL)
   {
      return 0;
   }

   pFilpData = (sQMIFilpStorage *)pFilp->private_data;
   if(pFilpData!=NULL)
   {
      pFilpData->iIsClosing = 1;
      pFilpData->mDeviceInvalid = 1;
      if(pFilpData->iSemID > 0)
      {
         int iRetry = 0;
         int iLockCount =0;
         do 
         {
            GobiSyncRcu();
            if(LocalClientMemLockSpinIsLock(pFilpData->mpDev)!=0)
            {
                if(iLockCount++ > 5)
                {
                   unsigned long flags = pFilpData->mpDev->mQMIDev.mFlag;
                   printk("Force Unlock!\n");
                   LocalClientMemUnLockSpinLockIRQRestore(pFilpData->mpDev,flags,__LINE__);
                }
                else
                {
                   gobi_flush_work();
                   continue;
                }
            }
            iLockCount = 0;
            #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,33 ))
            if(do_raw_spin_trylock(&(pFilpData->mReadSem.lock))==0)
            {
               DBG("NOT locked");
               do_raw_spin_unlock(&(pFilpData->mReadSem.lock));
               if(!down_trylock(&(pFilpData->mReadSem)))
               {
                   DBG("NOT locked");
               }
               up(&(pFilpData->mReadSem));
            }
            #else
            if(!down_trylock(&(pFilpData->mReadSem)))
            {
                DBG("NOT locked");
            }
            up(&(pFilpData->mReadSem));
            #endif
            weakup_inode_process(pFilp,NULL);
            gobi_flush_work();
            GobiSyncRcu();
            if((pFilpData==NULL) || (pFilp==NULL))
            {
               break;
            }
            if(iRetry++>5)
            {
            break;
            }
         }while(pFilpData->iSemID > 0);
         return 0;
      }
      if(pFilp)
        if(pFilp->private_data)
        {
          kfree(pFilp->private_data);
          pFilp->private_data = NULL;
        }
   }
   GobiSyncRcu();
   return 0;
}
/*=========================================================================*/
// Initializer and destructor
/*=========================================================================*/
int QMICTLSyncProc(sGobiUSBNet *pDev)
{
   void *pWriteBuffer;
   void *pReadBuffer;
   int result;
   u16 writeBufferSize;
   u8 transactionID;
   int iID = -1;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return -EFAULT;
   }

   writeBufferSize= QMICTLSyncReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   transactionID = QMIXactionIDGet(pDev);

   /* send a QMI_CTL_SYNC_REQ (0x0027) */
   result = QMICTLSyncReq( pWriteBuffer,
                           writeBufferSize,
                           transactionID );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   iID = iGetSemID(pDev,__LINE__);
   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       QMICTL );

   kfree( pWriteBuffer );
   if (result < 0)
   {
      pDev->iReasSyncTaskID[iID]=-__LINE__;
      return result;
   }

   // QMI CTL Sync Response
   result = ReadSync( pDev,
                      &pReadBuffer,
                      QMICTL,
                      transactionID,
                      &iID,NULL,NULL);
   if (result < 0)
   {
      return result;
   }

   result = QMICTLSyncResp( pReadBuffer,
                            (u16)result );
   if(pReadBuffer);
   kfree( pReadBuffer );

   if (result < 0) /* need to re-sync */
   {
      DBG( "sync response error code %d\n", result );
      /* start timer and wait for the response */
      /* process response */
      return result;
   }

   // Success
   return 0;
}

static int 
qmi_show(struct seq_file *m, void *v)
{
    sGobiUSBNet * pDev = (sGobiUSBNet*) m->private;
    seq_printf(m, "readTimeoutCnt %d\n", pDev->readTimeoutCnt);
    seq_printf(m, "writeTimeoutCnt %d\n", pDev->writeTimeoutCnt);
    return 0;
}

static int
qmi_open(struct inode *inode, struct file *file)
{
    char *data;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,10,0 ))
    data=PDE_DATA(inode);
#else
    data=PDE(inode)->data;
#endif

    return single_open(file, qmi_show, data);
}

static const struct file_operations proc_fops = {
    .owner      = THIS_MODULE,
    .open       = qmi_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

/*===========================================================================
METHOD:
   RegisterQMIDevice (Public Method)

DESCRIPTION:
   QMI Device initialization function

PARAMETERS:
   pDev     [ I ] - Device specific memory
   is9x15   [ I ]

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int RegisterQMIDevice( sGobiUSBNet * pDev, int is9x15 )
{
   char qcqmi_dev_name[10];
   int i;
   int result;
   dev_t devno;
   pDev->mQMIDev.proc_file = NULL;
   if (pDev->mQMIDev.mbCdevIsInitialized == true)
   {
      // Should never happen, but always better to check
      DBG( "device already exists\n" );
      return -EEXIST;
   }

   pDev->mbQMIValid = true;
   pDev->mbUnload = eStatRegister;
   pDev->readTimeoutCnt = 0;
   pDev->writeTimeoutCnt = 0;
   pDev->mtu = 0;
   pDev->iShutdown_write_sem = -1;
   pDev->iShutdown_read_sem = -1;
   init_rwsem(&pDev->shutdown_rwsem);
   InitSemID(pDev);

   i=0;
   do
   {
      // Set up for QMICTL
      //    (does not send QMI message, just sets up memory)
      if(kthread_should_stop())
      {
         return -1;
      }
      if(signal_pending(current))
      {
         return -1;
      }
      
      result = GetClientID( pDev, QMICTL ,NULL);

      if(kthread_should_stop())
      {
         return -1;
      }
      if(signal_pending(current))
      {
         return -1;
      }
      if(pDev->mbUnload != eStatRegister)
      {
         return result;
      }
      if (result != 0)
      {
         if(i++>MAX_RETRY)
         {
            pDev->mbQMIValid = false;
            return result;
         }
      }
   }while(result!=0);
   atomic_set( &pDev->mQMIDev.mQMICTLTransactionID, 1 );

   // Start Async reading
   result = StartRead( pDev );
   if (result != 0)
   {
      pDev->mbQMIValid = false;
      return result;
   }

   // Send SetControlLineState request (USB_CDC)
   //   Required for Autoconnect and 9x30 to wake up
   result = usb_control_msg( pDev->mpNetDev->udev,
                             usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
                             SET_CONTROL_LINE_STATE_REQUEST,
                             SET_CONTROL_LINE_STATE_REQUEST_TYPE,
                             CONTROL_DTR,
                             /* USB interface number to receive control message */
                             pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
                             NULL,
                             0,
                             100 );
   if (result < 0)
   {
      DBG( "Bad SetControlLineState status %d\n", result );
      return result;
   }


   // Device is not ready for QMI connections right away
   //   Wait up to 30 seconds before failing
   if (QMIReady( pDev, 30000 ) == false)
   {
      DBG( "Device unresponsive to QMI\n" );
      return -ETIMEDOUT;
   }
   if(pDev->mbUnload != eStatRegister)
   {
      return -ETIMEDOUT;
   }
   // Initiate QMI CTL Sync Procedure
   DBG( "Sending QMI CTL Sync Request\n" );
   i=0;
   do
   {
      result = QMICTLSyncProc(pDev);
      if(isModuleUnload(pDev))
      {
         return -EFAULT;;
      }
      if (result != 0)
      {
         if(i++>MAX_RETRY)
         {
            DBG( "QMI CTL Sync Procedure Error\n" );
            return result;
         }
      }
      else
      {
         DBG( "QMI CTL Sync Procedure Successful\n" );
      }
   }while(result!=0);
   // Setup Data Format
   if (is9x15)
   {
      i=0;
      do
      {
         #ifdef TE_FLOW_CONTROL
         result = QMIWDASetDataFormat (pDev, true);
         #else
         result = QMIWDASetDataFormat (pDev, false);
         #endif
         if(isModuleUnload(pDev))
         {
            return -EFAULT;;
         }
         if(i++>MAX_RETRY)
            break;
      }while(result!=0);
       if(result != 0)
       {
          #ifdef TE_FLOW_CONTROL
          result = QMIWDASetDataFormat (pDev, false);
          if(result != 0)
          {
             printk(KERN_INFO "Set Data Format Fail\n");
          }
          else
          {
              printk(KERN_INFO "TE Flow Control disabled\n");
          }
          #else
          printk(KERN_INFO "Set Data Format Fail\n");
          #endif
       }
       else
       {
          #ifdef TE_FLOW_CONTROL
          printk(KERN_INFO "TE Flow Control Enabled\n");
          #else
          printk(KERN_INFO "TE Flow Control disabled\n");
          #endif
       }
   }
   else
   {
       result = QMICTLSetDataFormat (pDev);
   }

   if (result != 0)
   {
       return result;
   }

   i=0;
   do
   {
      // Setup WDS callback
      result = SetupQMIWDSCallback( pDev );
      if(isModuleUnload(pDev))
      {
         return -EFAULT;
      }
      if (result != 0)
      {
         if(i++>MAX_RETRY)
         {
         return result;
         }
      }
   }while(result!=0);

   if (is9x15)
   {
       // Set FCC Authentication
       i=0;
       do
       {
          result = QMIDMSSWISetFCCAuth( pDev );
          if(pDev->mbUnload != eStatRegister)
          {
             return -EFAULT;;
          }
          if (result != 0)
          {
            if(i++>MAX_RETRY)
            {
               return result;
            }
          }
       }while(result!=0);
   }
   // Fill MEID for device
   i=0;
   do
   {
      result = QMIDMSGetMEID( pDev );
      if(pDev->mbUnload != eStatRegister)
      {
          return -EFAULT;;
      }
      if (result != 0)
      {
         if(i++>MAX_RETRY)
         {
            return result;
         }
      }
   }while(result!=0);
   // allocate and fill devno with numbers
   result = alloc_chrdev_region( &devno, 0, 1, "qcqmi" );
   if (result < 0)
   {
      return result;
   }
   for(i=0;i<MAX_QCQMI;i++)
   {
       if (qcqmi_table[i] == 0)
           break;
   }
   
   if (i == MAX_QCQMI)
   {
       printk(KERN_WARNING "no free entry available at qcqmi_table array\n");
       return -ENOMEM;
   }
   qcqmi_table[i] = 1;
   pDev->mQMIDev.qcqmi = i;

   // Always print this output
   printk( KERN_INFO "creating qcqmi%d\n",
           pDev->mQMIDev.qcqmi );

   // Create cdev
   cdev_init( &pDev->mQMIDev.mCdev, &UserspaceQMIFops );
   pDev->mQMIDev.mCdev.owner = THIS_MODULE;
   pDev->mQMIDev.mCdev.ops = &UserspaceQMIFops;
   pDev->mQMIDev.mbCdevIsInitialized = true;

   result = cdev_add( &pDev->mQMIDev.mCdev, devno, 1 );
   if (result != 0)
   {
      DBG( "error adding cdev\n" );
      return result;
   }

#if (LINUX_VERSION_CODE >= KERNEL_VERSION( 2,6,27 ))
   // kernel 2.6.27 added a new fourth parameter to device_create
   //    void * drvdata : the data to be added to the device for callbacks
   pDev->dev = device_create( pDev->mQMIDev.mpDevClass,
                  &pDev->mpIntf->dev,
                  devno,
                  NULL,
                  "qcqmi%d",
                  pDev->mQMIDev.qcqmi );
#else
   pDev->dev = device_create( pDev->mQMIDev.mpDevClass,
                  &pDev->mpIntf->dev,
                  devno,
                  "qcqmi%d",
                  pDev->mQMIDev.qcqmi );
#endif

   pDev->mQMIDev.mDevNum = devno;

   memset(pDev->maps.table, 0xff, sizeof(pDev->maps.table));
   pDev->maps.count = 0; 

   sprintf(qcqmi_dev_name, "qcqmi%d", (int)pDev->mQMIDev.qcqmi);
   pDev->mQMIDev.proc_file = proc_create_data(qcqmi_dev_name, 0, NULL, &proc_fops, pDev);

   if (!pDev->mQMIDev.proc_file) {
       return -ENOMEM;
   }

  // Success
   return 0;
}

/*===========================================================================
METHOD:
   CloseFileInode (Public Method)

DESCRIPTION:
   Close File Inode

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
void CloseFileInode(sGobiUSBNet * pDev)
{
   struct inode * pOpenInode=NULL;
   struct task_struct * pEachTask = NULL;
   struct fdtable * pFDT;
   struct file * pFilp;
   int count = 0;
   DBG("\n");
   if(pDev==NULL)
   {
      return ;
   }
   GobiSyncRcu();
   if(!list_empty_careful(&pDev->mQMIDev.mCdev.list))
   if(!list_empty(&pDev->mQMIDev.mCdev.list))
    {
       list_for_each_entry(pOpenInode,&pDev->mQMIDev.mCdev.list,i_devices)
       {
          // Get the inode
          if (pOpenInode != NULL && (IS_ERR( pOpenInode ) == false))
          {
             // Look for this inode in each task
             for_each_process( pEachTask )
             {
                int max_fds = 0;
                if (pEachTask == NULL || pEachTask->files == NULL)
                {
                   // Some tasks may not have files (e.g. Xsession)
                   continue;
                }
                // For each file this task has open, check if it's referencing
                // our inode.
                pFDT = files_fdtable( pEachTask->files );
                max_fds = pFDT->max_fds;
                if(pFDT)
                {
                    for (count = 0; count < max_fds; count++)
                    {
                       if(pFDT==NULL)
                       {
                          break;
                       }
                       pFilp = pFDT->fd[count];
                       
                       #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,19,0 ))
                       if (pFilp != NULL &&  pFilp->f_path.dentry != NULL)
                       #else
                       if (pFilp != NULL &&  pFilp->f_dentry != NULL)
                       #endif
                       {
                          #if (LINUX_VERSION_CODE >= KERNEL_VERSION( 3,19,0 ))
                          if (pFilp->f_path.dentry->d_inode == pOpenInode)
                          #else
                          if (pFilp->f_dentry->d_inode == pOpenInode)
                          #endif
                          {
                             //int ret = 0;
                             //int retry = 0;
                             int iFilpOpen =0;
                             int reffrom = 0;
                             int reffrom2 = 0;
                             sQMIFilpStorage * pFilpData = NULL;
                             printk( KERN_INFO "forcing close of opened file handle\n" );
                             GobiSyncRcu();
                             reffrom = atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount );
                             pFilpData = (sQMIFilpStorage *)pFilp->private_data;
                             if(pFilpData!=NULL)
                             {
                               if((pFilpData->iSemID > 0)&&(pFilpData->iIsClosing==0))
                               {
                                  int iRetry = 0;
                                  iFilpOpen = 1;
                                  do
                                  {
                                      GobiSyncRcu();
                                      if(!down_trylock(&(pFilpData->mReadSem)))
                                      {
                                        DBG("NOT locked");
                                      }
                                      up(&(pFilpData->mReadSem));
                                      weakup_inode_process(pFilp,pEachTask);
                                      gobi_flush_work();
                                      if(pFilp==NULL)
                                      {
                                         break;
                                      }
                                      if(pFilp->private_data==NULL)
                                      {
                                         break;
                                      }
                                      if(reffrom>atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount))
                                      {
                                          break;
                                      }else if (atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount)<2)
                                      {
                                         break;
                                      }
                                      if(iRetry++>5)
                                      {
                                        int ret = 0;
                                        printk( KERN_INFO "Retry Max\n");
                                        if (file_count(pFilp)>0)
                                        {
                                            ret =  ForceFilpClose( pFilp );
                                            if(isPreempt()==0)
                                            rcu_assign_pointer( pFDT->fd[count], NULL );
                                            gobi_flush_work();
                                         }
                                         else
                                        {
                                            if(isPreempt()==0)
                                            rcu_assign_pointer( pFDT->fd[count], NULL );
                                            GobiSyncRcu();
                                        }
                                         break;
                                      }   
                                  }while(pFilpData->iSemID > 0);
                               }
                               else
                               {
                                      GobiSyncRcu();
                                      weakup_inode_process(pFilp,pEachTask);
                                      gobi_flush_work();
                               }
                             }
                             GobiSyncRcu();
                             reffrom2 = atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount );
                             if((reffrom2>=reffrom)&&(iFilpOpen==0))
                             if((pFilp!=NULL)&&(reffrom2>1))
                             {
                                 int ret = 0;
                                 if (file_count(pFilp)>0)
                                 {
                                     if(pFilpData)
                                     {
                                         if(pFilpData->iIsClosing==0)
                                         {
                                            ret =  ForceFilpClose( pFilp);
                                            if(isPreempt()==0)
                                            rcu_assign_pointer( pFDT->fd[count], NULL );
                                         }
                                     }
                                     else
                                     {
                                        ret =  ForceFilpClose(pFilp);
                                        if(isPreempt()==0)
                                        rcu_assign_pointer( pFDT->fd[count], NULL );
                                     }
                                     gobi_flush_work();
                                     GobiSyncRcu();
                                 }
                                 else
                                {
                                    if(isPreempt()==0)
                                    rcu_assign_pointer( pFDT->fd[count], NULL );
                                    GobiSyncRcu();
                                }
                             }
                          }
                          
                       }
                    }
                }
             }
          }
       }
   }
   gobi_flush_work();
   return ;
}

/*===========================================================================
METHOD:
   gobi_flush_work (Public Method)

DESCRIPTION:
   sync memory

PARAMETERS:
   None

RETURN VALUE:
   None
===========================================================================*/
void gobi_flush_work(void)
{
    GobiSyncRcu();
    if(isPreempt()==0)
    wait_ms(500);
    GobiSyncRcu();
    return ;
}

/*===========================================================================
METHOD:
   DeregisterQMIDevice (Public Method)

DESCRIPTION:
   QMI Device cleanup function

   NOTE: When this function is run the device is no longer valid

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
void DeregisterQMIDevice( sGobiUSBNet * pDev )
{
   char qcqmi_dev_name[10]={0};
   int tries = 0;
   int result = -1;
   int i = 0;
   pDev->mbUnload = eStatUnloading;

   // Should never happen, but check anyway
   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "wrong device\n" );
      KillRead( pDev );
      // Send SetControlLineState request (USB_CDC)
      result = usb_control_msg( pDev->mpNetDev->udev,
                             usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
                             SET_CONTROL_LINE_STATE_REQUEST,
                             SET_CONTROL_LINE_STATE_REQUEST_TYPE,
                             0, // DTR not present
                             /* USB interface number to receive control message */
                             pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
                             NULL,
                             0,
                             100 );
      pDev->mbUnload = eStatUnloaded;
      StopSemID(pDev);

      return;
   }
   if(pDev->mQMIDev.proc_file != NULL)
   {
      sprintf(qcqmi_dev_name, "qcqmi%d", (int)pDev->mQMIDev.qcqmi);
      remove_proc_entry(qcqmi_dev_name, NULL);
      pDev->mQMIDev.proc_file = NULL;
      DBG("remove:%s",qcqmi_dev_name);
   }

   if(pDev->WDSClientID!=(u16)-1)
   ReleaseClientID( pDev, pDev->WDSClientID,NULL );

   tries = 0;
   pDev->mQMIDev.mCdev.ops = NULL;
   while(LocalClientMemLockSpinIsLock(pDev)!=0)
   {
      gobi_flush_work();
      if((tries%10)==0)
      {
         printk("Spinlocked\n");
      }
      if(200> tries++)
      {
         break;
      }
   }
   tries = 0;
   do
   {
      int ref = 0;
      ref = atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount );
      DBG("%s:%d tries:%d ref:%d\n",__FUNCTION__,__LINE__,tries,ref);
      if (ref > 1)
      {       
         CloseFileInode(pDev);
         gobi_flush_work();
      }
      else
      {
         break;
      }
   }while(20> tries++);

   StopSemID(pDev);
   gobi_flush_work();
   // Release all clients
   while (pDev->mQMIDev.mpClientMemList != NULL)
   {
      DBG( "release 0x%04X\n", pDev->mQMIDev.mpClientMemList->mClientID );

      if (ReleaseClientID(pDev,
                       pDev->mQMIDev.mpClientMemList->mClientID,NULL) == false)
          break;
      // NOTE: pDev->mQMIDev.mpClientMemList will
      //       be updated in ReleaseClientID()
      gobi_flush_work();
   }
   gobi_flush_work();

   // Stop all reads
   KillRead( pDev );

   pDev->mbQMIValid = false;

   if (pDev->mQMIDev.mbCdevIsInitialized == false)
   {
      pDev->mbUnload = eStatUnloaded;
      return;
   }

   // Find each open file handle, and manually close it

   // Generally there will only be only one inode, but more are possible
   
   if(pDev->iShutdown_write_sem>0)
   {
      up_write(&pDev->shutdown_rwsem);
   }

   if(pDev->iShutdown_read_sem>0)
   {
      up_read(&pDev->shutdown_rwsem);
   }
   i =0;
   while(!down_trylock( &(pDev->ReadsyncSem) ))
   {
      i++;
      if(i>MAX_RETRY_LOCK_NUMBER)
      {
         break;
      }
      set_current_state(TASK_INTERRUPTIBLE);
      wait_ms(MAX_RETRY_LOCK_MSLEEP_TIME);
      if(signal_pending(current))
      {
        break;
      }
      if(pDev==NULL)
      {
         return ;
      }
   }
   up(&(pDev->ReadsyncSem));
   set_current_state(TASK_RUNNING);
   // Send SetControlLineState request (USB_CDC)
   result = usb_control_msg( pDev->mpNetDev->udev,
                             usb_sndctrlpipe( pDev->mpNetDev->udev, 0 ),
                             SET_CONTROL_LINE_STATE_REQUEST,
                             SET_CONTROL_LINE_STATE_REQUEST_TYPE,
                             0, // DTR not present
                             /* USB interface number to receive control message */
                             pDev->mpIntf->cur_altsetting->desc.bInterfaceNumber,
                             NULL,
                             0,
                             100 );
   if (result < 0)
   {
      DBG( "Bad SetControlLineState status %d\n", result );
   }

   // Remove device (so no more calls can be made by users)
   if (IS_ERR( pDev->mQMIDev.mpDevClass ) == false)
   {
      device_destroy( pDev->mQMIDev.mpDevClass,
                      pDev->mQMIDev.mDevNum );
   }

   qcqmi_table[pDev->mQMIDev.qcqmi] = 0;

   // Hold onto cdev memory location until everyone is through using it.
   // Timeout after 30 seconds (10 ms interval).  Timeout should never happen,
   // but exists to prevent an infinate loop just in case.

   for (tries = 0; tries < 60; tries++)
   {
      int ref = atomic_read( &pDev->mQMIDev.mCdev.kobj.kref.refcount );
      if (ref > 1)
      {
         printk( KERN_WARNING "cdev in use by %d tasks\n", ref - 1 );
         CloseFileInode(pDev);
         wait_ms(500);
      }
      else
      {
         break;
      }
   }

   cdev_del( &pDev->mQMIDev.mCdev );

   unregister_chrdev_region( pDev->mQMIDev.mDevNum, 1 );
   pDev->mbUnload = eStatUnloaded;
   return;
}

/*=========================================================================*/
// Driver level client management
/*=========================================================================*/

/*===========================================================================
METHOD:
   QMIReady (Public Method)

DESCRIPTION:
   Send QMI CTL GET VERSION INFO REQ and SET DATA FORMAT REQ
   Wait for response or timeout

PARAMETERS:
   pDev     [ I ] - Device specific memory
   timeout  [ I ] - Milliseconds to wait for response

RETURN VALUE:
   bool
===========================================================================*/
bool QMIReady(
   sGobiUSBNet *    pDev,
   u16                timeout )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   struct semaphore readSem;
   u16 curTime;
   u8 transactionID;
   unsigned long flags;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return false;
   }

   writeBufferSize = QMICTLReadyReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return false;
   }

   // An implimentation of down_timeout has not been agreed on,
   //    so it's been added and removed from the kernel several times.
   //    We're just going to ignore it and poll the semaphore.

   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );
   // Send a write every 1000 ms and see if we get a response
   for (curTime = 0; curTime < timeout; curTime += 1000)
   {
      if(kthread_should_stop())
      {
         set_current_state(TASK_RUNNING);
         return -1;
      }
      if(signal_pending(current))
      {
         set_current_state(TASK_RUNNING);
         return -1;
      }
      // Start read
      set_current_state(TASK_INTERRUPTIBLE);
      transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
      if (transactionID == 0)
      {
         transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
      }
      result = ReadAsync( pDev, QMICTL, transactionID, UpSem, &readSem );
      if (result != 0)
      {
         kfree( pWriteBuffer );
         return false;
      }

      // Fill buffer
      result = QMICTLReadyReq( pWriteBuffer,
                               writeBufferSize,
                               transactionID );
      if (result < 0)
      {
         kfree( pWriteBuffer );
         return false;
      }

      // Disregard status.  On errors, just try again
      WriteSync( pDev,
                 pWriteBuffer,
                 writeBufferSize,
                 QMICTL );
      if(kthread_should_stop())
      {
         set_current_state(TASK_RUNNING);
         return -1;
      }
      if(signal_pending(current))
      {
         set_current_state(TASK_RUNNING);
         return -1;
      }

      if(curTime>=0)
      {
         int iScaleCount = 0;
         for(iScaleCount=0;iScaleCount<100;iScaleCount++)
         {
            if( (kthread_should_stop()) ||
                signal_pending(current))
            {
               if(pWriteBuffer)
               kfree(pWriteBuffer);
               set_current_state(TASK_RUNNING);
               return false;
            }
            wait_ms(10);//msleep( 10 );
            if( (kthread_should_stop()) ||
                signal_pending(current))
            {
               if(pWriteBuffer)
               kfree(pWriteBuffer);
               set_current_state(TASK_RUNNING);
               return false;
            }
            if(isModuleUnload(pDev))
            {
               if(pWriteBuffer)
               kfree(pWriteBuffer);
               set_current_state(TASK_RUNNING);
               return false;
            }
         }
         
      }

      if (down_trylock( &readSem ) == 0)
      {
         // Enter critical section
         flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

         // Pop the read data
         if (PopFromReadMemList( pDev,
                                 QMICTL,
                                 0, //ignore transaction id
                                 &pReadBuffer,
                                 &readBufferSize ) == true)
         {
            // Success

            // End critical section
            LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

            // We don't care about the result
            if(pReadBuffer)
            kfree( pReadBuffer );

            break;
         }
         else
         {
            // Read mismatch/failure, unlock and continue
            LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         }
      }
      else
      {
         if(pDev->mbUnload < eStatUnloading)
         {
             // Enter critical section
             flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
             // Timeout, remove the async read
             RemoveAndPopNotifyList(pDev,QMICTL,0);
             // End critical section
             LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
         }
         
      }
   }
   kfree( pWriteBuffer );
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);
   RemoveAndPopNotifyList(pDev,QMICTL,0);
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   // Did we time out?
   if (curTime >= timeout)
   {
      return false;
   }
   set_current_state(TASK_RUNNING);

   DBG( "QMI Ready after %u milliseconds\n", curTime );

   // Success
   return true;
}

/*===========================================================================
METHOD:
   QMIWDSCallback (Public Method)

DESCRIPTION:
   QMI WDS callback function
   Update net stats or link state

PARAMETERS:
   pDev     [ I ] - Device specific memory
   clientID [ I ] - Client ID
   pData    [ I ] - Callback data (unused)

RETURN VALUE:
   None
===========================================================================*/
void QMIWDSCallback(
   sGobiUSBNet *    pDev,
   u16                clientID,
   void *             pData )
{
   bool bRet;
   int result;
   void * pReadBuffer=NULL;
   u16 readBufferSize;
   u32 TXOk = (u32)-1;
   u32 RXOk = (u32)-1;
   u32 TXErr = (u32)-1;
   u32 RXErr = (u32)-1;
   u32 TXOfl = (u32)-1;
   u32 RXOfl = (u32)-1;
   u64 TXBytesOk = (u64)-1;
   u64 RXBytesOk = (u64)-1;
   bool bReconfigure;
   unsigned long flags;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return;
   }
   
   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

   bRet = PopFromReadMemList( pDev,
                              clientID,
                              0,
                              &pReadBuffer,
                              &readBufferSize );

   // End critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

   if (bRet == false)
   {
      DBG( "WDS callback failed to get data\n" );
      if(pReadBuffer)
         kfree( pReadBuffer );
      pReadBuffer = NULL;
      return;
   }

   // Default values
   pDev->bLinkState = ! GobiTestDownReason( pDev, NO_NDIS_CONNECTION );
   bReconfigure = false;

   result = QMIWDSEventResp( pReadBuffer,
                             readBufferSize,
                             &TXOk,
                             &RXOk,
                             &TXErr,
                             &RXErr,
                             &TXOfl,
                             &RXOfl,
                             &TXBytesOk,
                             &RXBytesOk,
                             (u8*)&pDev->bLinkState,
                             &bReconfigure );
   if (result < 0)
   {
      DBG( "bad WDS packet\n" );
   }
   else
   {
      if (bReconfigure == true)
      {
         DBG( "Net device link reset\n" );
         GobiSetDownReason( pDev, NO_NDIS_CONNECTION );
         GobiClearDownReason( pDev, NO_NDIS_CONNECTION );
      }
      else
      {
         if (pDev->bLinkState == true)
         {
            DBG( "Net device link is connected\n" );
            GobiClearDownReason( pDev, NO_NDIS_CONNECTION );
         }
         else
         {
            DBG( "Net device link is disconnected\n" );
            GobiSetDownReason( pDev, NO_NDIS_CONNECTION );
         }
      }
   }
   if(pReadBuffer)
   kfree( pReadBuffer );
   pReadBuffer = NULL;

   // Setup next read
   result = ReadAsync( pDev,
                       clientID,
                       0,
                       QMIWDSCallback,
                       pData );
   if (result != 0)
   {
      DBG( "unable to setup next async read\n" );
   }

   return;
}

void QMIQOSCallback(
   sGobiUSBNet *    pDev,
   u16                clientID,
   void *             pData )
{
   bool bRet;
   int result;
   void * pReadBuffer;
   u16 readBufferSize;
   unsigned long flags;
   if (IsDeviceValid( pDev ) == false)
   {
      QDBG( "Invalid device\n" );
      return;
   }

   // Critical section
   flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

   bRet = PopFromReadMemList( pDev,
                              clientID,
                              0,
                              &pReadBuffer,
                              &readBufferSize );

   // End critical section
   LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

   if (bRet == false)
   {
      QDBG( "QOS callback failed to get data\n" );
      return;
   }

   result = QMIQOSEventResp(pDev, pReadBuffer, readBufferSize);

   if (result < 0)
   {
      QDBG( "bad QOS packet\n" );
   }
   if(pReadBuffer)
   kfree( pReadBuffer );

   // Setup next read
   result = ReadAsync( pDev,
                       clientID,
                       0,
                       QMIQOSCallback,
                       pData );
   if (result != 0)
   {
      QDBG( "unable to setup next async read\n" );
   }

   return;
}

/*===========================================================================
METHOD:
   SetupQMIWDSCallback (Public Method)

DESCRIPTION:
   Request client and fire off reqests and start async read for
   QMI WDS callback

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   int - 0 for success
         Negative errno for failure
===========================================================================*/
int SetupQMIWDSCallback( sGobiUSBNet * pDev )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   u16 WDSClientID;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return -EFAULT;
   }

   result = GetClientID( pDev, QMIWDS,NULL );
   if (result < 0)
   {
      return result;
   }
   pDev->WDSClientID = WDSClientID = result;

   // QMI WDS Set Event Report
   writeBufferSize = QMIWDSSetEventReportReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIWDSSetEventReportReq( pWriteBuffer,
                                     writeBufferSize,
                                     1 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       WDSClientID );
   kfree( pWriteBuffer );

   if (result < 0)
   {
      return result;
   }

   // QMI WDS Get PKG SRVC Status
   writeBufferSize = QMIWDSGetPKGSRVCStatusReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIWDSGetPKGSRVCStatusReq( pWriteBuffer,
                                       writeBufferSize,
                                       2 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       WDSClientID );
   kfree( pWriteBuffer );

   if (result < 0)
   {
      return result;
   }

   // Setup asnyc read callback
   result = ReadAsync( pDev,
                       WDSClientID,
                       0,
                       QMIWDSCallback,
                       NULL );
   if (result != 0)
   {
      DBG( "unable to setup async read\n" );
      return result;
   }

   return 0;
}

/*===========================================================================
METHOD:
   QMIDMSSWISetFCCAuth (Public Method)

DESCRIPTION:
   Register DMS client
   send FCC Authentication req and parse response
   Release DMS client

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
int QMIDMSSWISetFCCAuth( sGobiUSBNet * pDev )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u16 DMSClientID;
   int iID = -1;

   DBG("\n");

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return -EFAULT;
   }

   result = GetClientID( pDev, QMIDMS ,NULL);
   if (result < 0)
   {
      return result;
   }
   DMSClientID = result;

   // QMI DMS Get Serial numbers Req
   writeBufferSize = QMIDMSSWISetFCCAuthReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIDMSSWISetFCCAuthReq( pWriteBuffer,
                                    writeBufferSize,
                                    1 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }
   iID = iGetSemID(pDev,__LINE__);

   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       DMSClientID );
   kfree( pWriteBuffer );

   if (result < 0)
   {
      pDev->iReasSyncTaskID[iID]=-__LINE__;
      return result;
   }

   // QMI DMS Get Serial numbers Resp
   result = ReadSync( pDev,
                      &pReadBuffer,
                      DMSClientID,
                      1,
                      &iID,NULL,NULL);
   if (result < 0)
   {
      return result;
   }
   readBufferSize = result;

//   result = QMIDMSSWISetFCCAuthResp( pReadBuffer,
//                                     readBufferSize );
   if(pReadBuffer)
   kfree( pReadBuffer );

   if (result < 0)
   {
      // Non fatal error, device did not return FCC Auth response
      DBG( "Bad FCC Auth resp\n" );
   }
   ReleaseClientID( pDev, DMSClientID,NULL );

   // Success
   return 0;
}

/*===========================================================================
METHOD:
   QMIDMSGetMEID (Public Method)

DESCRIPTION:
   Register DMS client
   send MEID req and parse response
   Release DMS client

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
int QMIDMSGetMEID( sGobiUSBNet * pDev )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u16 DMSClientID;
   int iID = -1;

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return -EFAULT;
   }

   result = GetClientID( pDev, QMIDMS ,NULL);
   if (result < 0)
   {
      return result;
   }
   DMSClientID = result;

   // QMI DMS Get Serial numbers Req
   writeBufferSize = QMIDMSGetMEIDReqSize();
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIDMSGetMEIDReq( pWriteBuffer,
                              writeBufferSize,
                              1 );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   iID = iGetSemID(pDev,__LINE__);
   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       DMSClientID );
   kfree( pWriteBuffer );

   if (result < 0)
   {
      pDev->iReasSyncTaskID[iID]=-__LINE__;
      return result;
   }

   // QMI DMS Get Serial numbers Resp
   result = ReadSync( pDev,
                      &pReadBuffer,
                      DMSClientID,
                      1,
                      &iID,NULL,NULL);
   if (result < 0)
   {
      return result;
   }
   readBufferSize = result;

   result = QMIDMSGetMEIDResp( pReadBuffer,
                               readBufferSize,
                               &pDev->mMEID[0],
                               14 );
   kfree( pReadBuffer );

   if (result < 0)
   {
      DBG( "bad get MEID resp\n" );

      // Non fatal error, device did not return any MEID
      //    Fill with 0's
      memset( &pDev->mMEID[0], '0', 14 );
   }

   ReleaseClientID( pDev, DMSClientID ,NULL);

   // always return Success as MEID is only available on CDMA devices only
   return 0;
}

/*===========================================================================
METHOD:
   QMICTLSetDataFormat (Public Method)

DESCRIPTION:
   send Data format request and parse response

PARAMETERS:
   pDev     [ I ] - Device specific memory

RETURN VALUE:
   None
===========================================================================*/
int QMICTLSetDataFormat( sGobiUSBNet * pDev )
{
   u8 transactionID;
   struct semaphore readSem;
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   unsigned long flags;

   DBG("\n");

   // Send SET DATA FORMAT REQ
   writeBufferSize = QMICTLSetDataFormatReqSize();

   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   // Start read
   sema_init( &readSem, SEMI_INIT_DEFAULT_VALUE );

   transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
   if (transactionID == 0)
   {
      transactionID = atomic_add_return( 1, &pDev->mQMIDev.mQMICTLTransactionID );
   }

   result = ReadAsync( pDev, QMICTL, transactionID, UpSem, &readSem );
   if (result != 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   // Fill buffer
   result = QMICTLSetDataFormatReq( pWriteBuffer,
                            writeBufferSize,
                            transactionID );

   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }

   DBG("Sending QMI Set Data Format Request, TransactionID: 0x%x\n", transactionID );

   WriteSync( pDev,
              pWriteBuffer,
              writeBufferSize,
              QMICTL );

   //msleep( 100 );
   if (down_trylock( &readSem ) == 0)
   {
      // Enter critical section
      flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

      // Pop the read data
      if (PopFromReadMemList( pDev,
                              QMICTL,
                              transactionID,
                              &pReadBuffer,
                              &readBufferSize ) == true)
      {
         // Success
         PrintHex(pReadBuffer, readBufferSize);

         // End critical section
         LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);

         // We care about the result: call Response  function
         result = QMICTLSetDataFormatResp( pReadBuffer, readBufferSize);
         if(pReadBuffer)
         kfree( pReadBuffer );

         if (result != 0)
         {
            DBG( "Device cannot set requested data format\n" );
            if(pWriteBuffer)
            kfree( pWriteBuffer );
            return result;
         }
      }
   }
   else
   {
      // Enter critical section
      flags = LocalClientMemLockSpinLockIRQSave( pDev , __LINE__);

      // Timeout, remove the async read
      NotifyAndPopNotifyList( pDev, QMICTL, transactionID );

      // End critical section
      LocalClientMemUnLockSpinLockIRQRestore ( pDev ,flags,__LINE__);
   }

   kfree( pWriteBuffer );

   return 0;
}

/*===========================================================================
METHOD:
   QMIWDASetDataFormat (Public Method)

DESCRIPTION:
   Register WDA client
   send Data format request and parse response
   Release WDA client

PARAMETERS:
   pDev            [ I ] - Device specific memory
   te_flow_control [ I ] - TE Flow Control Flag

RETURN VALUE:
   None
===========================================================================*/
int QMIWDASetDataFormat( sGobiUSBNet * pDev, bool te_flow_control )
{
   int result;
   void * pWriteBuffer;
   u16 writeBufferSize;
   void * pReadBuffer;
   u16 readBufferSize;
   u16 WDAClientID;
   int iID = -1;
   DBG("\n");

   if (IsDeviceValid( pDev ) == false)
   {
      DBG( "Invalid device\n" );
      return -EFAULT;
   }

   result = GetClientID( pDev, QMIWDA ,NULL);
   if (result < 0)
   {
      return result;
   }
   WDAClientID = result;

   // QMI WDA Set Data Format Request
   writeBufferSize = QMIWDASetDataFormatReqSize(te_flow_control);
   pWriteBuffer = kmalloc( writeBufferSize, GFP_KERNEL );
   if (pWriteBuffer == NULL)
   {
      return -ENOMEM;
   }

   result = QMIWDASetDataFormatReq( pWriteBuffer,
                                    writeBufferSize,
                                    1,
                                    te_flow_control );
   if (result < 0)
   {
      kfree( pWriteBuffer );
      return result;
   }
   iID = iGetSemID(pDev,__LINE__);
   result = WriteSync( pDev,
                       pWriteBuffer,
                       writeBufferSize,
                       WDAClientID );
   kfree( pWriteBuffer );

   if (result < 0)
   {
      pDev->iReasSyncTaskID[iID]=-__LINE__;
      return result;
   }

   // QMI DMS Get Serial numbers Resp
   result = ReadSync( pDev,
                      &pReadBuffer,
                      WDAClientID,
                      1,
                      &iID,NULL,NULL);
   if (result < 0)
   {
      return result;
   }
   readBufferSize = result;

   result = QMIWDASetDataFormatResp( pReadBuffer,
                                     readBufferSize );
   if(pReadBuffer)
   kfree( pReadBuffer );

   if (result < 0)
   {
      DBG( "Data Format Cannot be set\n" );
   }
   ReleaseClientID( pDev, WDAClientID ,NULL);

   // Success
   return result;
}

void wait_ms(unsigned int ms) {
   if(in_atomic())
   {
      DBG("preempt_ensabled\n");
      return ;
   }
   if (!in_interrupt()) {
       set_current_state(TASK_UNINTERRUPTIBLE);
       schedule_timeout(1 + ms * HZ / 1000);
       set_current_state(TASK_RUNNING);
   }
   else
   {
         mdelay(ms);
   }
}

/*===========================================================================
METHOD:
   RemoveAndPopNotifyList (Public Method)

DESCRIPTION:
   Remove first Notify entry from this client's notify list
   and Run function

   Caller MUST have lock on mClientMemLock

PARAMETERS:
   pDev              [ I ] - Device specific memory
   clientID          [ I ] - Requester's client ID
   transactionID     [ I ] - Transaction ID or 0 for any

RETURN VALUE:
   bool
===========================================================================*/
int RemoveAndPopNotifyList(
   sGobiUSBNet *      pDev,
   u16              clientID,
   u16              transactionID )
{
   sClientMemList * pClientMem;
   sNotifyList * pDelNotifyList, ** ppNotifyList;

   if(pDev==NULL)
   {
      DBG("NULL");
      return eNotifyListEmpty;
   }
#ifdef CONFIG_SMP
   // Verify Lock
   if (LocalClientMemLockSpinIsLock( pDev ) == 0)
   {
      DBG( "unlocked\n" );
      BUG();
   }
#endif
   do
   {
      // Get this client's memory location
      pClientMem = FindClientMem( pDev, clientID );
      if (pClientMem == NULL)
      {
         DBG( "Could not find this client's memory 0x%04X\n", clientID );
         return eNotifyListEmpty;
      }

      ppNotifyList = &(pClientMem->mpReadNotifyList);
      pDelNotifyList = NULL;
      // Remove from list
      CLIENT_READMEM_SNAPSHOT(clientID,pDev);
      while (*ppNotifyList != NULL)
      {
         // Do we care about transaction ID?
         if (transactionID == 0
         ||  (*ppNotifyList)->mTransactionID == 0
         ||  transactionID == (*ppNotifyList)->mTransactionID)
         {
            pDelNotifyList = *ppNotifyList;
            DBG( "Remove Notify TID = %x\n", (*ppNotifyList)->mTransactionID );
            break;
         }

         DBG( "skipping data TID = %x\n", (*ppNotifyList)->mTransactionID );

         // next
         ppNotifyList = &(*ppNotifyList)->mpNext;
      }
      if (pDelNotifyList != NULL)
      {
         // Remove element
         *ppNotifyList = (*ppNotifyList)->mpNext;

         // Delete memory
         kfree( pDelNotifyList );
      }
      else
      {
         void *pFreeData = NULL;
         u16 FreeDataSize;
         //Remove From memory List
         while(PopFromReadMemList( pDev,
             clientID,
             transactionID,
             &pFreeData,
             &FreeDataSize ) == true )
         {
             DBG( "Remove Mem ClientID: 0x%x, data TID = 0x%x\n", clientID,transactionID);
             kfree( pFreeData );
             pFreeData = NULL;
         }
         DBG( "no one to notify for TID 0x%x\n", transactionID );
         return eNotifyListNotFound;
      }
   }while(ppNotifyList!=NULL);
}

