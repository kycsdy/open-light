/*
 * This file was ported by Stijn Kuipers / Zephod from a part of the OpenKinect 
 * Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <conio.h>

#include "Kinect-win32.h"
#include "Kinect-win32-internal.h"

#define DORGB 1
#define DODEPTH 1

namespace Kinect
{
	#include "init.h"

	DWORD WINAPI DepthThread( LPVOID lpParam ) 
	{ 
		KinectInternalData *KID  = (KinectInternalData*) lpParam;
		if (DODEPTH)
		{
		}
		else
		{
			return 0;
		};

		KID->LockDepthThread();
		KID->mDepthInput = new KinectFrameInput(KID, KID->mDeviceHandle, 0x82, 1760, DEPTH_PKTS_PER_XFER, DEPTH_NUM_XFERS, 422400);
		if (KID->mDepthInput)
		{
		
			while (KID->DepthRunning)
			{
				int loopcount = 0;
				while (	KID->mDepthInput->Reap() && loopcount++ < 20);
				//if(loopcount == 20) printf("too much depth");
				Sleep(1);

			};
			delete KID->mDepthInput;
			KID->mDepthInput = NULL;
		};

		KID->UnlockDepthThread();
		
		return 0;
	};


	DWORD WINAPI RGBThread( LPVOID lpParam ) 
	{ 
		KinectInternalData *KID  = (KinectInternalData*) lpParam;
		if (DORGB)
		{
		}
		else
		{
			return 0;
		};
		KID->LockRGBThread();
		KID->mRGBInput = new KinectFrameInput(KID, KID->mDeviceHandle, 0x81, 1920, RGB_PKTS_PER_XFER, RGB_NUM_XFERS, 307200 );
		if (KID->mRGBInput )
		{
		
			while (KID->RGBRunning)
			{
				int loopcount = 0;
				while (	KID->mRGBInput->Reap() && loopcount++ < 20);
				//if(loopcount == 20) printf("too much depth");
				Sleep(1);

			};
			delete KID->mRGBInput;
			KID->mRGBInput = NULL;
		};

		KID->UnlockRGBThread();
		return 0;
	};

	
	struct cam_hdr {
		uint8_t magic[2];
		uint16_t len;
		uint16_t cmd;
		uint16_t tag;
	};

	void KinectInternalData::send_init()
	{

		int i, j, ret;
		uint8_t obuf[0x2000];
		uint8_t ibuf[0x2000];
		ZeroMemory(obuf, 0x2000);
		ZeroMemory(ibuf, 0x2000);
		
		cam_hdr *chdr = (cam_hdr *)obuf;
		cam_hdr *rhdr = (cam_hdr *)ibuf;		
		ret = 0;	
		
		ret = usb_control_msg(mDeviceHandle, 0x80, 0x06, 0x3ee, 0, (char*)ibuf, 0x12, 500);
		if (ret <0)
		{
			//	this call is expected to stall!
		};


		chdr->magic[0] = 0x47;
		chdr->magic[1] = 0x4d;

//	Addition by maa nov 16th 2010
//	This table keeps track of which init codes need extra sleep time
		CONST int bs = 1;
		int sleep[num_inits*2] =
		{
			0,0,	//1
			0,0,
			0,0,
			0,0,
			0,bs,	//5
			0,0,	//6
			0,0,
			0,0,
			0,0,
			0,0,
			0,0,	//11
			0,0,
			0,0,
			0,0,
			0,0,	
			0,bs,	//16
			0,0,
			0,0,
			0,0,
			0,0,
			0,bs,	//21
			0,0,
			0,0,
			0,0,
			0,0,
			0,0,	//26
			0,bs,	//27
			0,0,
		};

		for (i=0; i<num_inits; i++) 
		{
			if( sleep[2*i]!=0 )
			{
				Sleep(sleep[2*i]);	//maa
			};

			//Sleep(100);
			//printf("doing init %d\n", i);
			const struct caminit *ip = &inits[i];
			chdr->cmd = ip->command;
			chdr->tag = ip->tag;
			chdr->len = ip->cmdlen / 2;
			memcpy(obuf+sizeof(cam_hdr), ip->cmddata, ip->cmdlen);
			ret = usb_control_msg(mDeviceHandle, 0x40, 0, 0, 0, (char*)obuf, ip->cmdlen + sizeof(cam_hdr), 1600);
			if (ret <0)
			{
				printf("error: %s\n", usb_strerror());
				//return;
			}
            if(mDebugInfo)
			    printf("sending init %d from %d... ", i+1, num_inits);
			
			do 
			{
				if( sleep[2*i+1]!=0 )
				{
					Sleep(sleep[2*i+1]);	//maa
				}

				ret = usb_control_msg(mDeviceHandle, 0xc0, 0, 0, 0, (char*)ibuf, 0x200, 1600);
				if (ret<0)
				{
					printf("error: %s\n", usb_strerror());				
				}

			} while (ret == 0);

			if (rhdr->magic[0] != 0x52 || rhdr->magic[1] != 0x42) 
			{
				printf("Bad magic %02x %02x\n", rhdr->magic[0], rhdr->magic[1]);
				continue;
			}
            if(mDebugInfo)
			    printf("succes!\n");


			if (rhdr->cmd != chdr->cmd) 
			{
				printf("Bad cmd %02x != %02x\n", rhdr->cmd, chdr->cmd);
				continue;
			}

			if (rhdr->tag != chdr->tag) 
			{
				printf("Bad tag %04x != %04x\n", rhdr->tag, chdr->tag);
				continue;
			}

			if (rhdr->len != (ret-sizeof(*rhdr))/2) 
			{
				printf("Bad len %04x != %04x\n", rhdr->len, (int)(ret-sizeof(*rhdr))/2);
				continue;
			}
			
			if (rhdr->len != (ip->replylen/2) || memcmp(ibuf+sizeof(*rhdr), ip->replydata, ip->replylen)) 
			{
				printf("Expected: ");
				for (j=0; j<ip->replylen; j++) {
					printf("%02x ", ip->replydata[j]);
				}
				printf("\nGot:      ");
				for (j=0; j<(rhdr->len*2); j++) {
					printf("%02x ", ibuf[j+sizeof(*rhdr)]);
				}
				printf("\n");
			}
		}
	}

	void KinectInternalData::cams_init()
	{		
		send_init();
		RunThread();
	}

	KinectInternalData::~KinectInternalData()
	{
		if (mDeviceHandle)
		{
			if (mDeviceHandle_Motor)
			{
				usb_close(mDeviceHandle_Motor);
				mDeviceHandle_Motor = NULL;
			};
			if (mDeviceHandle_Audio)
			{
				usb_close(mDeviceHandle_Audio);
				mDeviceHandle_Motor = NULL;
			};
			Running = false;
			DepthRunning = false;
			LockDepthThread();
			UnlockDepthThread();
			RGBRunning = false;
			LockRGBThread();
			UnlockRGBThread();
			usb_reset(mDeviceHandle);
			mParent->KinectDisconnected();
			usb_close(mDeviceHandle);
		};
	};



	void KinectInternalData::RunThread()
	{
		DWORD did, rid ;
		if (DODEPTH)
		{
			HANDLE depththread = CreateThread(NULL,0,DepthThread,this,0,&did);   
			SetThreadPriority(depththread, THREAD_PRIORITY_TIME_CRITICAL);
		};
		if (DORGB)
		{
			HANDLE rgbthread = CreateThread(NULL,0,RGBThread,this,0,&rid);   
			SetThreadPriority(rgbthread, THREAD_PRIORITY_TIME_CRITICAL);
		};
		//		return;

		
		ThreadDone = true;
	};


	void KinectInternalData::OpenDevice(usb_device_t *dev, usb_device_t *motordev)
	{
		mDeviceHandle = usb_open(dev);
		if (!mDeviceHandle) 
		{				
			return;
		}

		mDeviceHandle_Motor = usb_open(motordev); // dont check for null... just dont move when asked and the pointer is null
		
		int ret;
		ret = usb_set_configuration(mDeviceHandle, 1);
		if (ret<0)
		{
			printf("usb_set_configuration error: %s\n", usb_strerror());
			//return;
		}

		ret = usb_claim_interface(mDeviceHandle, 0);
		ret = usb_set_configuration(mDeviceHandle, 1);

		if (ret<0)
		{
			printf("usb_claim_interface error: %s\n", usb_strerror());
			return;
		}

		usb_clear_halt(mDeviceHandle, 0x81);usb_clear_halt(mDeviceHandle, 0x82);
		cams_init();
	};

	void KinectInternalData::SetMotorPosition(double newpos)
	{
		if (mDeviceHandle_Motor)
		{
			if (newpos>1) newpos = 1;if(newpos<0) newpos = 0;
			unsigned char tiltValue = (unsigned char)(newpos*255);
			unsigned short value = (unsigned short)(0xffd0 + tiltValue / 5);
			
			usb_control_msg(mDeviceHandle_Motor, 0x40, 0x31, value, 0, NULL, 0, 160);		
		};
	};

	void KinectInternalData::SetLedMode(unsigned short NewMode)
	{
		if (mDeviceHandle_Motor)
		{			
            usb_control_msg(mDeviceHandle_Motor, 0x40, 0x06, NewMode, 0, NULL, 0, 160);
		};
	};

	bool KinectInternalData::GetAcceleroData(float *x, float *y, float *z)
	{
		if (mDeviceHandle_Motor)
		{
			unsigned char outbuf[10];
			if (usb_control_msg(mDeviceHandle_Motor, 0xC0, 0x32, 0, 0, (char*)outbuf, 10, 1000)>0)
			{
				unsigned short sx = outbuf[3] + (outbuf[2]<<8);
				unsigned short sy = outbuf[5] + (outbuf[3]<<8);
				unsigned short sz = outbuf[7] + (outbuf[6]<<8);
				short ix = *(short*)(&sx);
				short iy = *(short*)(&sy);
				short iz = *(short*)(&sz);
				*x = ix/512.0f;			
				*y = iy/512.0f;
				*z = iz/512.0f;
				return true;
			};
		};
		return false;
	};

	KinectInternalData::KinectInternalData(Kinect *inParent)
	{
		mParent = inParent;

		mDeviceHandle = NULL;
		mDeviceHandle_Audio = NULL;
		mDeviceHandle_Motor = NULL;
		
		mErrorCount = 0;

		mDepthInput = NULL;
		mRGBInput = NULL;
		

		ThreadDone = false;
		Running = true;
		RGBRunning = true;
		DepthRunning = true;

		depth_sourcebuf2 = new uint8_t[1000*1000*3];
		rgb_buf2= new uint8_t[640*480*3];

        mDebugInfo = false;

		InitializeCriticalSection(&depth_lock);
		InitializeCriticalSection(&rgb_lock);
		InitializeCriticalSection(&depththread_lock);
		InitializeCriticalSection(&rgbthread_lock);
	}

	void KinectInternalData::BufferComplete(KinectFrameInput *source)
	{
		if (source == mDepthInput)
		{
			LockDepth();
				memcpy(depth_sourcebuf2, source->mOutputBuffer, 422400);
			UnlockDepth();		
			mParent->DepthReceived();
			return;
		};

		if (source == mRGBInput)
		{
			LockRGB();
			memcpy(rgb_buf2,source->mOutputBuffer, 307200 );
			UnlockRGB();

			mParent->ColorReceived();			return;
		};
	};
};