 /*  NAME:
        ControllerCoreOSX.c

    DESCRIPTION:
        Implementation of Quesa controller API calls. Main part of the
		ControllerCore library under MacOS X.
		
		Under MacOS X the communication between driver, device server and client
		is implemented via IPC. 
		This source file defines the lower functions used by the Quesa framework to 
		communicate with the device server. It packs passed data, sends it to the
		device server and unpacks received data to be returned to the caller.
		
		Methods are defined to support controller, tracker and controller states.
		Under MacOS X there is currently no global 3D system cursor.
      
    COPYRIGHT:
        Copyright (c) 1999-2005, Quesa Developers. All rights reserved.

        For the current release of Quesa, please see:

            <http://www.quesa.org/>
        
        Redistribution and use in source and binary forms, with or without
        modification, are permitted provided that the following conditions
        are met:
        
            o Redistributions of source code must retain the above copyright
              notice, this list of conditions and the following disclaimer.
        
            o Redistributions in binary form must reproduce the above
              copyright notice, this list of conditions and the following
              disclaimer in the documentation and/or other materials provided
              with the distribution.
        
            o Neither the name of Quesa nor the names of its contributors
              may be used to endorse or promote products derived from this
              software without specific prior written permission.
        
        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
        A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
        OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
        SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
        TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
        PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
        LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
        NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
        SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    ___________________________________________________________________________
*/

//=============================================================================
//      Include files
//-----------------------------------------------------------------------------
#include "E3Prefix.h"				
#include "ControllerCoreOSX.h"
#include "IPCMessageIDs.h"
#include "IPCPackUnpack.h"






//=============================================================================
//      Internal constants
//-----------------------------------------------------------------------------
// Internal constants go here





//=============================================================================
//      Internal types
//-----------------------------------------------------------------------------
// Internal types go here


typedef struct TC3TrackerEvent
{
	TQ3Uns32		EventTimeStamp;
	TQ3Uns32		EventButtons;
	TQ3Point3D		EventPosition;
	TQ3Boolean		EventPositionIsNULL;
	TQ3Quaternion	EventOrientation;
	TQ3Boolean		EventOrientationIsNULL;
} TC3TrackerEvent, *TC3TrackerEventPtr;

typedef struct TC3TrackerInstanceData
{
	float			posThreshold;
	float			oriThreshold;
	TQ3TrackerNotifyFunc 
					theNotifyFunc;
					
	TQ3Uns32		theButtons;
	TQ3Point3D		thePosition;
	TQ3Vector3D		accuMoving;
	TQ3Quaternion	theOrientation;
	TQ3Quaternion	accuOrientation;
	
	//TQ3Uns32		theSerialNum;
	TQ3Uns32		oriSerialNum;
	TQ3Uns32		posSerialNum;
	TQ3Boolean		isActive;
	
	CFMutableArrayRef
					eventsRingBuffer;
	
	
	CFUUIDRef		trackerUUID;
	TQ3Object		tracker_self;	//tracker_self is needed by notification functions
	
} TC3TrackerInstanceData;

typedef struct TC3ControllerStateInstanceData
{
	TQ3ControllerRef	myController;
	CFStringRef			ctrlStateUUIDString;
} TC3ControllerStateInstanceData;

//=============================================================================
//      Internal macros
//-----------------------------------------------------------------------------
// Internal macros go here


/*=============================================================================
* =============================================================================
* Notes about Q3Controller:
*
* Q3Controller will not be registered as an object of QD3D
*
* originally Q3Controller was running out of a library inside the QD3D INIT 
* 
* In current implementation deviation against QD3D:
* there is !no! tracker held in background for the system cursor (aka mouse pointer),
* that would be updated on demand in case no other tracker was assigned to a controller 
*
* =============================================================================
* =============================================================================*/

/*
Details for implementation of moving the system cursor under MacOS X can be found 
in CGRemoteOperation.h! see: CGPostMouseEvent
*/

//=============================================================================
//      Internal variables
//-----------------------------------------------------------------------------
// Internal variables go here

/*
DeviceServerPort:
-is used to talk to device server; 
-created on first call of a device driver function
*/
static CFMessagePortRef			DeviceServerPort = NULL;

/*
ClientTrackers:
-is used for IPC bookkeeping of trackers created by a controller client
-created at first creation of a tracker oject
*/	
static CFMutableDictionaryRef	ClientTrackers	 = NULL;

/*
TrackerServerPort and TrackerRunLoopSource:
-Device server talks to client trackers via such a port
-created at first creation of a tracker object
*/
static CFMessagePortRef			TrackerServerPort = NULL;
static CFRunLoopSourceRef 		TrackerRunLoopSource = NULL;

/*
DeviceDriverPort and DeviceDriverRunLoopSource:
-allocated if GetChannel or SetChannel methods not NULL;
-Device server talks to device driver trackers via such a port
-created by CC3OSXController_New
*/
static CFMessagePortRef			DeviceDriverPort = NULL;
static CFRunLoopSourceRef 		DeviceDriverRunLoopSource = NULL;

//=============================================================================
//      Internal function prototypes
//-----------------------------------------------------------------------------
// Internal function prototypes go here

#pragma mark -
//=============================================================================
//      Internal functions
//-----------------------------------------------------------------------------
// Internal functions go here

//=============================================================================
//      CC3Quaternion_GetAngle : Get the rotation angle
//				represented by a quaternion.
//-----------------------------------------------------------------------------
//based on E3Quaternion_GetAxisAndAngle
//
TQ3Status 
CC3Quaternion_GetAngle(const TQ3Quaternion *q, float *outAngle)
{
	float w_temp;
	
	//fill w_temp;
	w_temp=q->w/sqrt(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
	
	if (w_temp > 1.0f - kQ3RealZero || w_temp < -1.0f + kQ3RealZero)
		{
		// |w| = 1 means this is a null rotation.
		// Return 0 for the angle, and pick an arbitrary axis.
		if (outAngle) *outAngle = 0.0f;
		}
	else
		{
		// |w| != 1, so we have a non-null rotation.
		// w is the cosine of the half-angle.
		if (outAngle)
			*outAngle = 2.0f * (float) acos(w_temp);
		}

	return (kQ3Success);
}

//=============================================================================
//      CC3Quaternion_Multiply : Multiply 'q2' by 'q1'.
//-----------------------------------------------------------------------------
//		Note : 'result' may be the same as 'q1' and/or 'q2'.
//
// identical to E3Quaternion_Multiply
// inserted to avoid circular linking
//
TQ3Quaternion *
CC3Quaternion_Multiply(const TQ3Quaternion *q1, const TQ3Quaternion *q2, TQ3Quaternion *result)
{
	// If result is alias of input, output to temporary
	TQ3Quaternion temp;
	TQ3Quaternion* output = (result == q1 || result == q2 ? &temp : result);

	// Reverse multiplication (q2 * q1)
	output->w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
	output->x = q1->w*q2->x + q1->x*q2->w - q1->y*q2->z + q1->z*q2->y;
	output->y = q1->w*q2->y + q1->y*q2->w - q1->z*q2->x + q1->x*q2->z;
	output->z = q1->w*q2->z + q1->z*q2->w - q1->x*q2->y + q1->y*q2->x;
	
	if (output == &temp)
		*result = temp;
	
	return(result);
}


#pragma mark -

TQ3Status
IPCControllerDriver_SetChannel(CFDictionaryRef dict, CFMutableDictionaryRef returnDict)
{
	TQ3Status			status = kQ3Failure;	//resulting status after calling driver method
	Boolean 			result;
	
	TQ3ControllerRef 	controllerRef;
	TQ3ChannelSetMethod theSetChannelMethod;
	TQ3Uns32            channel, dataSize;
	void 				*data;
	
	//method
	result = IPCGetBytes(dict, CFSTR(k3MethodRef), sizeof(TQ3ChannelSetMethod), &theSetChannelMethod);
		
	//controllerRef
	result = IPCGetControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
	//channel
	result = IPCGetTQ3Uns32(dict, CFSTR(k3Channel), &channel);
										
	//dataSize
	result = IPCGetTQ3Uns32(dict, CFSTR(k3DataSize), &dataSize);
		
	//data
	data = (void*)CFDataGetBytePtr((CFDataRef)CFDictionaryGetValue(dict, CFSTR(k3Data)));
	
	//do call
	status = theSetChannelMethod(controllerRef, channel, data, dataSize);
	
	return status;
}

TQ3Status
IPCControllerDriver_GetChannel(CFDictionaryRef dict, CFMutableDictionaryRef returnDict)
{
	TQ3Status			status = kQ3Failure;	//resulting status after calling tracker method
	Boolean 			result;
	
	TQ3ControllerRef 	controllerRef;
	TQ3ChannelGetMethod theGetChannelMethod;
	TQ3Uns32            channel, dataSize;
	void 				*data;
	
	//method
	result = IPCGetBytes(dict, CFSTR(k3MethodRef), sizeof(TQ3ChannelGetMethod), &theGetChannelMethod);
		
	//controllerRef
	result = IPCGetControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
	//channel
	result = IPCGetTQ3Uns32(dict, CFSTR(k3Channel), &channel);
										
	//dataSize
	result = IPCGetTQ3Uns32(dict, CFSTR(k3DataSize), &dataSize);
	
	//data
	//to do: sanity check for dataSize
	data = malloc(dataSize);
	
	//do call
	status = theGetChannelMethod(controllerRef, channel, data, &dataSize);
	
	//return:
	//data
	IPCPutBytes(returnDict, CFSTR(k3Data), dataSize, data);
	free(data);
	
	//dataSize
	result = IPCPutTQ3Uns32(returnDict, CFSTR(k3DataSize), &dataSize);
		
	return status;
}

TQ3Status
IPCControllerDriver_StateSaveAndReset(CFDictionaryRef dict, CFMutableDictionaryRef returnDict)
{
	TQ3Status			status = kQ3Failure;	//resulting status after calling driver method
	Boolean 			result;
	
	TQ3ControllerRef 	controllerRef;
	TQ3ChannelSetMethod theSetChannelMethod = NULL;
	TQ3ChannelGetMethod theGetChannelMethod = NULL;
	
	TQ3Uns32            channel, channelCount, dataSize;
	UInt8				channelData[kQ3ControllerSetChannelMaxDataSize];
	
	//Set method
	result = IPCGetBytes(dict, CFSTR(k3SetMethodRef), sizeof(TQ3ChannelSetMethod), &theSetChannelMethod);
	
	//Get method
	result = IPCGetBytes(dict, CFSTR(k3GetMethodRef), sizeof(TQ3ChannelGetMethod), &theGetChannelMethod);
		
	//controllerRef
	result = IPCGetControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
	
	//channelCount
	result = IPCGetTQ3Uns32(dict, CFSTR(k3ChannelCount), &channelCount);		
	
	//-Create Array
	CFMutableArrayRef channelArrayRef = CFArrayCreateMutable(kCFAllocatorDefault, channelCount, &kCFTypeArrayCallBacks);
		
	for (channel=0; channel<channelCount; channel++)
	{
		dataSize = kQ3ControllerSetChannelMaxDataSize;
		if (theGetChannelMethod!=NULL)
		{
			//--get channel data
			status = theGetChannelMethod(controllerRef, channel, channelData, &dataSize);
			
			//--create array element
			CFDataRef ChannelDataRef = CFDataCreate(kCFAllocatorDefault, (UInt8*)channelData, dataSize);
			
			//--store array element in array
			CFArrayAppendValue(channelArrayRef, ChannelDataRef);
			CFRelease(ChannelDataRef);
		}
			
		if (theSetChannelMethod!=NULL)
		{ 
			//--set channel data to NULL
			dataSize=0;
			status = theSetChannelMethod(controllerRef, channel, NULL, dataSize);
		}
	}
	
	//return Array
	CFDictionarySetValue(returnDict, CFSTR(k3ChannelsData), channelArrayRef);
	
	//-clean up
	if (channelArrayRef)
		CFRelease(channelArrayRef);
	
	return status;
}

TQ3Status
IPCControllerDriver_StateRestore(CFDictionaryRef dict, CFMutableDictionaryRef returnDict)
{
	TQ3Status			status = kQ3Failure;	//resulting status after calling driver method
	Boolean 			result;
	
	TQ3ControllerRef 	controllerRef;
	TQ3ChannelSetMethod theSetChannelMethod = NULL;
	
	TQ3Uns32            channel, channelCount, dataSize;
	void				*channelData;
	CFDataRef			channelDataRef;
	CFArrayRef			channelArrayRef;
	
	//Set method
	result = IPCGetBytes(dict, CFSTR(k3SetMethodRef), sizeof(TQ3ChannelSetMethod), &theSetChannelMethod);
	
	//controllerRef
	result = IPCGetControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
	
	//channelCount
	result = IPCGetTQ3Uns32(dict, CFSTR(k3ChannelCount), &channelCount);
	
	//Array
	channelArrayRef = (CFArrayRef)CFDictionaryGetValue(dict, CFSTR(k3ChannelsData));
	
	//Do what to do:
	for (channel=0; channel<channelCount; channel++)
	{
		status = kQ3Failure;
		//-for each channel
		if (theSetChannelMethod!=NULL)
		{ 
			//--set channel data from array
			channelDataRef = (CFDataRef)CFArrayGetValueAtIndex(channelArrayRef, channel);
			dataSize = CFDataGetLength(channelDataRef);
			channelData = (void*)CFDataGetBytePtr(channelDataRef);
			
			status = theSetChannelMethod(controllerRef, channel, channelData, dataSize);
		}
	}

	return status;
}


/*
IPCControllerDriver_Dispatcher will be called by CFMessagePort on incoming message 
*/
CFDataRef IPCControllerDriver_Dispatcher ( CFMessagePortRef local, SInt32 msgid, CFDataRef data, void *info)
{
	TQ3Status 				status;
		
	CFDictionaryRef			dict;
	CFMutableDictionaryRef	returnDict;
	
	CFStringRef				propertyListError;
	CFDataRef 				returnData;
	Boolean					result;
	
	dict=(CFDictionaryRef)CFPropertyListCreateFromXMLData(	kCFAllocatorDefault, 
															data, 
															kCFPropertyListImmutable, 
															&propertyListError);
	if (propertyListError)
	{
		//Do error handling
		
		CFRelease(propertyListError);
	}
	//nothing else to be released! dict will be used over function; the data parameter will be deallocated at exit!
	
	//create dictionary for return parameters
	returnDict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
											&kCFTypeDictionaryKeyCallBacks,
											&kCFTypeDictionaryValueCallBacks);
	
	if (returnDict)
	{
		//Dispatch msgid to local functions
		switch(msgid)
		{
			case m3ControllerDriver_SetChannel:
				status = IPCControllerDriver_SetChannel(dict, returnDict);
				break;
			case m3ControllerDriver_GetChannel:
				status = IPCControllerDriver_GetChannel(dict, returnDict);
				break;
			case m3ControllerDriver_StateSaveAndReset:
				status = IPCControllerDriver_StateSaveAndReset(dict, returnDict);
				break;
			case m3ControllerDriver_StateRestore:
				status = IPCControllerDriver_StateRestore(dict, returnDict);
				break;	
			default:
				break;
		}
	}
	if (dict)
		CFRelease(dict);
	
	//status
	result = IPCPutTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		
	//returnDict to CFDataRef
	returnData= CFPropertyListCreateXMLData(kCFAllocatorDefault,returnDict);
	
	if (returnDict)
		CFRelease(returnDict);
	
	return returnData;
};


/*
IPCControllerDriver_PortCreate:
- creates Messageport in case of callback functions for setting and getting controller channels
*/

void IPCControllerDriver_PortCreate(CFMutableDictionaryRef 	dict, const TQ3ControllerData *controllerData)
{
	if ((controllerData->channelGetMethod!=NULL)||(controllerData->channelSetMethod!=NULL))
	{
		//put extra signature value into dict
		CFMutableStringRef DriverPortName = CFStringCreateMutable (kCFAllocatorDefault,0);
		CFStringAppend(DriverPortName,CFSTR(kQuesa3DeviceDriver));
		CFStringAppend(DriverPortName,CFSTR("."));
		CFUUIDRef DriverUUID = CFUUIDCreate(kCFAllocatorDefault);
		CFStringRef DriverUUIDString = CFUUIDCreateString(kCFAllocatorDefault,DriverUUID);
		CFStringAppend(DriverPortName,DriverUUIDString);
		CFDictionarySetValue(dict,CFSTR(k3DriverPortName),DriverPortName);
		
		//create messageport for callbacks to tracker objects
		CFMessagePortContext	context;
	
		context.version = 0;
		context.info = NULL;//could be used as a pointer to a messagehandler, etc.
		context.retain = NULL;
		context.release = NULL;
		context.copyDescription = NULL;
	
		DeviceDriverPort = CFMessagePortCreateLocal(NULL, DriverPortName, IPCControllerDriver_Dispatcher, &context, NULL);
		DeviceDriverRunLoopSource = CFMessagePortCreateRunLoopSource(NULL, DeviceDriverPort, 0);
		CFRunLoopAddSource(CFRunLoopGetCurrent(), DeviceDriverRunLoopSource, kCFRunLoopDefaultMode);
		//do clean up
		if (DriverPortName)
			CFRelease(DriverPortName);
		if (DriverUUID)
			CFRelease(DriverUUID);
		if (DriverUUIDString)
			CFRelease(DriverUUIDString);
	}
}


TQ3Status IPCControllerDriver_Send( SInt32 msgid, CFMutableDictionaryRef dict, CFMutableDictionaryRef *returnDict)
{
	TQ3Status status = kQ3Failure;
	
	CFDataRef 	data,returnData;
	CFStringRef	propertyListError;
	
	data= CFPropertyListCreateXMLData(kCFAllocatorDefault,dict);
	//what to release?
	
	if (DeviceServerPort==NULL)
	{
		DeviceServerPort=CFMessagePortCreateRemote(kCFAllocatorDefault, CFSTR(kQuesa3DeviceServer));
		//Who does cleanup of DeviceServerPort? A custom exit-handler? Consider usage of atexit !		
	}
	
	SInt32 ReqRes = CFMessagePortSendRequest(	DeviceServerPort, msgid, data, 
												10, 10, kCFRunLoopDefaultMode,
												&returnData);
	
	*returnDict=NULL;								
																									
	if (ReqRes == kCFMessagePortSuccess)
	{								
		
		if (returnData)
		{
			*returnDict=(CFMutableDictionaryRef)CFPropertyListCreateFromXMLData(kCFAllocatorDefault, 
																				returnData, 
																				kCFPropertyListImmutable, 
																				&propertyListError);
			//was there an error? see propertyListError
			if (propertyListError)
				status=kQ3Failure;
			else	
				status=kQ3Success;
		}
		
		if (propertyListError)
			CFRelease(propertyListError);
		if (returnData)
			CFRelease(returnData);
	}
	
	if (data)
		CFRelease(data);

	return status;
};


//=============================================================================
//      Public functions
//-----------------------------------------------------------------------------
//      CC3OSXController_GetListChanged : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
#pragma mark -
TQ3Status
CC3OSXController_GetListChanged(TQ3Boolean *listChanged, TQ3Uns32 *serialNumber)
{
 	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//listChanged
		result = IPCPutTQ3Boolean(dict, CFSTR(k3ListChanged), listChanged);
		
		//serialNumber
		result = IPCPutTQ3Uns32(dict, CFSTR(k3SerNum), serialNumber);
		
		//try sending
		status = IPCControllerDriver_Send(m3Controller_GetListChanged, dict, &returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//listChanged
			result = IPCGetTQ3Boolean(returnDict, CFSTR(k3ListChanged), listChanged);
			
			//serialNumber
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3SerNum), serialNumber);						
															
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
												
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
};

//=============================================================================
//      CC3OSXController_Next : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_Next(TQ3ControllerRef controllerRef, TQ3ControllerRef *nextControllerRef)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//try sending
		status = IPCControllerDriver_Send(m3Controller_Next, dict, &returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//nextControllerRef
			result = IPCGetControllerRef(returnDict, CFSTR(k3CtrlRef), nextControllerRef);
						
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		}
		else *nextControllerRef=NULL;
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}





//=============================================================================
//      CC3OSXController_New : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// used on Driver Side
//-----------------------------------------------------------------------------
TQ3ControllerRef
CC3OSXController_New(const TQ3ControllerData *controllerData)
{
	TQ3Status 				status = kQ3Failure;
	TQ3ControllerRef 		controllerRef;
	CFMutableDictionaryRef 	dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		IPCControllerDriver_PortCreate(dict, controllerData);
		
		//Put structure controllerData into dict
		result = IPCPutBytes(dict, CFSTR(k3CtrlData), sizeof(TQ3ControllerData), controllerData);
				
		//send signature string seperate
		CFStringRef signature = CFStringCreateWithCString(	kCFAllocatorDefault,
															controllerData->signature, //should be null-terminated
															kCFStringEncodingASCII);
		CFDictionarySetValue(dict,CFSTR(k3Signature),signature);
		CFRelease(signature);													
				
		//try sending
		status = IPCControllerDriver_Send(m3Controller_New,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//controllerRef
			result = IPCGetControllerRef(returnDict, CFSTR(k3CtrlRef), &controllerRef);
						
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		}
		else controllerRef=NULL;
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	if (status==kQ3Failure)
		controllerRef=NULL;
	
	return controllerRef;
}




//=============================================================================
//      CC3OSXController_Decommission : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// used on Server/Driver Side
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_Decommission(TQ3ControllerRef controllerRef)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//try sending
		status = IPCControllerDriver_Send(m3Controller_Decommission,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}





//=============================================================================
//      CC3OSXController_SetActivation : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// QD3D:notification function of associated tracker might get called!			
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_SetActivation(TQ3ControllerRef controllerRef, TQ3Boolean active)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//active
		result = IPCPutTQ3Boolean(dict, CFSTR(k3Active), &active);
				
		//try sending
		status = IPCControllerDriver_Send(m3Controller_SetActivation,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}




//=============================================================================
//      CC3OSXController_GetActivation : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_GetActivation(TQ3ControllerRef controllerRef, TQ3Boolean *active)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
				
		//try sending
		status = IPCControllerDriver_Send(m3Controller_GetActivation,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//active
			result = IPCGetTQ3Boolean(returnDict, CFSTR(k3Active), active);
			
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}





//=============================================================================
//      CC3OSXController_GetSignature : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_GetSignature(TQ3ControllerRef controllerRef, char *signature, TQ3Uns32 numChars)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
				
		//try sending
		status = IPCControllerDriver_Send(m3Controller_GetSignature,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//signature
			char *tmpsig = (char*)malloc(256);
			/*
			result = CFStringGetCString((CFStringRef)CFDictionaryGetValue(returnDict,CFSTR(k3Signature)),
										tmpsig,
										256,
										kCFStringEncodingASCII);
										*/
			if (tmpsig!=NULL)
			{
				result = CFStringGetCString((CFStringRef)CFDictionaryGetValue(returnDict,CFSTR(k3Signature)),
										tmpsig,
										256,
										kCFStringEncodingASCII);
				strncpy(signature,tmpsig,numChars-1);
				signature[numChars-1] = '\0';
				free(tmpsig);
			}
			
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_SetChannel : One-line description of the method.
//-----------------------------------------------------------------------------
//		Only device clients will call CC3OSXController_SetChannel.
//		parameters will be transfered via IPC to the device driver.
//		the device server holds only the name of the device driver messageport.
//		the parameters are then forwarded by the server to the driver's messageport.
//		the device driver will call its channelSetMethod found in TQ3ControllerData.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_SetChannel(TQ3ControllerRef controllerRef, TQ3Uns32 channel, const void *data, TQ3Uns32 dataSize)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//channel
		result = IPCPutTQ3Uns32(dict, CFSTR(k3Channel), &channel);
				
		//data
		result = IPCPutBytes(dict, CFSTR(k3Data), dataSize, data);
		
		//dataSize
		result = IPCPutTQ3Uns32(dict, CFSTR(k3DataSize), &dataSize);
								
		//try sending
		status = IPCControllerDriver_Send(m3Controller_SetChannel,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}




//=============================================================================
//      CC3OSXController_GetChannel : One-line description of the method.
//-----------------------------------------------------------------------------
//		Only device clients will call CC3OSXController_GetChannel.
//		parameters will be transfered via IPC to the device driver.
//		the device server delivers only the name of device driver messageport.
//		the device driver will then call its channelGetMethod found in TQ3ControllerData.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_GetChannel(TQ3ControllerRef controllerRef, TQ3Uns32 channel, void *data, TQ3Uns32 *dataSize)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//channel
		result = IPCPutTQ3Uns32(dict, CFSTR(k3Channel), &channel);
				
		//dataSize
		result = IPCPutTQ3Uns32(dict, CFSTR(k3DataSize), dataSize);
										
		//try sending
		status = IPCControllerDriver_Send(m3Controller_GetChannel,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//get originaly returned dictionary from inside the returnDict
			CFMutableDictionaryRef 	workReturnDict = 
				(CFMutableDictionaryRef)CFDictionaryGetValue(returnDict,CFSTR(k3MethodsReturn));
				
			if (workReturnDict!=NULL)
			{
				
				//dataSize
				result = IPCGetTQ3Uns32(workReturnDict, CFSTR(k3DataSize), dataSize);
																		
				//data
				result = IPCGetBytes(workReturnDict, CFSTR(k3Data), *dataSize, data);
			}
			
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_GetValueCount : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_GetValueCount(TQ3ControllerRef controllerRef, TQ3Uns32 *valueCount)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
				
		//try sending
		status = IPCControllerDriver_Send(m3Controller_GetValueCount,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//valueCount
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3ValueCnt), valueCount);
									
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_SetTracker : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
//QD3D: notification function of old and new associated tracker might get called!
//used by controller client
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_SetTracker(TQ3ControllerRef controllerRef, TC3TrackerInstanceDataPtr tracker)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	CFDictionaryRef	trackers = ClientTrackers;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//equivalent of tracker
		//TrackerServerName
		CFStringRef TrackerServerName = (CFStringRef)CFDictionaryGetValue(trackers,CFSTR(k3TrackerPortName));
		CFDictionarySetValue(dict,CFSTR(k3TrackerPortName),TrackerServerName);
		
		//TrackerUUID instead of tracker if tracker is not NULL
		//NULL is a valid tracker!!
		if (tracker!=NULL)
		{
			CFStringRef TrackerUUIDString = CFUUIDCreateString(kCFAllocatorDefault,tracker->trackerUUID);
			CFDictionarySetValue(dict,CFSTR(k3TrackerUUID),TrackerUUIDString);
			CFRelease(TrackerUUIDString);
		}
		//try sending
		status = IPCControllerDriver_Send(m3Controller_SetTracker,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}



//=============================================================================
//      CC3OSXController_HasTracker : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_HasTracker(TQ3ControllerRef controllerRef, TQ3Boolean *hasTracker)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
				
		//try sending
		status = IPCControllerDriver_Send(m3Controller_HasTracker,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//hasTracker
			result = IPCGetTQ3Boolean(returnDict, CFSTR(k3hasTracker), hasTracker);
			
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_Track2DCursor : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// used on Server/Driver Side
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_Track2DCursor(TQ3ControllerRef controllerRef, TQ3Boolean *track2DCursor)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
				
		//try sending
		status = IPCControllerDriver_Send(m3Controller_Track2DCursor,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//track2DCursor
			result = IPCGetTQ3Boolean(returnDict, CFSTR(k3Track2DCrsr), track2DCursor);
			
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_Track3DCursor : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// used on Server/Driver Side
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_Track3DCursor(TQ3ControllerRef controllerRef, TQ3Boolean *track3DCursor)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
				
		//try sending
		status = IPCControllerDriver_Send(m3Controller_Track3DCursor,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//track3DCursor
			result = IPCGetTQ3Boolean(returnDict, CFSTR(k3Track3DCrsr), track3DCursor);
			
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}




//=============================================================================
//      CC3OSXController_GetButtons : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_GetButtons(TQ3ControllerRef controllerRef, TQ3Uns32 *buttons)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
				
		//try sending
		status = IPCControllerDriver_Send(m3Controller_GetButtons,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get parameters from returnDict
			//buttons
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Buttons), buttons);
			
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_SetButtons : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// QD3D:notification function of associated tracker might get called!
// used on Server/Driver Side
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_SetButtons(TQ3ControllerRef controllerRef, TQ3Uns32 buttons)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//buttons
		result = IPCPutTQ3Uns32(dict, CFSTR(k3Buttons), &buttons);
						
		//try sending
		status = IPCControllerDriver_Send(m3Controller_SetButtons,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_GetTrackerPosition : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_GetTrackerPosition(TQ3ControllerRef controllerRef, TQ3Point3D *position)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//try sending
		status = IPCControllerDriver_Send(m3Controller_GetTrackerPosition,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get result from returnDict
			//position
			result = IPCGetTQ3Point3D(returnDict,CFSTR(k3Position),position);
				
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_SetTrackerPosition : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// QD3D:notification function of associated tracker might get called!
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_SetTrackerPosition(TQ3ControllerRef controllerRef, const TQ3Point3D *position)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//position
		result = IPCPutTQ3Point3D(dict, CFSTR(k3Position), position);
						
		//try sending
		status = IPCControllerDriver_Send(m3Controller_SetTrackerPosition,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_MoveTrackerPosition : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// QD3D:notification function of associated tracker might get called!
// used on Server/Driver Side
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_MoveTrackerPosition(TQ3ControllerRef controllerRef, const TQ3Vector3D *delta)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//delta
		result = IPCPutTQ3Vector3D(dict, CFSTR(k3DeltaPos), delta);
						
		//try sending
		status = IPCControllerDriver_Send(m3Controller_MoveTrackerPosition,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_GetTrackerOrientation : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_GetTrackerOrientation(TQ3ControllerRef controllerRef, TQ3Quaternion *orientation)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//try sending
		status = IPCControllerDriver_Send(m3Controller_GetTrackerOrientation,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get result from returnDict
			//orientation
			result = IPCGetTQ3Quaternion(returnDict, CFSTR(k3Orient), orientation);
			
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_SetTrackerOrientation : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// QD3D:notification function of associated tracker might get called!
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_SetTrackerOrientation(TQ3ControllerRef controllerRef, const TQ3Quaternion *orientation)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//orientation
		result = IPCPutTQ3Quaternion(dict, CFSTR(k3Orient), orientation); 
						
		//try sending
		status = IPCControllerDriver_Send(m3Controller_SetTrackerOrientation,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}





//=============================================================================
//      CC3OSXController_MoveTrackerOrientation : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// QD3D:notification function of associated tracker might get called!
// used on Server/Driver Side
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_MoveTrackerOrientation(TQ3ControllerRef controllerRef, const TQ3Quaternion *delta)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//delta
		//orientation
		result = IPCPutTQ3Quaternion(dict, CFSTR(k3DeltaOrient), delta); 
						
		//try sending
		status = IPCControllerDriver_Send(m3Controller_MoveTrackerOrientation,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}






//=============================================================================
//      CC3OSXController_GetValues : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_GetValues(TQ3ControllerRef controllerRef, TQ3Uns32 valueCount, float *values, TQ3Boolean *changed, TQ3Uns32 *serialNumber)
{	
	TQ3Status 	status = kQ3Failure;
	TQ3Uns32 	privValueCount;
	TQ3Uns32 	tempSerNum;
	TQ3Boolean	active;
	
	CFMutableDictionaryRef dict,returnDict;
		
	Boolean result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//valueCount
		result = IPCPutTQ3Uns32(dict, CFSTR(k3ValueCount), &valueCount);
		
		//try sending
		status = IPCControllerDriver_Send(m3Controller_GetValues,dict,&returnDict);
		
		if (status!=kQ3Failure)
		{
			//Get result from returnDict
			//privValueCount
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3PrivValueCount), &privValueCount);
						
			//serialNumber
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3SerNum), &tempSerNum);
						
			//active (here: helper variable)
			result = IPCGetTQ3Boolean(returnDict, CFSTR(k3Active), &active);
			if (active)
			{
				//serialNumber NULL or not
				if (serialNumber==NULL)
				{
					if (privValueCount>0)
					{
						TQ3Uns32	maxCount,index;
						if (privValueCount > valueCount)
							maxCount=valueCount;
						else
							maxCount=privValueCount;
						
						CFArrayRef valAr = (CFArrayRef)CFDictionaryGetValue(dict,CFSTR(k3Values));
						if (valAr!=NULL)
							for (index=0; index<maxCount; index++)
								result = CFNumberGetValue(	(CFNumberRef)CFArrayGetValueAtIndex(valAr,index),
															kCFNumberFloatType,
															&values[index]);
						// CFRelease(valAr);
					}
					
					if (changed!=NULL)		
						*changed=kQ3True;
				}
				else
				{
					if (*serialNumber!=tempSerNum)
					{
						if (privValueCount>0)
						{
							TQ3Uns32	maxCount,index;
							if (privValueCount > valueCount)
								maxCount=valueCount;
							else
								maxCount=privValueCount;
							
							CFArrayRef valAr = (CFArrayRef)CFDictionaryGetValue(dict,CFSTR(k3Values));
							if (valAr!=NULL)
								for (index=0; index<maxCount; index++)
									result = CFNumberGetValue(	(CFNumberRef)CFArrayGetValueAtIndex(valAr,index),
																kCFNumberFloatType,
																&values[index]);
							// CFRelease(valAr);
						}
							
						if (changed!=NULL)		
							*changed=kQ3True;
					}
					else
						if (changed!=NULL)		
							*changed=kQ3False;
				}
			}
			
			if (serialNumber!=NULL)
				if (*serialNumber!=tempSerNum)
					*serialNumber = tempSerNum;
																								
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}





//=============================================================================
//      CC3OSXController_SetValues : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//
// Deviation from Apple-doc: Controller serialNumber is incremented!
// used on Server/Driver Side
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXController_SetValues(TQ3ControllerRef controllerRef, const float *values, TQ3Uns32 valueCount)
{
	TQ3Status status = kQ3Failure;
	CFMutableDictionaryRef dict,returnDict;
	
	Boolean		result;
	TQ3Uns32 	index;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//controllerRef
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
		
		//valueCount
		if (valueCount>kQ3MaxControllerValues) 
			valueCount = kQ3MaxControllerValues;
		result = IPCPutTQ3Uns32(dict, CFSTR(k3ValueCount), &valueCount);
				
		//values
		CFNumberRef *valuesInput = (CFNumberRef*)malloc(valueCount*sizeof(CFNumberRef));
		for (index=0; index<valueCount; index++)
			valuesInput[index]= CFNumberCreate(kCFAllocatorDefault,kCFNumberFloatType,&values[index]);
		CFArrayRef valuesRef = CFArrayCreate(kCFAllocatorDefault,(const void**)valuesInput,valueCount,&kCFTypeArrayCallBacks);
		for (index=0; index<valueCount; index++)
			CFRelease(valuesInput[index]);
		free(valuesInput);
		CFDictionarySetValue(dict,CFSTR(k3Values),valuesRef);
		CFRelease(valuesRef);
		
		//try sending
		status = IPCControllerDriver_Send(m3Controller_SetValues,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}

#pragma mark -

TQ3Status CC3OSXTracker_tryCall_notification_local(TQ3ControllerRef controllerRef,CFUUIDRef trackerUUID)
{
	TC3TrackerInstanceDataPtr 	theTrackerInstance;
	
	
	CFStringRef TrackerUUIDkey = CFUUIDCreateString(kCFAllocatorDefault,trackerUUID);
		
	CFDataGetBytes(	(CFDataRef)CFDictionaryGetValue(ClientTrackers,TrackerUUIDkey),//ClientTrackers is global!
											CFRangeMake(0,sizeof(TC3TrackerInstanceDataPtr)),
											(UInt8*)&theTrackerInstance);
											
	if (TrackerUUIDkey)
		CFRelease(TrackerUUIDkey);
	
	if (theTrackerInstance!=NULL)
	{
		if (theTrackerInstance->theNotifyFunc!=NULL)
			return theTrackerInstance->theNotifyFunc(theTrackerInstance->tracker_self,controllerRef);
		else	
			return(kQ3Failure);
	}
	else
	{
		//notification of System Cursor tracker not implemented!!
		return(kQ3Failure);
	};
}

TQ3Status CC3OSXTracker_tryCall_notification_disp(CFDictionaryRef dict, CFMutableDictionaryRef returnDict, TC3TrackerInstanceDataPtr trackerInstance)
{
	TQ3Status 					status = kQ3Failure;	//resulting status after calling tracker method
	TQ3ControllerRef 			theController;
	//TC3TrackerInstanceDataPtr	theInstanceData;
	TQ3Boolean					controllerHasTracker;
	
	//Get Parameters from dict
	//theController
	IPCGetControllerRef(dict, CFSTR(k3CtrlRef), &theController);
		
	CC3OSXController_HasTracker(theController,&controllerHasTracker);
	
	if (controllerHasTracker==kQ3True)
	{
		//do call
		/*if (theInstanceData->theNotifyFunc!=NULL)
			status = theInstanceData->theNotifyFunc(trackerInstance->tracker_self,theController);*/
		if (trackerInstance->theNotifyFunc!=NULL)
			status = trackerInstance->theNotifyFunc(trackerInstance->tracker_self,theController);	
	}
	/*
	else
	{
		//notification of System Cursor tracker not implemented!!
		status = kQ3Failure;
	};
	*/
	return status;
}


TQ3Status
CC3OSXTracker_changeButtons_disp(CFDictionaryRef dict, CFMutableDictionaryRef returnDict, TC3TrackerInstanceDataPtr trackerInstance)
{
	TQ3Status 			status = kQ3Failure;	//resulting status after calling tracker method
	TQ3ControllerRef 	controllerRef;
	TQ3Uns32 			buttons;
	TQ3Uns32 			buttonMask;
	
	Boolean 			result;

	//Get Parameters from dict
	//controllerRef
	result = IPCGetControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
								
	//buttons
	result = IPCGetTQ3Uns32(dict, CFSTR(k3Buttons), &buttons);
		
	//buttonMask
	result = IPCGetTQ3Uns32(dict, CFSTR(k3ButtonMask), &buttonMask);
		
	//do call						
	status = CC3OSXTracker_ChangeButtons(trackerInstance, controllerRef, buttons, buttonMask);
																			
	return status;
}

TQ3Status
CC3OSXTracker_getActivation_disp(CFDictionaryRef dict, CFMutableDictionaryRef returnDict, TC3TrackerInstanceDataPtr trackerInstance)
{
	TQ3Status		status = kQ3Failure;	//resulting status after calling tracker method
	TQ3Boolean 		active;

	//do call
	status=CC3OSXTracker_GetActivation(trackerInstance, &active);
	
	//Put parameters into dict
	//active
	IPCPutTQ3Boolean(returnDict, CFSTR(k3Active), &active);
		
	return status;
}

TQ3Status
CC3OSXTracker_getPosition_disp(CFDictionaryRef dict, CFMutableDictionaryRef returnDict, TC3TrackerInstanceDataPtr trackerInstance)
{
	TQ3Status	status = kQ3Failure;	//resulting status after calling tracker method
	TQ3Point3D 	position;
	
	//do call
	status = CC3OSXTracker_GetPosition(trackerInstance, &position, NULL, NULL, NULL);
	
	//Put parameters into dict
	//position
	IPCPutTQ3Point3D(returnDict, CFSTR(k3Position), &position);
	
	return status;
}

TQ3Status
CC3OSXTracker_setPosition_disp(CFDictionaryRef dict, CFMutableDictionaryRef returnDict, TC3TrackerInstanceDataPtr trackerInstance)
{
	TQ3Status			status = kQ3Failure;	//resulting status after calling tracker method
	TQ3ControllerRef 	controllerRef;
	TQ3Point3D 			position;
	
	Boolean				result;
	
	//Get Parameters from dict
	//controllerRef
	result = IPCGetControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
								
	//position
	result = IPCGetTQ3Point3D(dict, CFSTR(k3Position), &position);
	if (result==true)
	{
		//do call
		status=CC3OSXTracker_SetPosition(trackerInstance, controllerRef, &position);
	}
	return status;
}

TQ3Status
CC3OSXTracker_movePosition_disp(CFDictionaryRef dict, CFMutableDictionaryRef returnDict, TC3TrackerInstanceDataPtr trackerInstance)
{
	TQ3Status			status = kQ3Failure;	//resulting status after calling tracker method
	TQ3ControllerRef 	controllerRef;
	TQ3Vector3D 		delta;
	
	Boolean				result;
	
	//Get Parameters from dict
	//controllerRef
	result = IPCGetControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
								
	//delta
	result = IPCGetTQ3Vector3D(dict, CFSTR(k3DeltaPos), &delta);
	if (result==true)
	{
		//do call
		status=CC3OSXTracker_MovePosition(trackerInstance, controllerRef, &delta);
	}
	return status;
}

TQ3Status
CC3OSXTracker_getOrientation_disp(CFDictionaryRef dict, CFMutableDictionaryRef returnDict, TC3TrackerInstanceDataPtr trackerInstance)
{
	TQ3Status		status = kQ3Failure;	//resulting status after calling tracker method
	TQ3Quaternion 	orientation;
	
	//do call 
	status=CC3OSXTracker_GetOrientation(trackerInstance, &orientation, NULL, NULL, NULL);
	
	//Put parameters into dict
	//orientation
	IPCPutTQ3Quaternion(returnDict, CFSTR(k3Orient), &orientation);
		
	return status;
}

TQ3Status
CC3OSXTracker_setOrientation_disp(CFDictionaryRef dict, CFMutableDictionaryRef returnDict, TC3TrackerInstanceDataPtr trackerInstance)
{
	TQ3Status			status = kQ3Failure;	//resulting status after calling tracker method
	TQ3ControllerRef 	controllerRef;
	TQ3Quaternion 		orientation;
	
	Boolean				result;

	//Get Parameters from dict
	//controllerRef
	result = IPCGetControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
								
	//orientation
	result = IPCGetTQ3Quaternion(dict, CFSTR(k3Orient), &orientation);
	if (result==true)
	{
		//do call
		status=CC3OSXTracker_SetOrientation(trackerInstance, controllerRef, &orientation);
	}
	
	return status;
}

TQ3Status
CC3OSXTracker_moveOrientation_disp(CFDictionaryRef dict, CFMutableDictionaryRef returnDict, TC3TrackerInstanceDataPtr trackerInstance)
{
	TQ3Status			status = kQ3Failure;	//resulting status after calling tracker method
	TQ3ControllerRef 	controllerRef;
	TQ3Quaternion 		delta;
	
	Boolean				result;
	
	//Get Parameters from dict
	//controllerRef
	result = IPCGetControllerRef(dict, CFSTR(k3CtrlRef), &controllerRef);
								
	//orientation
	result = IPCGetTQ3Quaternion(dict, CFSTR(k3Orient), &delta);
	if (result==true)
	{
		//do call
		status=CC3OSXTracker_MoveOrientation(trackerInstance, controllerRef, &delta);
	}
	return status;
}


CFDataRef IPCTracker_Dispatcher ( CFMessagePortRef local, SInt32 msgid, CFDataRef data, void *info)
{
	TQ3Status 				status = kQ3Failure;
	
	CFDataRef 				returnData;
	
	CFDictionaryRef			dict;
	CFMutableDictionaryRef	returnDict;
	
	CFStringRef				propertyListError;
	
	dict=(CFDictionaryRef)CFPropertyListCreateFromXMLData(kCFAllocatorDefault, data, kCFPropertyListImmutable, &propertyListError);
	if (propertyListError)
	{
		//Do error handling
		
		CFRelease(propertyListError);
	}
	//nothing else to be released! dict will be in used over function; the data parameter will be deallocated at exit!
	
	//create dictionary for return parameters
	returnDict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
											&kCFTypeDictionaryKeyCallBacks,
											&kCFTypeDictionaryValueCallBacks);
	
	if (returnDict)
	{
		//get TrackerInstanceData from dictionary
		//trackerUUID - may be NULL, if key wasn't found in the dictionary				
		CFStringRef	trackerUUID = (CFStringRef)CFDictionaryGetValue(dict,CFSTR(k3TrackerUUID));//bug?
		
		if (trackerUUID)
		{
			TC3TrackerInstanceDataPtr 	trackerInstance;
			
			/*CFDataGetBytes(	CFDictionaryGetValue(ClientTrackers,trackerUUID),//ClientTrackers is global!
							CFRangeMake(0,sizeof(TC3TrackerInstanceDataPtr)),
							(UInt8*)&trackerInstance);*/
			IPCGetBytes(ClientTrackers,trackerUUID,sizeof(TC3TrackerInstanceDataPtr),&trackerInstance);				
			
			//Dispatch msgid to local functions
			switch(msgid)
			{
				case m3Tracker_ChangeButtons:
					status = CC3OSXTracker_changeButtons_disp(dict,returnDict,trackerInstance);
					break;
				case m3Tracker_GetActivation:
					status = CC3OSXTracker_getActivation_disp(dict,returnDict,trackerInstance);
					break;
				case m3Tracker_GetPosition:
					status = CC3OSXTracker_getPosition_disp(dict,returnDict,trackerInstance);
					break;
				case m3Tracker_SetPosition:
					status = CC3OSXTracker_setPosition_disp(dict,returnDict,trackerInstance);
					break;
				case m3Tracker_MovePosition:
					status = CC3OSXTracker_movePosition_disp(dict,returnDict,trackerInstance);
					break;
				case m3Tracker_GetOrientation:
					status = CC3OSXTracker_getOrientation_disp(dict,returnDict,trackerInstance);
					break;
				case m3Tracker_SetOrientation:
					status = CC3OSXTracker_setOrientation_disp(dict,returnDict,trackerInstance);
					break;
				case m3Tracker_MoveOrientation:
					status = CC3OSXTracker_moveOrientation_disp(dict,returnDict,trackerInstance);
					break;
				case m3Tracker_CallNotification:
					status = CC3OSXTracker_tryCall_notification_disp(dict,returnDict,trackerInstance);
					break;
				default:
					break;
			}
		}
	}	
	if (dict)
		CFRelease(dict);
	
	//status
	IPCPutTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
	
	//returnDict to CFDataRef
	returnData= CFPropertyListCreateXMLData(kCFAllocatorDefault,returnDict);
	
	if (returnDict)
		CFRelease(returnDict);
	
	return returnData;
};

void IPCTracker_Insert(CFMutableDictionaryRef* 	trackersDict, const TC3TrackerInstanceDataPtr theTrackerInstanceData)
{
	CFMutableDictionaryRef dict = *trackersDict;
	
	if(dict==NULL)
	{
		//create dictionary
		dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
											&kCFTypeDictionaryKeyCallBacks,
											&kCFTypeDictionaryValueCallBacks);
											
		*trackersDict = dict;									
		//create an unique name representing the client process and its messageport
		CFMutableStringRef TrackerPortName = CFStringCreateMutable (kCFAllocatorDefault,0);
		CFStringAppend(TrackerPortName,CFSTR(kQuesa3DeviceTracker));
		CFStringAppend(TrackerPortName,CFSTR("."));
		CFUUIDRef ClientUUID = CFUUIDCreate(kCFAllocatorDefault);
		CFStringRef ClientUUIDString = CFUUIDCreateString(kCFAllocatorDefault,ClientUUID);
		CFStringAppend(TrackerPortName,ClientUUIDString);
		//insert to dictionary
		CFDictionarySetValue(dict,CFSTR(k3TrackerPortName),TrackerPortName);
		
		//create messageport for callbacks to tracker objects
		CFMessagePortContext	context;
	
		context.version = 0;
		context.info = NULL;//could be used as a pointer to a messagehandler, etc.
		context.retain = NULL;
		context.release = NULL;
		context.copyDescription = NULL;
	
		TrackerServerPort = CFMessagePortCreateLocal(NULL, TrackerPortName, IPCTracker_Dispatcher, &context, NULL);
		TrackerRunLoopSource = CFMessagePortCreateRunLoopSource(NULL, TrackerServerPort, 0);
		CFRunLoopAddSource(CFRunLoopGetCurrent(), TrackerRunLoopSource, kCFRunLoopDefaultMode);
		//do clean up
		if (TrackerPortName)
			CFRelease(TrackerPortName);
		if (ClientUUID)
			CFRelease(ClientUUID);
		if (ClientUUIDString)
			CFRelease(ClientUUIDString);
	}
	
	//insert theInstanceData to dictionary
	//key is UUID inside theInstanceData
	CFStringRef TrackerUUIDkey = CFUUIDCreateString(kCFAllocatorDefault, theTrackerInstanceData->trackerUUID);
	//value is theTrackerObject itself (theInstanceData is a pointer)
	CFDataRef TrackerObjectRef = CFDataCreate(	kCFAllocatorDefault,
												(UInt8*)&theTrackerInstanceData,
												sizeof(TC3TrackerInstanceDataPtr));
	
	CFDictionarySetValue(dict,TrackerUUIDkey,TrackerObjectRef);
	
	//do clean up
	if (TrackerObjectRef)
		CFRelease(TrackerObjectRef);
	if (TrackerUUIDkey)
		CFRelease(TrackerUUIDkey);
	
	return;
}

void IPCTracker_Remove(CFMutableDictionaryRef 	dict,const TC3TrackerInstanceDataPtr theInstanceData)
{
#warning IPCTracker_Remove to be completed...
	//remove theInstanceData from dictionary
	//key is UUID inside theInstanceData
	CFStringRef TrackerUUIDkey = CFUUIDCreateString(kCFAllocatorDefault,theInstanceData->trackerUUID);
	
	CFDictionaryRemoveValue(dict,TrackerUUIDkey);
	
	if (TrackerUUIDkey)
		CFRelease(TrackerUUIDkey);
	
	//the last tracker is removed! Questions:
	//what happens to dict?
	//what happens to TrackerServerPort?
	//Who does cleanup of everything used by the IPCTracker-functions? A custom exit-handler? Consider usage of atexit !
	/*
	CFRunLoopRemoveSource(CFRunLoopGetCurrent(), TrackerRunLoopSource, kCFRunLoopDefaultMode);
	CFRelease(TrackerRunLoopSource);
	CFRelease(TrackerServerPort);
	*/
	return;
}


//=============================================================================
//      CC3OSXTracker_New : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
#pragma mark -
TC3TrackerInstanceDataPtr
CC3OSXTracker_New(TQ3Object theObject, TQ3TrackerNotifyFunc notifyFunc)
{
	TC3TrackerInstanceDataPtr theInstanceData;
	theInstanceData = (TC3TrackerInstanceDataPtr)malloc(sizeof(TC3TrackerInstanceData));
		
	if (theInstanceData==NULL) 
		return NULL;
		
	theInstanceData->eventsRingBuffer = CFArrayCreateMutable(kCFAllocatorDefault,10,&kCFTypeArrayCallBacks);	
	
	theInstanceData->posThreshold = 0.0;
	theInstanceData->oriThreshold = 0.0;
	
	//create UUID and add to theInstanceData;
	//UUID will later be key for ClientTracker dictionary
	theInstanceData->trackerUUID = CFUUIDCreate(kCFAllocatorDefault);
	
	//do bookkeeping for IPCTrackerDispatcher
	IPCTracker_Insert(&ClientTrackers, theInstanceData);
	
	if (notifyFunc!=NULL)
		theInstanceData->theNotifyFunc = notifyFunc;
	else
		theInstanceData->theNotifyFunc = NULL;
	
	theInstanceData->tracker_self = theObject;
	
	theInstanceData->theButtons = 0;

	theInstanceData->thePosition.x = 0.0;
	theInstanceData->thePosition.y = 0.0;
	theInstanceData->thePosition.z = 0.0;

	theInstanceData->accuMoving.x = 0.0;
	theInstanceData->accuMoving.y = 0.0;
	theInstanceData->accuMoving.z = 0.0;

	theInstanceData->theOrientation.w = 1.0;
	theInstanceData->theOrientation.x = 0.0;
	theInstanceData->theOrientation.y = 0.0;
	theInstanceData->theOrientation.z = 0.0;
	
	theInstanceData->accuOrientation.w = 1.0;
	theInstanceData->accuOrientation.x = 0.0;
	theInstanceData->accuOrientation.y = 0.0;
	theInstanceData->accuOrientation.z = 0.0;

	//theInstanceData->theSerialNum = 1;
	theInstanceData->posSerialNum = 1;	
	theInstanceData->oriSerialNum = 1;		
	//NULL is a valid input serialNumber for GetPosition and GetOrientatien
	
	theInstanceData->isActive = kQ3True; //??? unclear documentation; seems to be active after New
		
	return(theInstanceData);
}



/*
CC3OSXTracker_Delete:
- is counterpart to CC3OSXTracker_New
- is exported in CC3OSXControllerCore.h
- needs to be registered inside Quesa to let Q3Object_Dispose(aTQ3TrackerObject) work
- registered in E3Initialize.c as kQ3SharedTypeTracker
*/

//=============================================================================
//      CC3OSXTracker_Delete : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TC3TrackerInstanceDataPtr
CC3OSXTracker_Delete(TC3TrackerInstanceDataPtr trackerObject)
{
	//do bookkeeping for IPCTrackerDispatcher
	IPCTracker_Remove(ClientTrackers,trackerObject);
	
	CFRelease(trackerObject->eventsRingBuffer);
	
	free (trackerObject);
	return(NULL);
}

//=============================================================================
//      CC3OSXTracker_SetNotifyThresholds : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_SetNotifyThresholds(TC3TrackerInstanceDataPtr trackerObject, float positionThresh, float orientationThresh)
{
	trackerObject->posThreshold = positionThresh;
	trackerObject->oriThreshold = orientationThresh;
	return(kQ3Success);
}





//=============================================================================
//      CC3OSXTracker_GetNotifyThresholds : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_GetNotifyThresholds(TC3TrackerInstanceDataPtr trackerObject, float *positionThresh, float *orientationThresh)
{
	*positionThresh = trackerObject->posThreshold;
	*orientationThresh = trackerObject->oriThreshold;
	return(kQ3Success);
}






//=============================================================================
//      CC3OSXTracker_SetActivation : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_SetActivation(TC3TrackerInstanceDataPtr trackerObject, TQ3Boolean active)
{
	if (trackerObject->isActive != active)
	{
		//trackerObject->theSerialNum++;
		//trackerObject->posSerialNum++;//TBC
		//trackerObject->oriSerialNum++;//TBC
		trackerObject->isActive = active;
	}
	return(kQ3Success);
}





//=============================================================================
//      CC3OSXTracker_GetActivation : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_GetActivation(TC3TrackerInstanceDataPtr trackerObject, TQ3Boolean *active)
{
	*active = trackerObject->isActive;
	return(kQ3Success);
}





//=============================================================================
//      CC3OSXTracker_GetButtons : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_GetButtons(TC3TrackerInstanceDataPtr trackerObject, TQ3Uns32 *buttons)
{
	*buttons = trackerObject->theButtons;
	trackerObject->theButtons = 0;//needed for logical OR, if Tracker is shared by several controllers
	return(kQ3Success);
}




//=============================================================================
//      CC3OSXTracker_ChangeButtons : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_ChangeButtons(TC3TrackerInstanceDataPtr trackerObject, TQ3ControllerRef controllerRef, TQ3Uns32 buttons, TQ3Uns32 buttonMask)
{

	trackerObject->theButtons=trackerObject->theButtons | (buttons & buttonMask);
	
	//trackerObject->theSerialNum++;	//would make sense; see "activation count" inside Apple QD3D doc
	
	if (trackerObject->isActive==kQ3True)
		CC3OSXTracker_tryCall_notification_local(controllerRef,trackerObject->trackerUUID);
		
	return(kQ3Success);
}





//=============================================================================
//      CC3OSXTracker_GetPosition : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_GetPosition(TC3TrackerInstanceDataPtr trackerObject, TQ3Point3D *position, TQ3Vector3D *delta, TQ3Boolean *changed, TQ3Uns32 *serialNumber)
{
	if (trackerObject->isActive==kQ3True)
	{
		//serialNumber NULL or not
		if (serialNumber==NULL)
		{
			if (position!=NULL)	
				*position = trackerObject->thePosition;
				
			if (delta!=NULL)
				*delta = trackerObject->accuMoving;
				
			if (changed!=NULL)		
				*changed=kQ3True;
		}
		else
		{
			//if (*serialNumber!=trackerObject->theSerialNum)
			if (*serialNumber!=trackerObject->posSerialNum)
			{
				if (position!=NULL)
					*position = trackerObject->thePosition;
			
				if (delta!=NULL)
					*delta = trackerObject->accuMoving;
					
				if (changed!=NULL)		
					*changed=kQ3True;	
			}
			else
				if (changed!=NULL)		
					*changed=kQ3False;
					
			//serial number
			//*serialNumber = trackerObject->theSerialNum;
			*serialNumber = trackerObject->posSerialNum;
		}
		
		//clean up
		trackerObject->accuMoving.x = 0.0;
		trackerObject->accuMoving.y = 0.0;
		trackerObject->accuMoving.z = 0.0;
	}
	else
	{
		if (position!=NULL)
		{
			position->x = 0.0;
			position->y = 0.0;
			position->z = 0.0;
		};
		
		if (delta!=NULL)
		{
			delta->x = 0.0;
			delta->y = 0.0;
			delta->z = 0.0;
		};
		
		if (changed!=NULL)		
			*changed=kQ3False;
	}
	
	return(kQ3Success);
}





//=============================================================================
//      CC3OSXTracker_SetPosition : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_SetPosition(TC3TrackerInstanceDataPtr trackerObject, TQ3ControllerRef controllerRef, const TQ3Point3D *position)
{
	if (trackerObject->isActive==kQ3True)
	{
		TQ3Point3D deltaPosition;
		
		deltaPosition.x = position->x - trackerObject->thePosition.x;
		deltaPosition.y = position->y - trackerObject->thePosition.y;
		deltaPosition.z = position->z - trackerObject->thePosition.z;
		
		/*
		trackerObject->accuMoving.x += deltaPosition.x;
		trackerObject->accuMoving.y += deltaPosition.y;
		trackerObject->accuMoving.z += deltaPosition.z;
		*/
		
		trackerObject->accuMoving.x = .0;
		trackerObject->accuMoving.y = .0;
		trackerObject->accuMoving.z = .0;
		
		trackerObject->thePosition = *position;
		
		//trackerObject->theSerialNum++;	
		trackerObject->posSerialNum++;	
					
		//regard posThreshold!!
		float Threshold = trackerObject->posThreshold;
		if ( (fabs(deltaPosition.x)>=Threshold) || (fabs(deltaPosition.y)>=Threshold) || (fabs(deltaPosition.z)>=Threshold))
			CC3OSXTracker_tryCall_notification_local(controllerRef,trackerObject->trackerUUID);
	}

	return(kQ3Success);
}





//=============================================================================
//      CC3OSXTracker_MovePosition : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_MovePosition(TC3TrackerInstanceDataPtr trackerObject, TQ3ControllerRef controllerRef, const TQ3Vector3D *delta)
{
	if (trackerObject->isActive==kQ3True)
	{
		trackerObject->accuMoving.x += delta->x;
		trackerObject->accuMoving.y += delta->y;
		trackerObject->accuMoving.z += delta->z;
		
		trackerObject->thePosition.x += delta->x;
		trackerObject->thePosition.y += delta->y;
		trackerObject->thePosition.z += delta->z;
		
		//trackerObject->theSerialNum++;	
		trackerObject->posSerialNum++;	
					
		//regard posThreshold!!
		float Threshold = trackerObject->posThreshold;
		if ( (fabsf(delta->x)>=Threshold) || (fabsf(delta->y)>=Threshold) || (fabsf(delta->z)>=Threshold))
			CC3OSXTracker_tryCall_notification_local(controllerRef,trackerObject->trackerUUID);
	}

	return(kQ3Success);
}





//=============================================================================
//      CC3OSXTracker_GetOrientation : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_GetOrientation(TC3TrackerInstanceDataPtr trackerObject, TQ3Quaternion *orientation, TQ3Quaternion *delta, TQ3Boolean *changed, TQ3Uns32 *serialNumber)
{
	if (trackerObject->isActive==kQ3True)
	{
		//serialNumber NULL or not
		if (serialNumber==NULL)
		{
			if (orientation!=NULL)	
				*orientation = trackerObject->theOrientation;
			
			if (delta!=NULL)
				*delta = trackerObject->accuOrientation;
			
			if (changed!=NULL)		
				*changed=kQ3True;
		}
		else
		{
			//if (*serialNumber!=trackerObject->theSerialNum)
			if (*serialNumber!=trackerObject->oriSerialNum)
			{
				if (orientation!=NULL)	
					*orientation = trackerObject->theOrientation;
			
				if (delta!=NULL)
					*delta = trackerObject->accuOrientation;
					
				if (changed!=NULL)		
					*changed=kQ3True;	
			}
			else
				if (changed!=NULL)		
					*changed=kQ3False;
					
			//serial number
			//*serialNumber = trackerObject->theSerialNum;
			*serialNumber = trackerObject->oriSerialNum;
		}
		
		//clean up
		trackerObject->accuOrientation.w = 1.0;
		trackerObject->accuOrientation.x = 0.0;
		trackerObject->accuOrientation.y = 0.0;
		trackerObject->accuOrientation.z = 0.0;
	}
	else
	{
		if (orientation!=NULL)
		{
			orientation->w = 1.0;
			orientation->x = 0.0;
			orientation->y = 0.0;
			orientation->z = 0.0;
		};
		
		if (delta!=NULL)
		{
			delta->w = 1.0;
			delta->x = 0.0;
			delta->y = 0.0;
			delta->z = 0.0;
		}
		
		if (changed!=NULL)		
			*changed=kQ3False;
	}

	return(kQ3Success);
}





//=============================================================================
//      CC3OSXTracker_SetOrientation : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_SetOrientation(TC3TrackerInstanceDataPtr trackerObject, TQ3ControllerRef controllerRef, const TQ3Quaternion *orientation)
{
	if (trackerObject->isActive==kQ3True)
	{
		//Set tracker orientation to new orientation
		trackerObject->theOrientation=*orientation;
		
		//calculate delta angle between new and old orientation
		TQ3Quaternion 	temp_quat;
		float			Angle;
		
		CC3Quaternion_Multiply(orientation,&trackerObject->accuOrientation,&temp_quat);
		CC3Quaternion_GetAngle(&temp_quat,&Angle);				
		
		//Set tracker orientation accumulator to new orientation
		trackerObject->accuOrientation=*orientation;
		
		//trackerObject->theSerialNum++;
		trackerObject->oriSerialNum++;
		
		//regard oriThreshold!!
		float Threshold = trackerObject->oriThreshold;
		if (fabsf(Angle)>=Threshold)
			CC3OSXTracker_tryCall_notification_local(controllerRef,trackerObject->trackerUUID);
	}
	
	return(kQ3Success);
}





//=============================================================================
//      CC3OSXTracker_MoveOrientation : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_MoveOrientation(TC3TrackerInstanceDataPtr trackerObject, TQ3ControllerRef controllerRef, const TQ3Quaternion *delta)
{
	if (trackerObject->isActive==kQ3True)
	{
		//Quaternion trackerObject->accuOrientation multiplied with Quaternion delta
		CC3Quaternion_Multiply(delta,&trackerObject->accuOrientation,&trackerObject->accuOrientation);
		
		//Quaternion trackerObject->theOrientation multiplied with Quaternion delta
		CC3Quaternion_Multiply(delta,&trackerObject->theOrientation,&trackerObject->theOrientation);
		
		//trackerObject->theSerialNum++;
		trackerObject->oriSerialNum++;
	
		//delta to radians!!
		float			Angle;		
		CC3Quaternion_GetAngle(delta,&Angle);
		
		//regard oriThreshold!!
		float Threshold = trackerObject->oriThreshold;
		if (fabsf(Angle)>=Threshold)
			CC3OSXTracker_tryCall_notification_local(controllerRef,trackerObject->trackerUUID);
	}
	
	return(kQ3Success);
}




CFComparisonResult CC3OSXTracker_CmpEventStamp (const void *val1, const void *val2, void *context)
{
	//ignore context;
	
	//typecast val1 from CFDataRef to TC3TrackerEventPtr
	TC3TrackerEventPtr _val1 = (TC3TrackerEventPtr)CFDataGetBytePtr((CFDataRef)val1);
	
	//typecast val2 from CFDataRef to TC3TrackerEventPtr
	TC3TrackerEventPtr _val2 = (TC3TrackerEventPtr)CFDataGetBytePtr((CFDataRef)val2);
	
	//compare val1.timeStamp with val2.timeStamp
	CFComparisonResult _res;
	if (_val1->EventTimeStamp < _val2->EventTimeStamp)
		_res = kCFCompareLessThan;
	
	if (_val1->EventTimeStamp == _val2->EventTimeStamp)
		_res = kCFCompareEqualTo;
		
	if (_val1->EventTimeStamp > _val2->EventTimeStamp)
		_res = kCFCompareGreaterThan;
	
	return _res;
};


//=============================================================================
//      CC3OSXTracker_SetEventCoordinates : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_SetEventCoordinates(TC3TrackerInstanceDataPtr trackerObject, TQ3Uns32 timeStamp, TQ3Uns32 buttons, const TQ3Point3D *position, const TQ3Quaternion *orientation)
{
	//if number of events equals ten, remove oldest at 0
	CFIndex Count = CFArrayGetCount(trackerObject->eventsRingBuffer);
	if (Count==10)
		CFArrayRemoveValueAtIndex(trackerObject->eventsRingBuffer,0);
	//pack event
	TC3TrackerEvent workEvent;
	
	workEvent.EventTimeStamp=timeStamp;
	workEvent.EventButtons=buttons;
	if (position==NULL)
	{
		workEvent.EventPositionIsNULL=kQ3True;
	}
	else
	{
		workEvent.EventPositionIsNULL=kQ3False;
		workEvent.EventPosition=*position;
	}
	
	if (orientation==NULL)
	{
		workEvent.EventOrientationIsNULL=kQ3True;
	}
	else
	{
		workEvent.EventOrientationIsNULL=kQ3False;
		workEvent.EventOrientation=*orientation;
	}
	CFDataRef packedEventRef = CFDataCreate (kCFAllocatorDefault, (UInt8*)&workEvent,sizeof(workEvent));
	//insert event
	CFArrayAppendValue(trackerObject->eventsRingBuffer,packedEventRef);
	//sort array by timestamps (identical timestamps are allowed, sorting results depend on applied sorting algorithm)
	CFRange SortRange = CFRangeMake(0,CFArrayGetCount(trackerObject->eventsRingBuffer));
	CFArraySortValues(trackerObject->eventsRingBuffer,SortRange,CC3OSXTracker_CmpEventStamp,NULL);
	//clean up
	CFRelease(packedEventRef);//just ref'counting; CFArray retained packedEventRef!
	
	return(kQ3Success);
}





//=============================================================================
//      CC3OSXTracker_GetEventCoordinates : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXTracker_GetEventCoordinates(TC3TrackerInstanceDataPtr trackerObject, TQ3Uns32 timeStamp, TQ3Uns32 *buttons, TQ3Point3D *position, TQ3Quaternion *orientation)
{
	TQ3Status status = kQ3Failure;
	CFDataRef 			packedEventRef;
	TC3TrackerEvent		searchEvent;
	TC3TrackerEventPtr 	returnEvent;
	
	if (0!=CFArrayGetCount(trackerObject->eventsRingBuffer))
	{
		//find Event with stamp younger than timeStamp
		//-packetEventRef will be "search" value
		searchEvent.EventTimeStamp = timeStamp;
		packedEventRef = CFDataCreate (kCFAllocatorDefault, (UInt8*)&searchEvent,sizeof(searchEvent));
		
		//-do bsearch
		CFRange SearchRange	= CFRangeMake(0,CFArrayGetCount(trackerObject->eventsRingBuffer));
		CFIndex EventIndex	= CFArrayBSearchValues(trackerObject->eventsRingBuffer,SearchRange,packedEventRef,CC3OSXTracker_CmpEventStamp,NULL);
		CFRelease(packedEventRef);
		
		//Get event from EventIndex and clear buffer
		if (EventIndex<=SearchRange.length)
		{
			//-find Event closest to timestamp
			if (EventIndex>0)
			{
				returnEvent = (TC3TrackerEventPtr)CFDataGetBytePtr((CFDataRef)CFArrayGetValueAtIndex(trackerObject->eventsRingBuffer,EventIndex-1));
				TQ3Uns32 prevStamp = returnEvent->EventTimeStamp;
				
				returnEvent = (TC3TrackerEventPtr)CFDataGetBytePtr((CFDataRef)CFArrayGetValueAtIndex(trackerObject->eventsRingBuffer,EventIndex));
				TQ3Uns32 foundStamp = returnEvent->EventTimeStamp;
				
				if ((timeStamp-prevStamp)<(foundStamp-timeStamp))
					EventIndex--;
			};
			//-extract Event
			packedEventRef = (CFDataRef)CFArrayGetValueAtIndex(trackerObject->eventsRingBuffer,EventIndex);
			CFRetain(packedEventRef);
			//-remove Events older than timeStamp
			CFIndex i;
			for (i=0;i<=EventIndex;i++)
				CFArrayRemoveValueAtIndex(trackerObject->eventsRingBuffer,0);
			//-extract return values
			returnEvent = (TC3TrackerEventPtr)CFDataGetBytePtr(packedEventRef);
			if (buttons!=NULL)
				*buttons=returnEvent->EventButtons;
				
			if ((position!=NULL)&&(returnEvent->EventPositionIsNULL==kQ3False))
				*position=returnEvent->EventPosition;
				
			if ((orientation!=NULL)&&(returnEvent->EventOrientationIsNULL==kQ3False))
				*orientation=returnEvent->EventOrientation;
					
			CFRelease(packedEventRef);
		}
		
		status=kQ3Success;
	}
	
	return(status);
}


//=============================================================================
//      CC3OSXControllerState_New : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
#pragma mark -
TC3ControllerStateInstanceDataPtr
CC3OSXControllerState_New(TQ3Object theObject, TQ3ControllerRef theController)
{
	TC3ControllerStateInstanceDataPtr	theInstanceData = NULL;
	TQ3Status							status = kQ3Failure;
	CFMutableDictionaryRef				dict, returnDict;
	Boolean								result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//theController
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &theController);
					
		//try sending
		status = IPCControllerDriver_Send(m3ControllerState_New,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			theInstanceData = (TC3ControllerStateInstanceDataPtr)malloc(sizeof(TC3ControllerStateInstanceData));
	
			theInstanceData->myController = theController;
			
			//ctrlStateUUID - may be NULL, if key wasn't found in the dictionary				
			theInstanceData->ctrlStateUUIDString = (CFStringRef)CFDictionaryGetValue(returnDict,CFSTR(k3CtrlStateUUID));//bug?
			
			CFRetain(theInstanceData->ctrlStateUUIDString);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
					
	return(theInstanceData);
}



/*
CC3OSXControllerState_Delete:
- is counterpart to CC3OSXControllerState_New
- is exported in CC3OSXControllerCore.h
- needs to be registered inside Quesa to let Q3Object_Dispose(aTQ3ControllerStateObject) work
- registered in E3Initialize.c as kQ3SharedTypeControllerState
*/

//=============================================================================
//      CC3OSXControllerState_Delete : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TC3ControllerStateInstanceDataPtr
CC3OSXControllerState_Delete(TC3ControllerStateInstanceDataPtr ctrlStateObject)
{
	TQ3Status				status = kQ3Failure;
	CFMutableDictionaryRef	dict,returnDict;
	
	Boolean					result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//-myController
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &ctrlStateObject->myController);
		
		//-ctrlStateUUID 
		CFDictionarySetValue(dict,CFSTR(k3CtrlStateUUID),ctrlStateObject->ctrlStateUUIDString);
				
		//try sending
		status = IPCControllerDriver_Send(m3ControllerState_Delete,dict,&returnDict);
		
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	
	//-free things inside instanceData
	//release theInstanceData->ctrlStateUUIDString !?
	
	free (ctrlStateObject);
	
	return(NULL);
}




//=============================================================================
//      CC3OSXControllerState_SaveAndReset : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXControllerState_SaveAndReset(TC3ControllerStateInstanceDataPtr ctrlStateObject)
{
	TQ3Status				status = kQ3Failure;
	CFMutableDictionaryRef	dict, returnDict;
	
	Boolean					result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//-myController
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &ctrlStateObject->myController);
		
		//-ctrlStateUUID 
		CFDictionarySetValue(dict,CFSTR(k3CtrlStateUUID), ctrlStateObject->ctrlStateUUIDString);
				
		//try sending
		status = IPCControllerDriver_Send(m3ControllerState_SaveAndReset, dict, &returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}




//=============================================================================
//      CC3OSXControllerState_Restore : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXControllerState_Restore(TC3ControllerStateInstanceDataPtr ctrlStateObject)
{
	TQ3Status				status = kQ3Failure;
	CFMutableDictionaryRef	dict,returnDict;
	
	Boolean					result;
	
	//create dictionary
	dict = CFDictionaryCreateMutable(	kCFAllocatorDefault,0,
										&kCFTypeDictionaryKeyCallBacks,
										&kCFTypeDictionaryValueCallBacks);
	if (dict)
	{
		//Put parameters into dict
		//-myController
		result = IPCPutControllerRef(dict, CFSTR(k3CtrlRef), &ctrlStateObject->myController);
		
		//-ctrlStateUUID 
		CFDictionarySetValue(dict,CFSTR(k3CtrlStateUUID),ctrlStateObject->ctrlStateUUIDString);
				
		//try sending
		status = IPCControllerDriver_Send(m3ControllerState_Restore,dict,&returnDict);
		if (status!=kQ3Failure)
		{
			//Get status from returnDict
			result = IPCGetTQ3Uns32(returnDict, CFSTR(k3Status), (TQ3Uns32*)&status);
		};
		//Do clean up
		CFRelease(dict);
		
		if (returnDict)
			CFRelease(returnDict);
	}
	return(status);
}




//=============================================================================
//      CC3OSXCursorTracker_PrepareTracking : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
#pragma mark -
TQ3Status
CC3OSXCursorTracker_PrepareTracking(void)
{


	// To be implemented...
	return(kQ3Failure);
}




//=============================================================================
//      CC3OSXCursorTracker_SetTrackDeltas : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXCursorTracker_SetTrackDeltas(TQ3Boolean trackDeltas)
{


	// To be implemented...
	return(kQ3Failure);
}





//=============================================================================
//      CC3OSXCursorTracker_GetAndClearDeltas : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXCursorTracker_GetAndClearDeltas(float *depth, TQ3Quaternion *orientation, TQ3Boolean *hasOrientation, TQ3Boolean *changed, TQ3Uns32 *serialNumber)
{


	// To be implemented...
	return(kQ3Failure);
}





//=============================================================================
//      CC3OSXCursorTracker_SetNotifyFunc : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXCursorTracker_SetNotifyFunc(TQ3CursorTrackerNotifyFunc notifyFunc)
{


	// To be implemented...
	return(kQ3Failure);
}





//=============================================================================
//      CC3OSXCursorTracker_GetNotifyFunc : One-line description of the method.
//-----------------------------------------------------------------------------
//		Note : More detailed comments can be placed here if required.
//-----------------------------------------------------------------------------
TQ3Status
CC3OSXCursorTracker_GetNotifyFunc(TQ3CursorTrackerNotifyFunc *notifyFunc)
{


	// To be implemented...
	return(kQ3Failure);
}




