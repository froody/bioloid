//
//    MyDocument.M
//
//    Copyright (C) 2010  Christian Balkenius
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    Created: April 1, 2010
//
//    The .M suffix in the file name is necessary to call C++ from objective-C
//


#import "MyDocument.h"

#include "DynamixelComm.h"
#import "Page.h"
#import "Pose.h"

#define NUM_FIELDS 11
NSString *pageFieldNames[NUM_FIELDS] = {
	@"playCode",
	@"pageSpeed",
	@"dxlSetup",
	@"accelTime",
	@"nextPage",
	@"exitPage",
	@"linkedPage1",
	@"linkedPage1PlayCode",
	@"linkedPage2",
	@"linkedPage2PlayCode",
	@"checkSum"
};


@implementation MyDocument

#define BUF_SIZE 128*1024

-(void)timerFire:(NSTimer*)theTimer
{
    if(dc && [idList selectedRow] != -1) {
		[dcLock lock];
		dc->ReadAllData([[idNumber objectAtIndex: [idList selectedRow]] intValue], servoData);
		[dcLock unlock];
	}
	[histogramView setNeedsDisplay:YES];
    [self update: self];
}

HistogramView *gHisogramView;

- (id)init
{
    self = [super init];
    if (self)
    {
		NSBundle *bundle = [NSBundle mainBundle];
		NSString *path = [bundle pathForResource: @"motion" ofType: @"bin"];   // The AX12 file is currently used for all servo types (they all seem to be the same)
		NSLog(@"path is %@", path);
		int fd = [[NSFileHandle fileHandleForReadingAtPath:path] fileDescriptor];

		if(fd == -1) {
			NSLog(@"Couldn't open motion.bin");
			exit(1);
		}

		pages = (struct page *)malloc(BUF_SIZE);

		pagesArray = [NSMutableArray arrayWithCapacity:BUF_SIZE/sizeof(struct page)];
		[pagesArray retain];

		if(BUF_SIZE != read(fd, pages, BUF_SIZE)) {
			NSLog(@"Couldn't read BUF_SIZE from file");
			exit(1);
		}

		int i;
		for(i=0;i<BUF_SIZE/sizeof(struct page);i++)
		{
			if(pages[i].header.numPoses)
				[pagesArray insertObject:[[Page alloc] initWithPage:&pages[i] index:i] atIndex:i];
		}

		initRobotData(robotData);
		initInterpolationData(ipoData);
		robotData.writeTimeDiffAverage = ipoData.ipoTotalTime;
		initSCurveParameters(sCurveParams);
		dcLock = [[NSLock alloc] init];

		//gHisogramView = histogramView;
	}

	//NSLog(@"%@", pagesArray);
    return self;
}



- (NSString *)windowNibName
{
    return @"MyDocument";
}



- (NSString *)displayName
{
    return @"Dynamixel Monitor";
}


- (void)windowControllerDidLoadNib:(NSWindowController *) aController
{
    [super windowControllerDidLoadNib:aController];

    NSBundle *bundle = [NSBundle mainBundle];
    NSString *path = [bundle pathForResource: @"AX12" ofType: @"plist"];   // The AX12 file is currently used for all servo types (they all seem to be the same)
        
    controlTable = [[NSArray arrayWithContentsOfFile: path] retain];
    
    ID = [[NSMutableArray arrayWithCapacity: 128] retain];
    idNumber = [[NSMutableArray arrayWithCapacity: 128] retain];


    dc = NULL;
    newID = NO;

    timer = [NSTimer scheduledTimerWithTimeInterval:0.1
        target:self
        selector:@selector(timerFire:)
        userInfo:nil
        repeats:YES];

	gHisogramView = histogramView;

	[self connect:self];
}


/*

- (NSData *)dataOfType:(NSString *)typeName error:(NSError **)outError
{
    // Insert code here to write your document to data of the specified type. If the given outError != NULL, ensure that you set *outError when returning nil.

    // You can also choose to override -fileWrapperOfType:error:, -writeToURL:ofType:error:, or -writeToURL:ofType:forSaveOperation:originalContentsURL:error: instead.

    // For applications targeted for Panther or earlier systems, you should use the deprecated API -dataRepresentationOfType:. In this case you can also choose to override -fileWrapperRepresentationOfType: or -writeToFile:ofType: instead.

    if ( outError != NULL ) {
		*outError = [NSError errorWithDomain:NSOSStatusErrorDomain code:unimpErr userInfo:NULL];
	}
	return nil;
}



- (BOOL)readFromData:(NSData *)data ofType:(NSString *)typeName error:(NSError **)outError
{
    // Insert code here to read your document from the given data of the specified type.  If the given outError != NULL, ensure that you set *outError when returning NO.

    // You can also choose to override -readFromFileWrapper:ofType:error: or -readFromURL:ofType:error: instead. 
    
    // For applications targeted for Panther or earlier systems, you should use the deprecated API -loadDataRepresentation:ofType. In this case you can also choose to override -readFromFile:ofType: or -loadFileWrapperRepresentation:ofType: instead.
    
    if ( outError != NULL ) {
		*outError = [NSError errorWithDomain:NSOSStatusErrorDomain code:unimpErr userInfo:NULL];
	}
    return YES;
}

*/


- (int)numberOfRowsInTableView:(NSTableView *)tableView
{
    if([tableView tag] == 0)
    {
//        NSLog(@"ID count = %d", [ID count]);
        return [ID count];    
    }
    else
    {
        if([idList selectedRow] == -1)
            return 0;
        else
            return [controlTable count];
    }
}



// Function that translates the values in the control table to a more readable form
//
// This could probably be done more elegantly by adding conversion information and units to the 
// property list file for each servo type

-(NSString *)translateValue:(unsigned int)value atAddress: (unsigned int)address
{
    switch(address)
    {
        case 0: // Model Number
            switch(value)
            {
                case 10:   return @"RX-19";
                case 64:   return @"RX-64";
                case 12:   return @"AX-12"; // no difference between AX-12 and AX-12+ ???
                case 113:  return @"DX-113";
                case 116:  return @"DX-116";
                case 117:  return @"DX-117";
                case 106:  return @"EX-106"; // this is just a guess
                default:   return @"Unkown dynamixel";
            }
            
            return (value == 12 ? @"AX-12" : @"Unkown dynamixel");
    
        case 4: // Buad Rate
            if(value == 1)
                return @"1 Mbps";
            else if(value <10)
                return [[NSString alloc] initWithFormat:@"%d kbps", 2000/(1+value)];
            else
                return [[NSString alloc] initWithFormat:@"%d bps", 2000000/(1+value)];

        case 5: // Return Delay Time
            return [[NSString alloc] initWithFormat:@"%d μs", 2*value];
        
        case 16: // Status Return Level
            switch(value)
            {
                case 0:  return @"Never";
                case 1:  return @"READ_DATA only";
                case 2:  return @"Always";
                default: return @"Illegal value";
            }
            
        case 24: // ON/OFF
        case 25:
            return (value == 0 ? @"Off" : @"On");

        case 17: // Status Return Level
        case 18: // Alarm Shutdown
            return [[NSString alloc] initWithFormat:@"%d", value]; // Check the bits here => IOCRHAV
        
        case 11: // Temperatures
        case 43:
            return [[NSString alloc] initWithFormat:@"%d °C", value];
            
        case 12: // Voltages
        case 13:
        case 42:
            return [[NSString alloc] initWithFormat:@"%.1f V", float(value)/10];
        
        case 38: // Speed/Load
        case 40:
            if(value == 0)
                return @"0";
            else if(value < 1024)
                return [[NSString alloc] initWithFormat:@"+%d", value];
            else
                return [[NSString alloc] initWithFormat:@"-%d", value-1024];

        case 6: // Angles
        case 8:
        case 30:
        case 36:
        case 26: // We assume that the correct unit for the compliance parameters is in degrees
        case 27:
        case 28:
        case 29:
            return [[NSString alloc] initWithFormat:@"%.1f°", 300.0f*float(value)/float(0x3ff)];

        case 32: //Speed
            if(value > 0)
                return [[NSString alloc] initWithFormat:@"%.1f RPM", 114.0f*float(value)/float(0x3ff)];
            else
                return @"Max RPM";

        case 48: // Punch
            return [[NSString alloc] initWithFormat:@"%.1f%%", 100.0f*float(value)/float(0x3ff)];

        default:
            return [[NSString alloc] initWithFormat:@"%d", value];
    }
}




- (id)tableView:(NSTableView *)tableView
      objectValueForTableColumn:(NSTableColumn *)tableColumn
      row:(int)row
{
    if([tableView tag] == 0)
    {
        return [ID objectAtIndex: row];
    }
    
    else
    {
        if([[tableColumn identifier] isEqualToString: @"Item"])
        {
            return [[controlTable objectAtIndex: row] objectForKey: [tableColumn identifier]];
        
        }
        else if([[tableColumn identifier] isEqualToString: @"Value"])
        {
            int address = [[[controlTable objectAtIndex: row] objectForKey: @"Address"] intValue];
            int size = [[[controlTable objectAtIndex: row] objectForKey: @"Size"] intValue];

            if(size == 1)
                return [self translateValue: servoData[address] atAddress: address];
            else
                return [self translateValue: servoData[address]+256*servoData[address+1] atAddress: address];
        }
        else // Raw
        {
            int address = [[[controlTable objectAtIndex: row] objectForKey: @"Address"] intValue];
            int size = [[[controlTable objectAtIndex: row] objectForKey: @"Size"] intValue];
            if(size == 1)
                return [[NSString alloc] initWithFormat:@"%d", servoData[address]];
            else
                return [[NSString alloc] initWithFormat:@"%d", servoData[address]+256*servoData[address+1]];
        }
    }
}



- (IBAction)connect:(id)sender
{       
    NSUserDefaults *defaults = [NSUserDefaults standardUserDefaults];
    NSString *device = [defaults stringForKey:@"device"];
    if(device == nil) device = @"/dev/cu.usbserial-A7005Lxn";
    
    if(!dc)
    {    
        try
        {
            dc = new DynamixelComm([device UTF8String], 1000000);
        }
        catch(...)
        {
            NSAlert *alert = [[NSAlert alloc] init];
            [alert addButtonWithTitle:@"OK"];
            [alert setMessageText:@"Could not find USB2Dynamixel"];
            [alert setInformativeText:@"Check that the cable is connected and that the device name is set correctly in the preferences."];
            [alert setAlertStyle:NSWarningAlertStyle];

            if ([alert runModal] == NSAlertFirstButtonReturn) {
                // OK clicked, delete the record
            }
            [alert release];
            return;
        }
        
        // Scan for dynamixels
        
        [ID removeAllObjects];
        
        for(int i=0; i<32; i++)
        {
			[dcLock lock];
            if(dc->Ping(i))
            {
            //  NSLog(@"ID = %d found", i);
                [ID addObject: [[NSString alloc] initWithFormat:@"ID-%d", i]];
                [idNumber addObject: [NSNumber numberWithInt: i]];
            }
			[dcLock unlock];
        }
        [idList reloadData];
        [connectButton setTitle: @"Disconnect"];
        [torqueEnableButton setEnabled: YES];   // TODO: check first that we have connected
        [torqueDisableButton setEnabled: YES];
    }
    else
    {
//        NSLog(@"Disconnect");
        delete dc;
        dc = NULL;
        [ID removeAllObjects];
        [idList reloadData];
        [connectButton setTitle: @"Connect"];

        [torqueEnableButton setEnabled: NO];
        [torqueDisableButton setEnabled: NO];
    }
}



- (void)forceUpdate
{
	/*
    if(dc && [idList selectedRow] != -1)
        dc->ReadAllData([[idNumber objectAtIndex: [idList selectedRow]] intValue], servoData);
	 */

    [servoValues reloadData];
    [servoValues display];      // Force redraw
    [motionData reloadData];
    [motionData display];      // Force redraw

    // Update controls
    
    if([idList selectedRow] == -1)
        return;

    [positionIndicator setIntValue: servoData[P_PRESENT_POSITION_L]+256*servoData[P_PRESENT_POSITION_H]];
    
    unsigned int speed = servoData[P_PRESENT_SPEED_L]+256*servoData[P_PRESENT_SPEED_H];
    if(speed < 1024)
        [speedIndicator setIntValue: speed];
    else
        [speedIndicator setIntValue: speed-1024];
        
    int load = servoData[P_PRESENT_LOAD_L]+256*servoData[P_PRESENT_LOAD_H];
    if(load >= 1024) load -= 1024;
    [loadIndicator setIntValue: load];

    [voltageIndicator setIntValue: servoData[P_PRESENT_VOLTAGE]];
    [temperatureIndicator setIntValue: servoData[P_PRESENT_TEMPERATURE]];

    // Move the goal position with the servo if torque is disabled

    if(servoData[P_TORQUE_ENABLE] == 0 && dc) //  && [goalTracking  intValue] 
        [goalPositionSlider setIntValue: servoData[P_PRESENT_POSITION_L]+256*servoData[P_PRESENT_POSITION_H]];

    [torqueEnable setIntValue: servoData[P_TORQUE_ENABLE]];

}



-(void)enableControls:(BOOL)enable
{
    [positionIndicator setEnabled: enable];
    [speedIndicator setEnabled: enable];
    [loadIndicator setEnabled: enable];
    [voltageIndicator setEnabled: enable];
    [temperatureIndicator setEnabled: enable];

    [goalPositionSlider setEnabled: enable];
    [movingSpeedSlider setEnabled: enable];

    [torqueEnable setEnabled: enable];

    [torqueEnableButton setEnabled: enable];
    [torqueDisableButton setEnabled: enable];
    
    if(!enable)
    {
        [positionIndicator setIntValue: 0];
        [speedIndicator setIntValue: 0];
        [loadIndicator  setIntValue: 0];
        [voltageIndicator setIntValue: 0];
        [temperatureIndicator setIntValue: 0];

        [goalPositionSlider setIntValue: 0];

        [torqueEnable setIntValue: 0];
    }
}



- (IBAction)update:(id)sender
{
    [servoValues reloadData];
    
    if([idList selectedRow] == -1)
    {
        [self enableControls: NO];
        return;
    }
    
    [self enableControls: YES];

    [positionIndicator setIntValue: servoData[P_PRESENT_POSITION_L]+256*servoData[P_PRESENT_POSITION_H]];
    
    unsigned int speed = servoData[P_PRESENT_SPEED_L]+256*servoData[P_PRESENT_SPEED_H];
    if(speed < 1024)
        [speedIndicator setIntValue: speed];
    else
        [speedIndicator setIntValue: speed-1024];
        
    int load = servoData[P_PRESENT_LOAD_L]+256*servoData[P_PRESENT_LOAD_H];
    if(load >= 1024) load -= 1024;
    [loadIndicator setIntValue: load];
    
    [voltageIndicator setIntValue: servoData[P_PRESENT_VOLTAGE]];
    [temperatureIndicator setIntValue: servoData[P_PRESENT_TEMPERATURE]];

    // Move the goal position with the servo if torque is disabled
    // or new ID is selected

    if((servoData[P_TORQUE_ENABLE] == 0 || newID) && dc)
        [goalPositionSlider setIntValue: servoData[P_PRESENT_POSITION_L]+256*servoData[P_PRESENT_POSITION_H]];

    [torqueEnable setIntValue: servoData[P_TORQUE_ENABLE]];
    
    newID = NO;
}



- (IBAction)setGoalPosition:(id)sender
{
    int p = [goalPositionSlider intValue];
    int s = [movingSpeedSlider intValue];
    if(s == 1024) s = 0; // set maximum speed (no speed control) for maximal value

	// Keep the display lively
	[[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode beforeDate:[NSDate distantFuture]];

	[dcLock lock];
    dc->Move([[idNumber objectAtIndex: [idList selectedRow]] intValue], p, s);
	[dcLock unlock];
}



- (IBAction)setSpeed:(id)sender
{
    int s = [movingSpeedSlider intValue];
    if(s == 1024) s = 0; // set maximum speed (no speed control) for maximum value

	// Keep the display lively
	[[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode beforeDate:[NSDate distantFuture]];

	[dcLock lock];
    dc->SetSpeed([[idNumber objectAtIndex: [idList selectedRow]] intValue], s);
	[dcLock unlock];
    
    [self forceUpdate];
}

- (IBAction)setIpoPause:(id)sender
{
	NSLog(@"Old ipo stuff %d %d", ipoData.ipoPauseAutoSet, ipoData.ipoPause);
	ipoData.ipoPauseAutoSet = 0;
	ipoData.ipoPause = [ipoPauseSlider intValue];
	NSLog(@"New ipo stuff %d %d", ipoData.ipoPauseAutoSet, ipoData.ipoPause);
}




- (IBAction)toggleTorque:(id)sender
{
	[dcLock lock];
    dc->SetTorque([[idNumber objectAtIndex: [idList selectedRow]] intValue], [sender intValue]);
	[dcLock unlock];
}



- (IBAction)torqueAllEnable:(id)sender
{
	[dcLock lock];
    dc->SetTorque(254, 1);
	[dcLock unlock];
}



- (IBAction)torqueAllDisable:(id)sender
{
	[dcLock lock];
    dc->SetTorque(254, 0);
	[dcLock unlock];
}

- (IBAction)stop:(id)sender
{
	stop = YES;
}

- (IBAction)playPose:(id)sender
{
	int row = [motionData selectedRow];
	Pose *thePose;
	Page *thePage;

	if(row == -1)
		return;
	id item = [motionData itemAtRow:[motionData selectedRow]];
	
	if([item class] == [Page class]) {
		thePage = item;
		NSLog(@"playing page %d, pose %d", thePage->index, 0);
		[self reallyPlayPose:&thePage->thePage->rec[0]];
	}
	else if([item class] == [Pose class])
	{
		thePose = item;
		int poseIndex = thePose->index;

		while(--row >= 0) {
			thePage = [motionData itemAtRow:row];
			if([thePage class] == [Page class]) {
				NSLog(@"playing page %d, pose %d", thePage->index, poseIndex);
				[self reallyPlayPose:thePose->thePose];
				break;
			}
		}
	} else if([[item class] isSubclassOfClass:[NSDictionary class]]) {
		while(--row >= 0) {
			thePage = [motionData itemAtRow:row];
			if([thePage class] == [Page class]) {
				NSLog(@"playing page %d, pose %d", thePage->index, 0);
				[self reallyPlayPose:&thePage->thePage->rec[0]];
				break;
			}
		}

	} else {
		Pose *thePose;
		Page *thePage;
		id elem;
		while(--row >= 0) {
			elem = [motionData itemAtRow:row];
			if([elem class] == [Pose class])
			thePose = elem;
			else if([elem class] == [Page class])
			{
				thePage = elem;
				break;
			}
		}
		NSLog(@"playing page %d, pose %d", thePage->index, thePose->index);
		[self reallyPlayPose:thePose->thePose];
	}
}

- (IBAction)playPage:(id)sender
{
	int row = [motionData selectedRow];

	if(row == -1)
		return;
	id item = [motionData itemAtRow:[motionData selectedRow]];
	Page *thePage;

	if([item class] == [Page class]) {
		thePage = item;
	} else {
		while(--row >= 0) {
			thePage = [motionData itemAtRow:row];
			if([thePage class] == [Page class]) {
				break;
			}
		}
	}

	[NSThread detachNewThreadSelector:@selector(reallyPlayPage:) toTarget:self withObject:thePage];
}

- (void) reallyPlayPage:(Page *)thePage
{
	NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];

	struct page *page = thePage->thePage;
	int i;
	int nextPage;
	stop = NO;
	while(1) {
		for(i=0;i< page->header.numPoses; i++)
		{
			#if crap
			[self reallyPlayPose:&page->rec[i]];
			usleep(500000);
			#else
			{
				[dcLock lock];
				struct pose *pose = &page->rec[i];
				int j;
				NSMutableString *s = [NSMutableString stringWithCapacity:100];
				for(j=0;j<[ID count];j++) {
					int servoID = [[idNumber objectAtIndex:j] intValue];
					robotData.writeBuffer[0][j*6] = pose->posData[servoID] & 0xff;
					robotData.writeBuffer[0][j*6+1] = (pose->posData[servoID] >> 8) & 0xff;
					robotData.writeBuffer[0][j*6+2] = 255;
					robotData.writeBuffer[0][j*6+3] = 0;
					robotData.writeBuffer[0][j*6+4] = 255;
					robotData.writeBuffer[0][j*6+5] = 0;

					//NSLog(@"setting servo %d to %d\n", servoID, pose->posData[servoID]);
					//dc->Move([[idNumber objectAtIndex: j] intValue], pose->posData[servoID], 100);
				}

				[s appendFormat:@"setting joints "];
				for(int i = 0;i<18 /*[idNumber count]*/;i++) {
					[s appendFormat:@"%04x ", pose->posData[i+1]];
				}
				NSLog(@"%@",s);

				robotData.writeBufferLength = 1;
				robotData.writeBufferIndex = 0;
				while(robotData.writeBufferLength) {
					//NSLog(@"doing interpolation!");

					doInterpolation(dc, robotData, ipoData, sCurveParams);
				}
				NSLog(@"done a pose!");
				[dcLock unlock];
			}
			#endif
		}

		NSLog(@"stop = %d\n", stop);
		if(stop) {
			nextPage = page->header.exitPage;
			NSLog(@"selecting exitPage %d\n", nextPage);
		}
		else 
			nextPage = page->header.nextPage;

		if(nextPage != 0) {
			Page *thePage = [pagesArray objectAtIndex:nextPage];
			page = thePage->thePage;
			continue;
		} else {
			break;
		}
	}

	[pool release];
}

- (void) reallyPlayPose:(struct pose *)pose
{
	//NSLog(@"Really playing a pose omg w00t! %d servos", [ID count]);
	//[[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode beforeDate:[NSDate distantFuture]];
#if crap
	int i;
	for(i=0;i<[ID count];i++) {
		int servoID = [[idNumber objectAtIndex:i] intValue];
		//NSLog(@"setting servo %d to %d\n", servoID, pose->posData[servoID]);
		[dcLock lock];
		dc->Move([[idNumber objectAtIndex: i] intValue], pose->posData[servoID], 100);
		[dcLock unlock];
	}
#else
	{
		[dcLock lock];
		int j;
		NSMutableString *s = [NSMutableString stringWithCapacity:100];
		for(j=0;j<[ID count];j++) {
			int servoID = [[idNumber objectAtIndex:j] intValue];
			robotData.writeBuffer[0][j*6] = pose->posData[servoID] & 0xff;
			robotData.writeBuffer[0][j*6+1] = (pose->posData[servoID] >> 8) & 0xff;
			robotData.writeBuffer[0][j*6+2] = 255;
			robotData.writeBuffer[0][j*6+3] = 0;
			robotData.writeBuffer[0][j*6+4] = 255;
			robotData.writeBuffer[0][j*6+5] = 3;

			//NSLog(@"setting servo %d to %d\n", servoID, pose->posData[servoID]);
			//dc->Move([[idNumber objectAtIndex: j] intValue], pose->posData[servoID], 100);
		}

		[s appendFormat:@"setting joints "];
		for(int i = 0;i<18 /*[idNumber count]*/;i++) {
			[s appendFormat:@"%04x ", pose->posData[i+1]];
		}
		NSLog(@"%@",s);

		robotData.writeBufferLength = 1;
		robotData.writeBufferIndex = 0;
		while(robotData.writeBufferLength) {
			//NSLog(@"doing interpolation!");

			doInterpolation(dc, robotData, ipoData, sCurveParams);
		}
		NSLog(@"done a pose!");
		[dcLock unlock];
	}
#endif
}






- (void) tableViewSelectionDidChange: (NSNotification *) notification
{
    int row;

    if([notification object] == idList)
    {
        row = [idList selectedRow];

        if (row == -1) {
            //
        }
        else
        {
            newID = YES;
        }
    }
}


- (id)outlineView:(NSOutlineView *)outlineView child:(NSInteger)index ofItem:(id)item
{
	if(item == nil) {

		if(index > [pagesArray count]) {
			NSLog(@"returning nil");
			return nil;
		}

		return [pagesArray objectAtIndex:index];
	} else if([item class] == [Page class]) {
		Page *thePage = item;
		if(index < NUM_FIELDS) {
			uint8_t value = (&thePage->thePage->header.playCode)[index];
			NSDictionary *d = [NSDictionary dictionaryWithObject:[NSNumber numberWithInt:value] forKey:pageFieldNames[index]];
				[d retain];
			return d;
		} else 
			return [(Page *)item getPose:index-NUM_FIELDS];
	} else if([item class] == [Pose class]) {
		return nil; //[(Pose *)item getPosition:index+1];
	} else {
		NSLog(@"child returning nil");
		return nil;
	}
}

- (NSInteger)outlineView:(NSOutlineView *)outlineView numberOfChildrenOfItem:(id)item
{
	if(item == nil) {
		return [pagesArray count];
	} else if([item class] == [Page class]) {
		NSMutableArray *poses = ((Page *)item)->poses;

		int ret = NUM_FIELDS + [poses count];

		return ret;
	} else if([item class] == [Pose class]) {
		return 0; //18; // NumServos
	} else {
		return 0;
	}
}

- (BOOL)outlineView:(NSOutlineView *)outlineView isItemExpandable:(id)item
{
	if(item == nil) {
		return YES;
	} else if([item class] == [Page class]) {
		return YES;
	} else if([item class] == [Pose class]) {
		return NO;
	} 
	
	return NO;
}

- (id)outlineView:(NSOutlineView *)outlineView objectValueForTableColumn:(NSTableColumn *)tableColumn byItem:(id)item
{
	//NSLog(@"request for value for column %@", tableColumn);
	bool field = false;
	
	if([[tableColumn identifier] isEqualToString:@"Field"])
		field = true;
	
	if(item == nil) {
		NSLog(@"objectValueForTableColumn for nil item?");
		return nil;
	} else if([item class] == [Page class]) {
		if(field) {
			Page *thePage = item;
			char s[15];
			s[14] = 0;
			bcopy(thePage->thePage->header.name, s, 14);
			return [NSString stringWithFormat:@"Page %d: %s", thePage->index, s];
		} else {
			return nil;
		}

	} else if([item class] == [Pose class]) {
		Pose *thePose = item;
		if(field)
			return [NSString stringWithFormat:@"Pose %d", thePose->index];
		else {
			NSMutableString *s = [NSMutableString stringWithCapacity:100];

			[s appendFormat:@"S: %03x D: %03x ", thePose->thePose->speed, thePose->thePose->delay];

			for(int i = 0;i<18 /*[idNumber count]*/;i++) {
				[s appendFormat:@"%04x ", thePose->thePose->posData[i+1]];
			}

			return s;
		}
	} else if([[item class] isSubclassOfClass:[NSDictionary class]]) {
		NSDictionary *theDict = item;
		if(field)
			return [[theDict allKeys] objectAtIndex:0];
		else
			return [[theDict allValues] objectAtIndex:0];

	} else {
		NSLog(@"wtf mate?");
		return [item description];
	}
}

void push_data(int servo, int val);
void
push_data(int servo, int val)
{
	[gHisogramView addPoint:val forGraph:servo];
}

@end
