//
//  Pose.m
//  DynamixelMonitor
//
//  Created by local on 5/9/10.
//  Copyright 2010 LUCS. All rights reserved.
//

#import "Pose.h"


@implementation Pose
- (id)initWithPose:(struct pose *)pose index:(int)theIndex
{
	thePose = pose;
	index = theIndex;

	return self;
}

- (NSNumber *)getPosition:(int)i
{
	if(i >= 31)
		return nil;
	else {
		return [[NSNumber numberWithInt:thePose->posData[i]] retain];
	}
}
@end
