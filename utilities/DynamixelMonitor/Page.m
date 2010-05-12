//
//  Page.m
//  DynamixelMonitor
//
//  Created by local on 5/9/10.
//  Copyright 2010 LUCS. All rights reserved.
//

#import "Page.h"


@implementation Page

- (id) initWithPage:(struct page *)page index:(int)theIndex
{
//	NSLog(@"initWithPage called %p %p\n", self, page);
	thePage = page;
	index = theIndex;
	
	poses = [NSMutableArray arrayWithCapacity:thePage->header.numPoses];
	[poses retain];

	//NSLog(@"poses is %p, %@", poses, poses);
	
	int i;
	for(i=0;i<thePage->header.numPoses;i++)
	{
		[poses addObject:[[Pose alloc] initWithPose:&thePage->rec[i] index:i]];
	}
	
	return self;
}

- (Pose *) getPose:(int)theIndex
{
	if( theIndex > [poses count]) {
		return nil;
	} else {
		return [poses objectAtIndex:theIndex];
	}
}

- (NSString *)description
{
	return [NSString stringWithFormat:@"Page %p - %p", thePage, poses];
}

@end
