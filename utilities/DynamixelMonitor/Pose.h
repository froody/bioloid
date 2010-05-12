//
//  Pose.h
//  DynamixelMonitor
//
//  Created by local on 5/9/10.
//  Copyright 2010 LUCS. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "Motion.h"

@interface Pose : NSObject {
	@public struct pose *thePose;
	@public int index;
}

- (id)initWithPose:(struct pose *)pose index:(int)theIndex;
- (NSNumber *)getPosition:(int)index;
@end
