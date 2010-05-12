//
//  Page.h
//  DynamixelMonitor
//
//  Created by local on 5/9/10.
//  Copyright 2010 LUCS. All rights reserved.
//

#import <Foundation/Foundation.h>
#import "Pose.h"

@interface Page : NSObject {
	@public struct page *thePage;
	@public NSMutableArray *poses;
	@public int index;
}
- (id) initWithPage:(struct page *)page index:(int)theIndex;
- (Pose *) getPose:(int) index;
@end
