//
//  HistogramView.h
//  DynamixelMonitor
//
//  Created by local on 11/8/10.
//  Copyright 2010 LUCS. All rights reserved.
//

#import <Cocoa/Cocoa.h>


@interface HistogramView : NSView {
	NSArray *points;
	NSLock *lock;
}

- (void)addPoint:(int)val forGraph:(int)index;

@end
