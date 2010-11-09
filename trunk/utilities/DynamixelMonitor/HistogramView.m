//
//  HistogramView.m
//  DynamixelMonitor
//
//  Created by local on 11/8/10.
//  Copyright 2010 LUCS. All rights reserved.
//

#import "HistogramView.h"

#define NUM_BLAH 18

@implementation HistogramView

- (id)initWithFrame:(NSRect)frame {
    self = [super initWithFrame:frame];
    if (self) {
		NSLog(@"Initing!");
		NSMutableArray *tmpArray = [NSMutableArray arrayWithCapacity:0];

		int i;
		for(i=0;i<NUM_BLAH;i++) {
			[tmpArray addObject:[NSMutableArray arrayWithCapacity:0]];
		}

		points = [NSArray arrayWithArray:tmpArray];
		NSLog(@"Inited!");
		[points retain];
		lock = [[NSLock alloc] init];
    }
    return self;
}

- (void)addPoint:(int)val forGraph:(int)index
{
	[lock lock];
	NSMutableArray *localPoints = [points objectAtIndex:index];
	[localPoints insertObject:[NSNumber numberWithInt: val] atIndex:0];
	[lock unlock];
	[self setNeedsDisplay:YES];
}

- (void)drawRect:(NSRect)dirtyRect
{
	NSRect bounds = [self bounds];
	[[NSColor blackColor] set];
	NSRectFill(bounds);

	[[NSColor whiteColor] set];

	NSBezierPath *line = [NSBezierPath bezierPath];

//	NSLog(@"bounds %f %f\n", bounds.size.width, bounds.size.height);


	double w = bounds.size.width;
	double h = bounds.size.height;

	int i, j;
	[line setLineWidth:1.0];
	for(i=0;i<NUM_BLAH;i++) {

		[line appendBezierPathWithRect:NSMakeRect(0, i*(h/NUM_BLAH), w, h/NUM_BLAH)];
		[line stroke];

		NSMutableArray *localPoints = [points objectAtIndex:i];

		j=0;
		[lock lock];
		for( NSNumber *num in localPoints) {
			if(w-2*j < 0)
				break;
			if(j==0) {
				[line moveToPoint:NSMakePoint(w-2*j, i*(h/NUM_BLAH) + (h/(NUM_BLAH))*[num floatValue]/1024)];
			} else {
				[line lineToPoint:NSMakePoint(w-2*j, i*(h/NUM_BLAH) + (h/(NUM_BLAH))*[num floatValue]/1024)];
			}
			//[line appendBezierPathWithRect:NSMakeRect(w-2*j, i*(h/NUM_BLAH) + (h/(NUM_BLAH))*[num floatValue]/1024, 1, 1)];
			//[line appendBezierPathWithRect:NSMakeRect(w, h + (random() %30) - 15, 1, 1)];
			//[line lineToPoint:NSMakePoint(bounds.size.width,bounds.size.height)];
			j++;

		}
		[lock unlock];
		[line stroke];
	}
}

@end
