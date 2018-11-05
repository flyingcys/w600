//
//  OneShotConfig.h
//  OneShotConfig

//  Created by codebat on 15/1/22.
//  Copyright (c) 2015年 Winnermicro. All rights reserved.
//
//

#import <Foundation/Foundation.h>


@interface OneShotConfig : NSObject
/*返回：
 0,表示发送正常结束，如果没有配置成功，需要继续调用该方法
 -1,表示由于调用stop或是stopConfig接口，中断该方法
 -2,表示内部错误
 */
-(int)startConfig: (NSString*) ssid pwd: (NSString*) password;
/*中断startConfig的配网操作
 */
-(void)stopConfig;

-(void) start: (NSString*)ssid key:(NSString*)key  timeout:(int) timeout;

-(void) stop;

@end
