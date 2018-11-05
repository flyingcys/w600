//
//  ManualConfig.h
//  OneShotConfig
//
//  Created by codebat on 15/1/22.
//  Copyright (c) 2015年 Winnermicro. All rights reserved.
//

#import <Foundation/Foundation.h>
@interface ManualConfig : NSObject
/*设置可用来响应连接请求的WIFI网络名称的前缀，
 如以softap开头，则[ManualConfig setValidSSIDPrefix:@"softap"]
 */
+(void)setValidSSIDPrefix:(NSString*)name;
+ (instancetype)getInstance;

/*
 返回
 -1，表示没有连接到wifi
 -2，表示当前连接网络的SSID不对，应修改为合适前缀的SSID
 -3，表示连接TCP Server超时
 -4，表示发送数据超时
 0，表示发送成功
 发送完后会关闭tcp socket
 */

-(int)startConfig:(NSString*)ssid pwd:(NSString*)password;
@end
