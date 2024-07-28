/**
 ****************************************************************************************
 *
 * @file app_keypad.h
 *
 * @brief APP KEY Module entry point
 *
 * Copyright (C) mx 2022
 *
 *
 ****************************************************************************************
 */

#ifndef APP_KEYPAD_H_
#define APP_KEYPAD_H_



typedef enum
{
    IR_ZOOM          = 0x6F,    // 比例
    IR_SEARCH        = 0x7F,    // 搜索
    IR_COLLECT       = 0xE1,    // 收藏
    IR_MITV          = 0x9E,    // MITV
    IR_MENU          = 0x62,    // 菜单
    IR_EXIT          = 0xF9,    // 返回
    IR_REVIEW        = 0xD8,    // 回看
    IR_APP_STORE     = 0x29,    // APP商店
    IR_POWER         = 0xD5,    // 待机
    IR_MUTE          = 0xC0,    // 静音
    IR_0             = 0xCF,    // 0
    IR_1             = 0xCE,    // 1    
    IR_2             = 0xCD,    // 2
    IR_3             = 0xCC,    // 3
    IR_4             = 0xCB,    // 4
    IR_5             = 0xCA,    // 5
    IR_6             = 0xC9,    // 6
    IR_7             = 0xC8,    // 7
    IR_8             = 0xC7,    // 8
    IR_9             = 0xC6,    // 9
    IR_ARROW_UP      = 0xA6,    // 上
    IR_ARROW_DOWN    = 0xA7,    // 下
    IR_ARROW_LEFT    = 0xA9,    // 左
    IR_ARROW_RIGHT   = 0xA8,    // 右
    IR_OK            = 0x0B,    // OK
    IR_HOME          = 0xF7,    // 主页
    IR_VOL_UP        = 0xD0,    // 音量+
    IR_VOL_DOWN      = 0xD1,    // 音量-
    IR_CH_DOWN       = 0xD3,    // 节目-
    IR_CH_UP         = 0xD2,    // 节目+
    IR_RED           = 0xFF,    // 红
    IR_GREEN         = 0x17,    // 绿
    IR_YELLOW        = 0x1B,    // 黄
    IR_BLUE          = 0x27,    // 蓝
    IR_PICTURE       = 0xED,    // 图效
    IR_EPG           = 0xE5,    // EPG
    IR_TV            = 0xC5,    // TV直播
    IR_USB           = 0xFD,    // usb
    IR_HOST          = 0x1C,    // 热键
    IR_SETTING       = 0x30,    // 设置
    IR_FREEZE        = 0xF3,    // 静止
    IR_DISPLAY       = 0xC3,    // 显示
    IR_INFO          = 0x2E,    // 信息窗
    IR_SOURCE        = 0x5C,    // 信源
    IR_GOLIVE        = 0x19,    // 全球播
    IR_HISTORY       = 0x31,    // 历史
    IR_SLEEP         = 0xF8,    // 睡眠
    IR_FAV           = 0xFA,    // 喜好
    IR_NUM           = 0x32,    // 数字
    IR_3D            = 0x67,    // 3D
    IR_PAIR          = 0xA0,    // 开启对码
    IR_UNPAIR        = 0xAB,    // 解除对码
    IR_RF_ERR        = 0xA3,    // RF连接异常提示
    IR_PAIN_ERR      = 0xAA,    // 对码失败提示
    IR_SOUND         = 0xA5,    // 音效
    IR_NEXT          = 0xAC,    // 下一首
    IR_RECORD        = 0xE8,    // 录制
    IR_MANGO         = 0x70,    // mangoTV
    IR_MORE          = 0x71,    // 更多
    IR_MIRCAST       = 0x05,    // 传屏
    IR_MIC           = 0xD6,    // 语音
    
    IR_OPTION        = 0x13,    // 选项
    IR_VOICEBOX      = 0x1F,    // 音箱
    //IR_MIC           = 0xA3,
    
}app_ir_value;





#endif // 






