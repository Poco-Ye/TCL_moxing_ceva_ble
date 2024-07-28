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
    IR_ZOOM          = 0x6F,    // ����
    IR_SEARCH        = 0x7F,    // ����
    IR_COLLECT       = 0xE1,    // �ղ�
    IR_MITV          = 0x9E,    // MITV
    IR_MENU          = 0x62,    // �˵�
    IR_EXIT          = 0xF9,    // ����
    IR_REVIEW        = 0xD8,    // �ؿ�
    IR_APP_STORE     = 0x29,    // APP�̵�
    IR_POWER         = 0xD5,    // ����
    IR_MUTE          = 0xC0,    // ����
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
    IR_ARROW_UP      = 0xA6,    // ��
    IR_ARROW_DOWN    = 0xA7,    // ��
    IR_ARROW_LEFT    = 0xA9,    // ��
    IR_ARROW_RIGHT   = 0xA8,    // ��
    IR_OK            = 0x0B,    // OK
    IR_HOME          = 0xF7,    // ��ҳ
    IR_VOL_UP        = 0xD0,    // ����+
    IR_VOL_DOWN      = 0xD1,    // ����-
    IR_CH_DOWN       = 0xD3,    // ��Ŀ-
    IR_CH_UP         = 0xD2,    // ��Ŀ+
    IR_RED           = 0xFF,    // ��
    IR_GREEN         = 0x17,    // ��
    IR_YELLOW        = 0x1B,    // ��
    IR_BLUE          = 0x27,    // ��
    IR_PICTURE       = 0xED,    // ͼЧ
    IR_EPG           = 0xE5,    // EPG
    IR_TV            = 0xC5,    // TVֱ��
    IR_USB           = 0xFD,    // usb
    IR_HOST          = 0x1C,    // �ȼ�
    IR_SETTING       = 0x30,    // ����
    IR_FREEZE        = 0xF3,    // ��ֹ
    IR_DISPLAY       = 0xC3,    // ��ʾ
    IR_INFO          = 0x2E,    // ��Ϣ��
    IR_SOURCE        = 0x5C,    // ��Դ
    IR_GOLIVE        = 0x19,    // ȫ��
    IR_HISTORY       = 0x31,    // ��ʷ
    IR_SLEEP         = 0xF8,    // ˯��
    IR_FAV           = 0xFA,    // ϲ��
    IR_NUM           = 0x32,    // ����
    IR_3D            = 0x67,    // 3D
    IR_PAIR          = 0xA0,    // ��������
    IR_UNPAIR        = 0xAB,    // �������
    IR_RF_ERR        = 0xA3,    // RF�����쳣��ʾ
    IR_PAIN_ERR      = 0xAA,    // ����ʧ����ʾ
    IR_SOUND         = 0xA5,    // ��Ч
    IR_NEXT          = 0xAC,    // ��һ��
    IR_RECORD        = 0xE8,    // ¼��
    IR_MANGO         = 0x70,    // mangoTV
    IR_MORE          = 0x71,    // ����
    IR_MIRCAST       = 0x05,    // ����
    IR_MIC           = 0xD6,    // ����
    
    IR_OPTION        = 0x13,    // ѡ��
    IR_VOICEBOX      = 0x1F,    // ����
    //IR_MIC           = 0xA3,
    
}app_ir_value;





#endif // 






