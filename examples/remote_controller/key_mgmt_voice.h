/**
 * Copyright 2021 by MooreSilicon.All rights reserved
 * @file  key_mgmt_voice.h
 * @brief
 * @author weiquan.ou
 * @date 2022-4-27
 * @version 1.0
 * @Revision
 */

enum key_mgmt_voice_mode {
    KEY_MGMT_VOICE_ON_REQUEST = 0x0,
    KEY_MGMT_VOICE_PTT = 0x1,
    KEY_MGMT_VOICE_HTT = 0x3
};

struct key_mgmt_voice_cap {

    /*
     * The spec version that is implemented on the Remote Device.
     * 0x0100: version 1.0;
     * 0x0004: version 0.4e
     */
    unsigned short version;

    /*
     * Audio codecs that are supported on the Remote Device.
     * 0x01: [0000_0001B] ADPCM 8khz/16bit;
     * 0x02: [0000_0010B] ADPCM 16khz/16bit (recommended);
     * 0x03: [0000_0011B] ADPCM 8khz/16bit & 16khz/16bit. The value should be used if the Remote Device supports
     */
    unsigned char codecs_supported;

    /*
     * Assistant interaction model that is used by this Remote Device. The returned value should be aligned with ¡°supported assistant interaction models¡± value provided by the Android TV Device.
     * 0x00: On-request model (should be supported by all remotes);
     * 0x01: Press-to-Talk model enabled;
     * 0x03: Hold-to-Talk model enabled.
     */
    unsigned char assistant_interaction_model;

    /*
     * The desired audio packet size. The value is used in audio frame counting and usually matches the maximum payload size of a single notification, but can be any arbitrary number.
     * Some examples:
     * 0x0014: 20 bytes (default);
     * 0x00A0: 160 bytes (recommended for 16kHz audio codec and 20ms connection interval).
     */
    unsigned short audio_frame_size;

    /*
     * 0x01: [0000_0001B] If set, host will attempt to enable DLE (and negotiate a new ATT_MTU size) which would allow to transfer an entire audio frame in a single bluetooth packet.
     * It¡¯s recommended for the Remote to implement the ¡°GATT Client¡± role and negotiate ATT MTU and send DLE before sending CAPS_RESP message. Then there is no need to request DLE from the host side.
     */
    unsigned char extra_configuration;

    unsigned char reserved;
};

