#pragma once

#include <Arduino.h>
#include <PluggableUSB.h>

#include <WS2812.h>
#include "HID.h"
#include "HID-Settings.h"

#define blink1_ver_major 2
#define blink1_ver_minor 3

enum ConsumerKeycode : uint16_t {
	// Some keys might only work with linux
	CONSUMER_POWER	= 0x30,
	CONSUMER_SLEEP = 0x32,

	MEDIA_RECORD = 0xB2,
	MEDIA_FAST_FORWARD	= 0xB3,
	MEDIA_REWIND	= 0xB4,
	MEDIA_NEXT	= 0xB5,
	MEDIA_PREVIOUS	= 0xB6,
	MEDIA_PREV	= 0xB6, // Alias
	MEDIA_STOP	= 0xB7,
	MEDIA_PLAY_PAUSE	= 0xCD,
	MEDIA_PAUSE	= 0xB0,

	MEDIA_VOLUME_MUTE	= 0xE2,
	MEDIA_VOL_MUTE = 0xE2, // Alias
	MEDIA_VOLUME_UP	= 0xE9,
	MEDIA_VOL_UP	= 0xE9, // Alias
	MEDIA_VOLUME_DOWN	= 0xEA,
	MEDIA_VOL_DOWN	= 0xEA, // Alias

	CONSUMER_SCREENSAVER = 0x19e,

	CONSUMER_PROGRAMMABLE_BUTTON_CONFIGURATION = 0x182,
	CONSUMER_CONTROL_CONFIGURATION = 0x183,
	CONSUMER_EMAIL_READER	= 0x18A,
	CONSUMER_CALCULATOR	= 0x192,
	CONSUMER_EXPLORER	= 0x194,

	CONSUMER_BROWSER_HOME	= 0x223,
	CONSUMER_BROWSER_BACK	= 0x224,
	CONSUMER_BROWSER_FORWARD	= 0x225,
	CONSUMER_BROWSER_REFRESH	= 0x227,
	CONSUMER_BROWSER_BOOKMARKS	= 0x22A,


	// Consumer_Page_(0x0C)	0x15
	HID_CONSUMER_UNASSIGNED		= 0x00,
	HID_CONSUMER_NUMERIC_KEY_PAD	= 0x02,	// HID type NARY
	HID_CONSUMER_PROGRAMMABLE_BUTTONS	= 0x03,	// HID type NARY
	HID_CONSUMER_MICROPHONE_CA	= 0x04,
	HID_CONSUMER_HEADPHONE_CA	= 0x05,
	HID_CONSUMER_GRAPHIC_EQUALIZER_CA	= 0x06,
	// Reserved	= 0x07-1F
	HID_CONSUMER_PLUS_10	= 0x20,	// HID type OSC
	HID_CONSUMER_PLUS_100	= 0x21,	// HID type OSC
	HID_CONSUMER_AM_SLASH_PM	= 0x22,	// HID type OSC
	// Reserved	= 0x23-3F
	HID_CONSUMER_POWER	= 0x30,	// HID type OOC
	HID_CONSUMER_RESET	= 0x31,	// HID type OSC
	HID_CONSUMER_SLEEP	= 0x32,	// HID type OSC
	HID_CONSUMER_SLEEP_AFTER	= 0x33,	// HID type OSC
	HID_CONSUMER_SLEEP_MODE	= 0x34,	// HID type RTC
	HID_CONSUMER_ILLUMINATION	= 0x35,	// HID type OOC
	HID_CONSUMER_FUNCTION_BUTTONS	= 0x36,	// HID type NARY
	// Reserved	= 0x37-3F
	HID_CONSUMER_MENU	= 0x40,	// HID type OOC
	HID_CONSUMER_MENU_PICK	= 0x41,	// HID type OSC
	HID_CONSUMER_MENU_UP	= 0x42,	// HID type OSC
	HID_CONSUMER_MENU_DOWN	= 0x43,	// HID type OSC
	HID_CONSUMER_MENU_LEFT	= 0x44,	// HID type OSC
	HID_CONSUMER_MENU_RIGHT	= 0x45,	// HID type OSC
	HID_CONSUMER_MENU_ESCAPE	= 0x46,	// HID type OSC
	HID_CONSUMER_MENU_VALUE_INCREASE	= 0x47,	// HID type OSC
	HID_CONSUMER_MENU_VALUE_DECREASE	= 0x48,	// HID type OSC
	// Reserved 0x49-5F
	HID_CONSUMER_DATA_ON_SCREEN	= 0x60,	// HID type OOC
	HID_CONSUMER_CLOSED_CAPTION	= 0x61,	// HID type OOC
	HID_CONSUMER_CLOSED_CAPTION_SELECT	= 0x62,	// HID type OSC
	HID_CONSUMER_VCR_SLASH_TV	= 0x63,	// HID type OOC
	HID_CONSUMER_BROADCAST_MODE	= 0x64,	// HID type OSC
	HID_CONSUMER_SNAPSHOT	= 0x65,	// HID type OSC
	HID_CONSUMER_STILL	= 0x66,	// HID type OSC
	// Reserved 0x67-7F
	HID_CONSUMER_SELECTION	= 0x80,	// HID type NARY
	HID_CONSUMER_ASSIGN_SELECTION	= 0x81,	// HID type OSC
	HID_CONSUMER_MODE_STEP	= 0x82,	// HID type OSC
	HID_CONSUMER_RECALL_LAST	= 0x83,	// HID type OSC
	HID_CONSUMER_ENTER_CHANNEL	= 0x84,	// HID type OSC
	HID_CONSUMER_ORDER_MOVIE	= 0x85,	// HID type OSC
	HID_CONSUMER_CHANNEL	= 0x86,	// HID type LC
	HID_CONSUMER_MEDIA_SELECTION	= 0x87,	// HID type NARY
	HID_CONSUMER_MEDIA_SELECT_COMPUTER	= 0x88,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_TV	= 0x89,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_WWW	= 0x8A,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_DVD	= 0x8B,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_TELEPHONE	= 0x8C,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_PROGRAM_GUIDE	= 0x8D,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_VIDEO_PHONE	= 0x8E,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_GAMES	= 0x8F,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_MESSAGES	= 0x90,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_CD	= 0x91,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_VCR	= 0x92,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_TUNER	= 0x93,	// HID type SEL
	HID_CONSUMER_QUIT	= 0x94,	// HID type OSC
	HID_CONSUMER_HELP	= 0x95,	// HID type OOC
	HID_CONSUMER_MEDIA_SELECT_TAPE	= 0x96,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_CABLE	= 0x97,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_SATELLITE	= 0x98,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_SECURITY	= 0x99,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_HOME	= 0x9A,	// HID type SEL
	HID_CONSUMER_MEDIA_SELECT_CALL	= 0x9B,	// HID type SEL
	HID_CONSUMER_CHANNEL_INCREMENT	= 0x9C,	// HID type OSC
	HID_CONSUMER_CHANNEL_DECREMENT	= 0x9D,	// HID type OSC
	HID_CONSUMER_MEDIA_SELECT_SAP	= 0x9E,	// HID type SEL
	// Reserved 0x9F
	HID_CONSUMER_VCR_PLUS	= 0xA0,	// HID type OSC
	HID_CONSUMER_ONCE	= 0xA1,	// HID type OSC
	HID_CONSUMER_DAILY	= 0xA2,	// HID type OSC
	HID_CONSUMER_WEEKLY	= 0xA3,	// HID type OSC
	HID_CONSUMER_MONTHLY	= 0xA4,	// HID type OSC
	// Reserved 0xA5-AF
	HID_CONSUMER_PLAY	= 0xB0,	// HID type OOC
	HID_CONSUMER_PAUSE	= 0xB1,	// HID type OOC
	HID_CONSUMER_RECORD	= 0xB2,	// HID type OOC
	HID_CONSUMER_FAST_FORWARD	= 0xB3,	// HID type OOC
	HID_CONSUMER_REWIND	= 0xB4,	// HID type OOC
	HID_CONSUMER_SCAN_NEXT_TRACK	= 0xB5,	// HID type OSC
	HID_CONSUMER_SCAN_PREVIOUS_TRACK	= 0xB6,	// HID type OSC
	HID_CONSUMER_STOP	= 0xB7,	// HID type OSC
	HID_CONSUMER_EJECT	= 0xB8,	// HID type OSC
	HID_CONSUMER_RANDOM_PLAY	= 0xB9,	// HID type OOC
	HID_CONSUMER_SELECT_DISC	= 0xBA,	// HID type NARY
	HID_CONSUMER_ENTER_DISC_MC	= 0xBB,
	HID_CONSUMER_REPEAT	= 0xBC,	// HID type OSC
	HID_CONSUMER_TRACKING	= 0xBD,	// HID type LC
	HID_CONSUMER_TRACK_NORMAL	= 0xBE,	// HID type OSC
	HID_CONSUMER_SLOW_TRACKING	= 0xBF,	// HID type LC
	HID_CONSUMER_FRAME_FORWARD	= 0xC0,	// HID type RTC
	HID_CONSUMER_FRAME_BACK	= 0xC1,	// HID type RTC
	HID_CONSUMER_MARK	= 0xC2,	// HID type OSC
	HID_CONSUMER_CLEAR_MARK	= 0xC3,	// HID type OSC
	HID_CONSUMER_REPEAT_FROM_MARK	= 0xC4,	// HID type OOC
	HID_CONSUMER_RETURN_TO_MARK	= 0xC5,	// HID type OSC
	HID_CONSUMER_SEARCH_MARK_FORWARD	= 0xC6,	// HID type OSC
	HID_CONSUMER_SEARCH_MARK_BACKWARDS	= 0xC7,	// HID type OSC
	HID_CONSUMER_COUNTER_RESET	= 0xC8,	// HID type OSC
	HID_CONSUMER_SHOW_COUNTER	= 0xC9,	// HID type OSC
	HID_CONSUMER_TRACKING_INCREMENT	= 0xCA,	// HID type RTC
	HID_CONSUMER_TRACKING_DECREMENT	= 0xCB,	// HID type RTC
	HID_CONSUMER_STOP_SLASH_EJECT	= 0xCC,	// HID type OSC
	HID_CONSUMER_PLAY_SLASH_PAUSE	= 0xCD,	// HID type OSC
	HID_CONSUMER_PLAY_SLASH_SKIP	= 0xCE,	// HID type OSC
	// Reserved 0xCF-DF
	HID_CONSUMER_VOLUME	= 0xE0,	// HID type LC
	HID_CONSUMER_BALANCE	= 0xE1,	// HID type LC
	HID_CONSUMER_MUTE	= 0xE2,	// HID type OOC
	HID_CONSUMER_BASS	= 0xE3,	// HID type LC
	HID_CONSUMER_TREBLE	= 0xE4,	// HID type LC
	HID_CONSUMER_BASS_BOOST	= 0xE5,	// HID type OOC
	HID_CONSUMER_SURROUND_MODE	= 0xE6,	// HID type OSC
	HID_CONSUMER_LOUDNESS	= 0xE7,	// HID type OOC
	HID_CONSUMER_MPX	= 0xE8,	// HID type OOC
	HID_CONSUMER_VOLUME_INCREMENT	= 0xE9,	// HID type RTC
	HID_CONSUMER_VOLUME_DECREMENT	= 0xEA,	// HID type RTC
	// Reserved 0xEB-EF
	HID_CONSUMER_SPEED_SELECT	= 0xF0,	// HID type OSC
	HID_CONSUMER_PLAYBACK_SPEED	= 0xF1,	// HID type NARY
	HID_CONSUMER_STANDARD_PLAY	= 0xF2,	// HID type SEL
	HID_CONSUMER_LONG_PLAY	= 0xF3,	// HID type SEL
	HID_CONSUMER_EXTENDED_PLAY	= 0xF4,	// HID type SEL
	HID_CONSUMER_SLOW	= 0xF5,	// HID type OSC
	// Reserved 0xF6-FF
	HID_CONSUMER_FAN_ENABLE	= 0x100,	// HID type OOC
	HID_CONSUMER_FAN_SPEED	= 0x101,	// HID type LC
	HID_CONSUMER_LIGHT_ENABLE	= 0x102,	// HID type OOC
	HID_CONSUMER_LIGHT_ILLUMINATION_LEVEL	= 0x103,	// HID type LC
	HID_CONSUMER_CLIMATE_CONTROL_ENABLE	= 0x104,	// HID type OOC
	HID_CONSUMER_ROOM_TEMPERATURE	= 0x105,	// HID type LC
	HID_CONSUMER_SECURITY_ENABLE	= 0x106,	// HID type OOC
	HID_CONSUMER_FIRE_ALARM	= 0x107,	// HID type OSC
	HID_CONSUMER_POLICE_ALARM	= 0x108,	// HID type OSC
	HID_CONSUMER_PROXIMITY	= 0x109,	// HID type LC
	HID_CONSUMER_MOTION	= 0x10A,	// HID type OSC
	HID_CONSUMER_DURESS_ALARM	= 0x10B,	// HID type OSC
	HID_CONSUMER_HOLDUP_ALARM	= 0x10C,	// HID type OSC
	HID_CONSUMER_MEDICAL_ALARM	= 0x10D,	// HID type OSC
	// Reserved 0x10E-14F
	HID_CONSUMER_BALANCE_RIGHT	= 0x150,	// HID type RTC
	HID_CONSUMER_BALANCE_LEFT	= 0x151,	// HID type RTC
	HID_CONSUMER_BASS_INCREMENT	= 0x152,	// HID type RTC
	HID_CONSUMER_BASS_DECREMENT	= 0x153,	// HID type RTC
	HID_CONSUMER_TREBLE_INCREMENT	= 0x154,	// HID type RTC
	HID_CONSUMER_TREBLE_DECREMENT	= 0x155,	// HID type RTC
	// Reserved 0x156-15F
	HID_CONSUMER_SPEAKER_SYSTEM	= 0x160,	// HID type CL
	HID_CONSUMER_CHANNEL_LEFT	= 0x161,	// HID type CL
	HID_CONSUMER_CHANNEL_RIGHT	= 0x162,	// HID type CL
	HID_CONSUMER_CHANNEL_CENTER	= 0x163,	// HID type CL
	HID_CONSUMER_CHANNEL_FRONT	= 0x164,	// HID type CL
	HID_CONSUMER_CHANNEL_CENTER_FRONT	= 0x165,	// HID type CL
	HID_CONSUMER_CHANNEL_SIDE	= 0x166,	// HID type CL
	HID_CONSUMER_CHANNEL_SURROUND	= 0x167,	// HID type CL
	HID_CONSUMER_CHANNEL_LOW_FREQUENCY_ENHANCEMENT	= 0x168,	// HID type CL
	HID_CONSUMER_CHANNEL_TOP	= 0x169,	// HID type CL
	HID_CONSUMER_CHANNEL_UNKNOWN	= 0x16A,	// HID type CL
	// Reserved 0x16B-16F
	HID_CONSUMER_SUB_CHANNEL	= 0x170,	// HID type LC
	HID_CONSUMER_SUB_CHANNEL_INCREMENT	= 0x171,	// HID type OSC
	HID_CONSUMER_SUB_CHANNEL_DECREMENT	= 0x172,	// HID type OSC
	HID_CONSUMER_ALTERNATE_AUDIO_INCREMENT	= 0x173,	// HID type OSC
	HID_CONSUMER_ALTERNATE_AUDIO_DECREMENT	= 0x174,	// HID type OSC
	// Reserved 0x175-17F
	HID_CONSUMER_APPLICATION_LAUNCH_BUTTONS	= 0x180,	// HID type NARY
	HID_CONSUMER_AL_LAUNCH_BUTTON_CONFIGURATION_TOOL	= 0x181,	// HID type SEL
	HID_CONSUMER_AL_PROGRAMMABLE_BUTTON_CONFIGURATION	= 0x182,	// HID type SEL
	HID_CONSUMER_AL_CONSUMER_CONTROL_CONFIGURATION	= 0x183,	// HID type SEL
	HID_CONSUMER_AL_WORD_PROCESSOR	= 0x184,	// HID type SEL
	HID_CONSUMER_AL_TEXT_EDITOR	= 0x185,	// HID type SEL
	HID_CONSUMER_AL_SPREADSHEET	= 0x186,	// HID type SEL
	HID_CONSUMER_AL_GRAPHICS_EDITOR	= 0x187,	// HID type SEL
	HID_CONSUMER_AL_PRESENTATION_APP	= 0x188,	// HID type SEL
	HID_CONSUMER_AL_DATABASE_APP	= 0x189,	// HID type SEL
	HID_CONSUMER_AL_EMAIL_READER	= 0x18A,	// HID type SEL
	HID_CONSUMER_AL_NEWSREADER	= 0x18B,	// HID type SEL
	HID_CONSUMER_AL_VOICEMAIL	= 0x18C,	// HID type SEL
	HID_CONSUMER_AL_CONTACTS_SLASH_ADDRESS_BOOK	= 0x18D,	// HID type SEL
	HID_CONSUMER_AL_CALENDAR_SLASH_SCHEDULE	= 0x18E,	// HID type SEL
	HID_CONSUMER_AL_TASK_SLASH_PROJECT_MANAGER	= 0x18F,	// HID type SEL
	HID_CONSUMER_AL_LOG_SLASH_JOURNAL_SLASH_TIMECARD	= 0x190,	// HID type SEL
	HID_CONSUMER_AL_CHECKBOOK_SLASH_FINANCE	= 0x191,	// HID type SEL
	HID_CONSUMER_AL_CALCULATOR	= 0x192,	// HID type SEL
	HID_CONSUMER_AL_A_SLASH_V_CAPTURE_SLASH_PLAYBACK	= 0x193,	// HID type SEL
	HID_CONSUMER_AL_LOCAL_MACHINE_BROWSER	= 0x194,	// HID type SEL
	HID_CONSUMER_AL_LAN_SLASH_WAN_BROWSER	= 0x195,	// HID type SEL
	HID_CONSUMER_AL_INTERNET_BROWSER	= 0x196,	// HID type SEL
	HID_CONSUMER_AL_REMOTE_NETWORKING_SLASH_ISP_CONNECT	= 0x197,	// HID type SEL
	HID_CONSUMER_AL_NETWORK_CONFERENCE	= 0x198,	// HID type SEL
	HID_CONSUMER_AL_NETWORK_CHAT	= 0x199,	// HID type SEL
	HID_CONSUMER_AL_TELEPHONY_SLASH_DIALER	= 0x19A,	// HID type SEL
	HID_CONSUMER_AL_LOGON	= 0x19B,	// HID type SEL
	HID_CONSUMER_AL_LOGOFF	= 0x19C,	// HID type SEL
	HID_CONSUMER_AL_LOGON_SLASH_LOGOFF	= 0x19D,	// HID type SEL
	HID_CONSUMER_AL_TERMINAL_LOCK_SLASH_SCREENSAVER	= 0x19E,	// HID type SEL
	HID_CONSUMER_AL_CONTROL_PANEL	= 0x19F,	// HID type SEL
	HID_CONSUMER_AL_COMMAND_LINE_PROCESSOR_SLASH_RUN	= 0x1A0,	// HID type SEL
	HID_CONSUMER_AL_PROCESS_SLASH_TASK_MANAGER	= 0x1A1,	// HID type SEL
	HID_CONSUMER_AL_SELECT_TASK_SLASH_APPLICATION	= 0x1A2,	// HID type SEL
	HID_CONSUMER_AL_NEXT_TASK_SLASH_APPLICATION	= 0x1A3,	// HID type SEL
	HID_CONSUMER_AL_PREVIOUS_TASK_SLASH_APPLICATION	= 0x1A4,	// HID type SEL
	HID_CONSUMER_AL_PREEMPTIVE_HALT_TASK_SLASH_APPLICATION	= 0x1A5,	// HID type SEL
	HID_CONSUMER_AL_INTEGRATED_HELP_CENTER	= 0x1A6,	// HID type SEL
	HID_CONSUMER_AL_DOCUMENTS	= 0x1A7,	// HID type SEL
	HID_CONSUMER_AL_THESAURUS	= 0x1A8,	// HID type SEL
	HID_CONSUMER_AL_DICTIONARY	= 0x1A9,	// HID type SEL
	HID_CONSUMER_AL_DESKTOP	= 0x1AA,	// HID type SEL
	HID_CONSUMER_AL_SPELL_CHECK	= 0x1AB,	// HID type SEL
	HID_CONSUMER_AL_GRAMMAR_CHECK	= 0x1AC,	// HID type SEL
	HID_CONSUMER_AL_WIRELESS_STATUS	= 0x1AD,	// HID type SEL
	HID_CONSUMER_AL_KEYBOARD_LAYOUT	= 0x1AE,	// HID type SEL
	HID_CONSUMER_AL_VIRUS_PROTECTION	= 0x1AF,	// HID type SEL
	HID_CONSUMER_AL_ENCRYPTION	= 0x1B0,	// HID type SEL
	HID_CONSUMER_AL_SCREEN_SAVER	= 0x1B1,	// HID type SEL
	HID_CONSUMER_AL_ALARMS	= 0x1B2,	// HID type SEL
	HID_CONSUMER_AL_CLOCK	= 0x1B3,	// HID type SEL
	HID_CONSUMER_AL_FILE_BROWSER	= 0x1B4,	// HID type SEL
	HID_CONSUMER_AL_POWER_STATUS	= 0x1B5,	// HID type SEL
	HID_CONSUMER_AL_IMAGE_BROWSER	= 0x1B6,	// HID type SEL
	HID_CONSUMER_AL_AUDIO_BROWSER	= 0x1B7,	// HID type SEL
	HID_CONSUMER_AL_MOVIE_BROWSER	= 0x1B8,	// HID type SEL
	HID_CONSUMER_AL_DIGITAL_RIGHTS_MANAGER	= 0x1B9,	// HID type SEL
	HID_CONSUMER_AL_DIGITAL_WALLET	= 0x1BA,	// HID type SEL
	// _Reserved 0x1BB
	HID_CONSUMER_AL_INSTANT_MESSAGING	= 0x1BC,	// HID type SEL
	HID_CONSUMER_AL_OEM_FEATURES_SLASH__TIPS_SLASH_TUTORIAL_BROWSER	= 0x1BD,	// HID type SEL
	HID_CONSUMER_AL_OEM_HELP	= 0x1BE,	// HID type SEL
	HID_CONSUMER_AL_ONLINE_COMMUNITY	= 0x1BF,	// HID type SEL
	HID_CONSUMER_AL_ENTERTAINMENT_CONTENT_BROWSER	= 0x1C0,	// HID type SEL
	HID_CONSUMER_AL_ONLINE_SHOPPING_BROWSER	= 0x1C1,	// HID type SEL
	HID_CONSUMER_AL_SMARTCARD_INFORMATION_SLASH_HELP	= 0x1C2,	// HID type SEL
	HID_CONSUMER_AL_MARKET_MONITOR_SLASH_FINANCE_BROWSER	= 0x1C3,	// HID type SEL
	HID_CONSUMER_AL_CUSTOMIZED_CORPORATE_NEWS_BROWSER	= 0x1C4,	// HID type SEL
	HID_CONSUMER_AL_ONLINE_ACTIVITY_BROWSER	= 0x1C5,	// HID type SEL
	HID_CONSUMER_AL_RESEARCH_SLASH_SEARCH_BROWSER	= 0x1C6,	// HID type SEL
	HID_CONSUMER_AL_AUDIO_PLAYER	= 0x1C7,	// HID type SEL
	// Reserved 0x1C8-1FF
	HID_CONSUMER_GENERIC_GUI_APPLICATION_CONTROLS	= 0x200, // HID type NARY
	HID_CONSUMER_AC_NEW	= 0x201,	// HID type SEL
	HID_CONSUMER_AC_OPEN	= 0x202,	// HID type SEL
	HID_CONSUMER_AC_CLOSE	= 0x203,	// HID type SEL
	HID_CONSUMER_AC_EXIT	= 0x204,	// HID type SEL
	HID_CONSUMER_AC_MAXIMIZE	= 0x205,	// HID type SEL
	HID_CONSUMER_AC_MINIMIZE	= 0x206,	// HID type SEL
	HID_CONSUMER_AC_SAVE	= 0x207,	// HID type SEL
	HID_CONSUMER_AC_PRINT	= 0x208,	// HID type SEL
	HID_CONSUMER_AC_PROPERTIES	= 0x209,	// HID type SEL
	HID_CONSUMER_AC_UNDO	= 0x21A,	// HID type SEL
	HID_CONSUMER_AC_COPY	= 0x21B,	// HID type SEL
	HID_CONSUMER_AC_CUT	= 0x21C,	// HID type SEL
	HID_CONSUMER_AC_PASTE	= 0x21D,	// HID type SEL
	HID_CONSUMER_AC_SELECT_ALL	= 0x21E,	// HID type SEL
	HID_CONSUMER_AC_FIND	= 0x21F,	// HID type SEL
	HID_CONSUMER_AC_FIND_AND_REPLACE	= 0x220,	// HID type SEL
	HID_CONSUMER_AC_SEARCH	= 0x221,	// HID type SEL
	HID_CONSUMER_AC_GO_TO	= 0x222,	// HID type SEL
	HID_CONSUMER_AC_HOME	= 0x223,	// HID type SEL
	HID_CONSUMER_AC_BACK	= 0x224,	// HID type SEL
	HID_CONSUMER_AC_FORWARD	= 0x225,	// HID type SEL
	HID_CONSUMER_AC_STOP	= 0x226,	// HID type SEL
	HID_CONSUMER_AC_REFRESH	= 0x227,	// HID type SEL
	HID_CONSUMER_AC_PREVIOUS_LINK	= 0x228,	// HID type SEL
	HID_CONSUMER_AC_NEXT_LINK	= 0x229,	// HID type SEL
	HID_CONSUMER_AC_BOOKMARKS	= 0x22A,	// HID type SEL
	HID_CONSUMER_AC_HISTORY	= 0x22B,	// HID type SEL
	HID_CONSUMER_AC_SUBSCRIPTIONS	= 0x22C,	// HID type SEL
	HID_CONSUMER_AC_ZOOM_IN	= 0x22D,	// HID type SEL
	HID_CONSUMER_AC_ZOOM_OUT	= 0x22E,	// HID type SEL
	HID_CONSUMER_AC_ZOOM	= 0x22F,	// HID type LC
	HID_CONSUMER_AC_FULL_SCREEN_VIEW	= 0x230,	// HID type SEL
	HID_CONSUMER_AC_NORMAL_VIEW	= 0x231,	// HID type SEL
	HID_CONSUMER_AC_VIEW_TOGGLE	= 0x232,	// HID type SEL
	HID_CONSUMER_AC_SCROLL_UP	= 0x233,	// HID type SEL
	HID_CONSUMER_AC_SCROLL_DOWN	= 0x234,	// HID type SEL
	HID_CONSUMER_AC_SCROLL	= 0x235,	// HID type LC
	HID_CONSUMER_AC_PAN_LEFT	= 0x236,	// HID type SEL
	HID_CONSUMER_AC_PAN_RIGHT	= 0x237,	// HID type SEL
	HID_CONSUMER_AC_PAN	= 0x238,	// HID type LC
	HID_CONSUMER_AC_NEW_WINDOW	= 0x239,	// HID type SEL
	HID_CONSUMER_AC_TILE_HORIZONTALLY	= 0x23A,	// HID type SEL
	HID_CONSUMER_AC_TILE_VERTICALLY	= 0x23B,	// HID type SEL
	HID_CONSUMER_AC_FORMAT	= 0x23C,	// HID type SEL
	HID_CONSUMER_AC_EDIT	= 0x23D,	// HID type SEL
	HID_CONSUMER_AC_BOLD	= 0x23E,	// HID type SEL
	HID_CONSUMER_AC_ITALICS	= 0x23F,	// HID type SEL
	HID_CONSUMER_AC_UNDERLINE	= 0x240,	// HID type SEL
	HID_CONSUMER_AC_STRIKETHROUGH	= 0x241,	// HID type SEL
	HID_CONSUMER_AC_SUBSCRIPT	= 0x242,	// HID type SEL
	HID_CONSUMER_AC_SUPERSCRIPT	= 0x243,	// HID type SEL
	HID_CONSUMER_AC_ALL_CAPS	= 0x244,	// HID type SEL
	HID_CONSUMER_AC_ROTATE	= 0x245,	// HID type SEL
	HID_CONSUMER_AC_RESIZE	= 0x246,	// HID type SEL
	HID_CONSUMER_AC_FLIP_HORIZONTAL	= 0x247,	// HID type SEL
	HID_CONSUMER_AC_FLIP_VERTICAL	= 0x248,	// HID type SEL
	HID_CONSUMER_AC_MIRROR_HORIZONTAL	= 0x249,	// HID type SEL
	HID_CONSUMER_AC_MIRROR_VERTICAL	= 0x24A,	// HID type SEL
	HID_CONSUMER_AC_FONT_SELECT	= 0x24B,	// HID type SEL
	HID_CONSUMER_AC_FONT_COLOR	= 0x24C,	// HID type SEL
	HID_CONSUMER_AC_FONT_SIZE	= 0x24D,	// HID type SEL
	HID_CONSUMER_AC_JUSTIFY_LEFT	= 0x24E,	// HID type SEL
	HID_CONSUMER_AC_JUSTIFY_CENTER_H	= 0x24F,	// HID type SEL
	HID_CONSUMER_AC_JUSTIFY_RIGHT	= 0x250,	// HID type SEL
	HID_CONSUMER_AC_JUSTIFY_BLOCK_H	= 0x251,	// HID type SEL
	HID_CONSUMER_AC_JUSTIFY_TOP	= 0x252,	// HID type SEL
	HID_CONSUMER_AC_JUSTIFY_CENTER_V	= 0x253,	// HID type SEL
	HID_CONSUMER_AC_JUSTIFY_BOTTOM	= 0x254,	// HID type SEL
	HID_CONSUMER_AC_JUSTIFY_BLOCK_V	= 0x255,	// HID type SEL
	HID_CONSUMER_AC_INDENT_DECREASE	= 0x256,	// HID type SEL
	HID_CONSUMER_AC_INDENT_INCREASE	= 0x257,	// HID type SEL
	HID_CONSUMER_AC_NUMBERED_LIST	= 0x258,	// HID type SEL
	HID_CONSUMER_AC_RESTART_NUMBERING	= 0x259,	// HID type SEL
	HID_CONSUMER_AC_BULLETED_LIST	= 0x25A,	// HID type SEL
	HID_CONSUMER_AC_PROMOTE	= 0x25B,	// HID type SEL
	HID_CONSUMER_AC_DEMOTE	= 0x25C,	// HID type SEL
	HID_CONSUMER_AC_YES	= 0x25D,	// HID type SEL
	HID_CONSUMER_AC_NO	= 0x25E,	// HID type SEL
	HID_CONSUMER_AC_CANCEL	= 0x25F,	// HID type SEL
	HID_CONSUMER_AC_CATALOG	= 0x260,	// HID type SEL
	HID_CONSUMER_AC_BUY_SLASH_CHECKOUT	= 0x261,	// HID type SEL
	HID_CONSUMER_AC_ADD_TO_CART	= 0x262,	// HID type SEL
	HID_CONSUMER_AC_EXPAND	= 0x263,	// HID type SEL
	HID_CONSUMER_AC_EXPAND_ALL	= 0x264,	// HID type SEL
	HID_CONSUMER_AC_COLLAPSE	= 0x265,	// HID type SEL
	HID_CONSUMER_AC_COLLAPSE_ALL	= 0x266,	// HID type SEL
	HID_CONSUMER_AC_PRINT_PREVIEW	= 0x267,	// HID type SEL
	HID_CONSUMER_AC_PASTE_SPECIAL	= 0x268,	// HID type SEL
	HID_CONSUMER_AC_INSERT_MODE	= 0x269,	// HID type SEL
	HID_CONSUMER_AC_DELETE	= 0x26A,	// HID type SEL
	HID_CONSUMER_AC_LOCK	= 0x26B,	// HID type SEL
	HID_CONSUMER_AC_UNLOCK	= 0x26C,	// HID type SEL
	HID_CONSUMER_AC_PROTECT	= 0x26D,	// HID type SEL
	HID_CONSUMER_AC_UNPROTECT	= 0x26E,	// HID type SEL
	HID_CONSUMER_AC_ATTACH_COMMENT	= 0x26F,	// HID type SEL
	HID_CONSUMER_AC_DELETE_COMMENT	= 0x270,	// HID type SEL
	HID_CONSUMER_AC_VIEW_COMMENT	= 0x271,	// HID type SEL
	HID_CONSUMER_AC_SELECT_WORD	= 0x272,	// HID type SEL
	HID_CONSUMER_AC_SELECT_SENTENCE	= 0x273,	// HID type SEL
	HID_CONSUMER_AC_SELECT_PARAGRAPH	= 0x274,	// HID type SEL
	HID_CONSUMER_AC_SELECT_COLUMN	= 0x275,	// HID type SEL
	HID_CONSUMER_AC_SELECT_ROW	= 0x276,	// HID type SEL
	HID_CONSUMER_AC_SELECT_TABLE	= 0x277,	// HID type SEL
	HID_CONSUMER_AC_SELECT_OBJECT	= 0x278,	// HID type SEL
	HID_CONSUMER_AC_REDO_SLASH_REPEAT	= 0x279,	// HID type SEL
	HID_CONSUMER_AC_SORT	= 0x27A,	// HID type SEL
	HID_CONSUMER_AC_SORT_ASCENDING	= 0x27B,	// HID type SEL
	HID_CONSUMER_AC_SORT_DESCENDING	= 0x27C,	// HID type SEL
	HID_CONSUMER_AC_FILTER	= 0x27D,	// HID type SEL
	HID_CONSUMER_AC_SET_CLOCK	= 0x27E,	// HID type SEL
	HID_CONSUMER_AC_VIEW_CLOCK	= 0x27F,	// HID type SEL
	HID_CONSUMER_AC_SELECT_TIME_ZONE	= 0x280,	// HID type SEL
	HID_CONSUMER_AC_EDIT_TIME_ZONES	= 0x281,	// HID type SEL
	HID_CONSUMER_AC_SET_ALARM	= 0x282,	// HID type SEL
	HID_CONSUMER_AC_CLEAR_ALARM	= 0x283,	// HID type SEL
	HID_CONSUMER_AC_SNOOZE_ALARM	= 0x284,	// HID type SEL
	HID_CONSUMER_AC_RESET_ALARM	= 0x285,	// HID type SEL
	HID_CONSUMER_AC_SYNCHRONIZE	= 0x286,	// HID type SEL
	HID_CONSUMER_AC_SEND_SLASH_RECEIVE	= 0x287,	// HID type SEL
	HID_CONSUMER_AC_SEND_TO	= 0x288,	// HID type SEL
	HID_CONSUMER_AC_REPLY	= 0x289,	// HID type SEL
	HID_CONSUMER_AC_REPLY_ALL	= 0x28A,	// HID type SEL
	HID_CONSUMER_AC_FORWARD_MSG	= 0x28B,	// HID type SEL
	HID_CONSUMER_AC_SEND	= 0x28C,	// HID type SEL
	HID_CONSUMER_AC_ATTACH_FILE	= 0x28D,	// HID type SEL
	HID_CONSUMER_AC_UPLOAD	= 0x28E,	// HID type SEL
	HID_CONSUMER_AC_DOWNLOAD_SAVE_TARGET_AS	= 0x28F,	// HID type SEL
	HID_CONSUMER_AC_SET_BORDERS	= 0x290,	// HID type SEL
	HID_CONSUMER_AC_INSERT_ROW	= 0x291,	// HID type SEL
	HID_CONSUMER_AC_INSERT_COLUMN	= 0x292,	// HID type SEL
	HID_CONSUMER_AC_INSERT_FILE	= 0x293,	// HID type SEL
	HID_CONSUMER_AC_INSERT_PICTURE	= 0x294,	// HID type SEL
	HID_CONSUMER_AC_INSERT_OBJECT	= 0x295,	// HID type SEL
	HID_CONSUMER_AC_INSERT_SYMBOL	= 0x296,	// HID type SEL
	HID_CONSUMER_AC_SAVE_AND_CLOSE	= 0x297,	// HID type SEL
	HID_CONSUMER_AC_RENAME	= 0x298,	// HID type SEL
	HID_CONSUMER_AC_MERGE	= 0x299,	// HID type SEL
	HID_CONSUMER_AC_SPLIT	= 0x29A,	// HID type SEL
	HID_CONSUMER_AC_DISRIBUTE_HORIZONTALLY	= 0x29B,	// HID type SEL
	HID_CONSUMER_AC_DISTRIBUTE_VERTICALLY	= 0x29C,	// HID type SEL
};


typedef union {
	// Every usable Consumer key possible, up to 4 keys presses possible
	uint8_t whole8[0];
	uint16_t whole16[0];
	uint32_t whole32[0];
	ConsumerKeycode keys[4];
	struct {
		ConsumerKeycode key1;
		ConsumerKeycode key2;
		ConsumerKeycode key3;
		ConsumerKeycode key4;
	};
} HID_ConsumerControlReport_Data_t;

class Blink1HID : public PluggableUSBModule
{
private:
	uint8_t epType[1];

	uint8_t protocol;
	uint8_t idle;

	WS2812 led;
	int playing;

	HID_ConsumerControlReport_Data_t _report;
protected:
    // Implementation of the PUSBListNode
    int getInterface(uint8_t* interfaceCount);
    int getDescriptor(USBSetup& setup);
    bool setup(USBSetup& setup);
	uint8_t getShortName(char*);
	int sendReportConsumer();
	int SendReport(uint8_t id, const void* data, int len);

	void handleMessage(uint8_t *msgbuf, uint16_t length);
	void rgb_setCurr(uint8_t *buf);
	void rgb_setDest(uint8_t *buf, uint16_t steps);

public:
    Blink1HID(int led_pin);
    void wakeupHost(void);

	inline void begin(void) { end(); }
	inline void end(void) { memset(&_report, 0, sizeof(_report)); sendReportConsumer(); }
	inline void write(ConsumerKeycode m) { press(m); release(m); }
	inline void press(ConsumerKeycode m) { for (uint8_t i = 0; i < 4; i++) { if (_report.keys[i] == HID_CONSUMER_UNASSIGNED) { _report.keys[i] = m; break; } }; sendReportConsumer(); }
	inline void release(ConsumerKeycode m) { for (uint8_t i = 0; i < 4; i++) { if (_report.keys[i] == m) { _report.keys[i] = HID_CONSUMER_UNASSIGNED; } } sendReportConsumer(); }
	inline void releaseAll(void) { end(); }
};
