#ifndef __MAIN_H
#define __MAIN_H
#define MAX_TRACK_SIZE	0x1700
#define MAX_SECTORS_PER_TRACK   27
#define MAIN_THREAD_SEEK_COMMAND 1
#define MAIN_THREAD_CHANGE_DISK_COMMAND 2
#define DISK_FLUSH_COUNT 1000000		// how many times through the main thread loop before we autoflush to disk
#endif

