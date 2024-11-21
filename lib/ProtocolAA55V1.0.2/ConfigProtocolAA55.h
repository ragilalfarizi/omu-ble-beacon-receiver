#ifndef ConfigProtocolAA55_h
#define ConfigProtocolAA55_h

#define MAX_BUFFER_DATA 512
#define MAX_TYPE_OF_OIL 6

#define DEBUG_INTERNAL_LIB 1

// define header usage in a protcol comm
#define HEADER_DATA_1ST_BYTE 0xAA
#define HEADER_DATA_2ND_BYTE 0xAA

// define header usage in a protcol comm
#define TAIL_DATA_1ST_BYTE 0x55
#define TAIL_DATA_2ND_BYTE 0x55

// data manipulation
#define identifyDiffHeaderandData 0xA5
#define identifyDiffTailData      0x5A
#define HeaderDataCombination1    0xAA
#define HeaderDataCombination2    0xAA
#define TailDataCombination1      0x55
#define TailDataCombination2      0x55

#endif
