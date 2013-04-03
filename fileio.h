#ifndef FILEIO_H
#define FILEIO_H

#define FILEIO_CMD_MAGIC		0xC0DECAFE

//
// opcodes
//

#define FILEIO_OPCODE_HDR		0x00000001
#define FILEIO_OPCODE_S64		0x0000000F
#define FILEIO_OPCODE_DELAY	0x0000000E

struct fileio_hdr {
	uint_32		magic;
	uint_32		opcode;
};

struct fileio_opcode_hdr {
	struct fileio_hdr hdr;
	uint_32		version;
	/* more will go here */
};

struct fileio_opcode_delay {
	struct fileio_hdr hdr;
	uint_32 delay; // msecs to wait
};

struct fileio_opcode_screen_64 {
	struct fileio_hdr hdr;
	uint_32		length;
	uint_32		data[64];
};


void fileio_play_file (uint_8 *filename);

#endif
