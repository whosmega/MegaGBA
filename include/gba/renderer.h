#ifndef gba_renderer_h
#define gba_renderer_h

typedef struct GBA GBA;

typedef enum {
	VMODE_0, 				// } -------------------
	VMODE_1, 				// }    Tilemap Modes
	VMODE_2,				// } -------------------
	VMODE_3, 				// } -------------------
	VMODE_4, 				// } 	Bitmap Modes
	VMODE_6 				// } -------------------
} PPU_VideoMode;

typedef enum {
	PPU_HDRAW, 				// } Horizontal State
	PPU_HBLANK,				// }

	PPU_VDRAW, 				// } Vertical State
	PPU_VBLANK				// }
} PPU_State;

/* Latch the value of DISPCNT into internal fields */
void latchDISPCNT(GBA* gba);
/* Step the PPU State */
void stepPPU(GBA* gba);

#endif
