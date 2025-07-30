# megagba

INCLUDE=include
INCLUDE_GBA=$(INCLUDE)/gba
SRC_GB=gba
DEBUG=debug

CC = gcc
CPPC = g++
CFLAGS = -O3 `sdl2-config --cflags` -I$(INCLUDE)
LFLAGS = -O3 `sdl2-config --libs` -lm
EXE = megagba

BIN_GBA = arm7tdmi.o debugGBA.o gamepak.o gba.o renderer.o
BIN_IMGUI = imgui.o imgui_tables.o imgui_draw.o imgui_widgets.o imgui_impl_sdlrenderer2.o imgui_impl_sdl2.o

# test suite

ASMFLAGS = -i $(DEBUG)/test_suite/

$(EXE): $(BIN_GBA) $(BIN_IMGUI) main.o
	$(CPPC) $(BIN_GBA) $(BIN_IMGUI) main.o $(LFLAGS) -o $(EXE)
# ----------------------------------------------------------------------
gamepak.o : $(INCLUDE_GBA)/gamepak.h \
              $(SRC_GB)/gamepak.c
	$(CC) -c $(SRC_GB)/gamepak.c $(CFLAGS)

gui.o : $(INCLUDE_GBA)/gui.h $(INCLUDE_GBA)/debug.h \
        $(SRC_GB)/gui.cpp
	$(CPPC) -c $(SRC_GB)/gui.cpp $(CFLAGS) -Iimgui

gba.o : $(INCLUDE_GBA)/gba.h $(INCLUDE_GBA)/gamepak.h $(INCLUDE_GBA)/debugGBA.h \
        $(INCLUDE_GBA)/arm7tdmi.h \
       	$(SRC_GB)/gba.c
	$(CC) -c $(SRC_GB)/gba.c $(CFLAGS)

main.o : $(INCLUDE_GBA)/gba.h \
         main.c
	$(CC) -c main.c $(CFLAGS)

arm7tdmi.o : $(INCLUDE_GBA)/arm7tdmi.h $(INCLUDE_GBA)/gba.h \
        $(SRC_GB)/arm7tdmi.c
	$(CC) -c $(SRC_GB)/arm7tdmi.c $(CFLAGS)

renderer.o : $(INCLUDE_GBA)/renderer.h $(INCLUDE_GBA)/gui.h $(INCLUDE_GBA)/gba.h\
            $(SRC_GB)/renderer.c
	$(CC) -c $(SRC_GB)/renderer.c $(CFLAGS)

debugGBA.o : $(INCLUDE_GBA)/debugGBA.h \
         $(SRC_GB)/debugGBA.c
	$(CC) -c $(SRC_GB)/debugGBA.c $(CFLAGS)

# --------------------------------------------------------------------
imgui.o: imgui/imgui.cpp
	$(CPPC) -c imgui/imgui.cpp $(CFLAGS) -Iimgui
imgui_draw.o: imgui/imgui_draw.cpp
	$(CPPC) -c imgui/imgui_draw.cpp $(CFLAGS) -Iimgui
imgui_tables.o: imgui/imgui_tables.cpp
	$(CPPC) -c imgui/imgui_tables.cpp $(CFLAGS) -Iimgui
imgui_widgets.o: imgui/imgui_widgets.cpp
	$(CPPC) -c imgui/imgui_widgets.cpp $(CFLAGS) -Iimgui
imgui_impl_sdlrenderer2.o: imgui/backends/imgui_impl_sdlrenderer2.cpp
	$(CPPC) -c imgui/backends/imgui_impl_sdlrenderer2.cpp $(CFLAGS) -Iimgui
imgui_impl_sdl2.o: imgui/backends/imgui_impl_sdl2.cpp
	$(CPPC) -c imgui/backends/imgui_impl_sdl2.cpp $(CFLAGS) -Iimgui

# --------------------------------------------------------------------
tests: edge_sprite.o sound.o
	rgblink -o edge_sprite.gb edge_sprite.o
	rgblink -o sound.gb sound.o
	rgbfix -v -p 0xFF edge_sprite.gb
	rgbfix -v -p 0xFF sound.gb

	mkdir -p roms
	mv *.gb roms/

edge_sprite.o :
	rgbasm $(ASMFLAGS) -L -o edge_sprite.o $(DEBUG)/test_suite/edge_sprite.s

sound.o :
	rgbasm $(ASMFLAGS) -L -o sound.o $(DEBUG)/test_suite/sound.s

clean:
	rm *.o

