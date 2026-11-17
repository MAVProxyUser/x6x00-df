# x6x00-df on a host: the engine's checks and its cost. The daemon itself is built for the radio by GoldCorpHF (its
# tools/rtlsdr/build.sh and the image's x6x00-rtlspec package), against the RTL-SDR Blog librtlsdr with
# rtlsdr_set_dithering; "make x6x00-df" builds it on a Linux host that has those.
CC     ?= cc
CFLAGS ?= -O2 -Wall -Wextra
ENGINE  = df_cal.c df_chan.c df_doa.c df_engine.c
DAEMON  = $(shell sed -e '/^\#/d' df_sources)

all: test

test: df_test
	./df_test

df_test: df_test.c $(ENGINE) df.h
	$(CC) $(CFLAGS) -o $@ df_test.c $(ENGINE) -lm

df_bench: df_bench.c $(ENGINE) df.h
	$(CC) -O3 -o $@ df_bench.c $(ENGINE) -lm

x6x00-df: $(DAEMON) df.h df_proto.h
	$(CC) -O3 -Wall -o $@ $(DAEMON) $$(pkg-config --cflags --libs librtlsdr libusb-1.0) -lpthread -lm

clean:
	rm -f df_test df_bench x6x00-df

.PHONY: all test clean
