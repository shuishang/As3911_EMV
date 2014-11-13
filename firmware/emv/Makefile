# *****************************************************************************
# * Copyright @ 2009 by austriamicrosystems AG                                *
# * All rights are reserved.                                                  *
# *                                                                           *
# * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
# * THE SOFTWARE.                                                             *
# *                                                                           *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       * 
# * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
# * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
# * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
# * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
# * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
# * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
# * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
# * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
# * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
# * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
# *****************************************************************************

###############################################################################
# Define here the target for that you want to build 
# either uncomment one target here or call: make TARGET=as3933 all
###############################################################################
TARGET=AS3911_EMV
DISABLE_BOOTLOADER=1
# to enable debugging define the symbol DEBUG 
#DEBUG=1

###############################################################################
# Define some variables later used by the makefile
###############################################################################
CPU=24FJ64GB002
C30_RESOURE_FILE = $(C30DIR)"/bin/c30_device.info"

ROOT_DIR=../../..
MCU_DIR=$(ROOT_DIR)/common/firmware/microchip
USB_DIR=$(MCU_DIR)/usb
USB_HID_DRIVER_DIR=$(MCU_DIR)/usb/hid_device_driver
USB_HID_STREAM_DIR=$(MCU_DIR)/usb/hid_stream_driver

SOURCE_DIR=src
BUILD_DIR=build
DOC_DIR=doc
DOC_OUTPUT_DIR=$(DOC_DIR)/generated

# additional libraries to be linked.
# e.g. the peripheral library for the pic24f: pPIC24Fxxx
#LIBS=pPIC24Fxxx
LIBS= 

###############################################################################
# Try to find the microchip toolchain - if the variable c30dir is not set as
# e.g. in Windows as a system variable.
###############################################################################
ifndef c30dir
c30dir= $(wildcard /cygdrive/c/c30 /cygdrive/c/Program\ Files/Microchip/MPLAB\ C30)
endif
C30DIR="$(c30dir)"

###############################################################################
# Set tool chain
###############################################################################
CC = $(C30DIR)/bin/pic30-gcc
AS = $(C30DIR)/bin/pic30-as
LD = $(C30DIR)/bin/pic30-ld
AR = $(C30DIR)/bin/pic30-ar
HEXER = $(C30DIR)/bin/pic30-bin2hex
HEX2BIN = objcopy -I ihex -O binary
DOXYGEN = doxygen
PICPROGRAMMER = "C:/Program Files (x86)/Microchip/MPLAB IDE/Programmer Utilities/ICD3/ICD3CMD.exe"

###############################################################################
# Set flags
###############################################################################
LIBRARIES=$(addprefix -l,$(LIBS))
INC_DIRS=$(addprefix -I,$(INCLUDE_DIRS))
CFLAGS=\
	-D__PIC$(CPU)__=1 \
	$(INC_DIRS) -I$(C30DIR)/support/PIC24F/h \
	-I$(C30DIR)/support/peripheral_24F -I$(C30DIR)/include \
	-Wall -O2 -fno-omit-frame-pointer -mcpu=$(CPU) \
	-mresource=$(C30_RESOURE_FILE) 

ASFLAGS=-p$(CPU)
LDFLAGS=-mcpu=$(CPU)\
	-Wl,--script="$(LINKER_SCRIPT)",--report-mem,-Map,$(BUILD_DIR)/$(TARGET).map

ifdef DEBUG
CFLAGS+=-DDEBUG -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -g 
LDFLAGS+=-Wl,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--defsym=__ICD2RAM=1
ifndef DISABLE_BOOTLOADER
$(warning Check your $(LINKER_SCRIPT) file that 0x800-0x850 is free for the debugger or disable the bootloader)
endif
endif

ifndef DISABLE_BOOTLOADER
CFLAGS += -DHAS_BOOTLOADER
LINKER_SCRIPT = $(SOURCE_DIR)/p24FJ64GB002_BL.gld
C_FILES+=irq_table.c
else
LINKER_SCRIPT = $(SOURCE_DIR)/p24FJ64GB002.gld
endif

###############################################################################
# Add all include directories
###############################################################################
INCLUDE_DIRS+=\
	$(MCU_DIR)/include \
    $(SOURCE_DIR) \
	$(SOURCE_DIR)/AFE \
	$(SOURCE_DIR)/AS3910 \
	$(SOURCE_DIR)/AS3911 \    


vpath %.c $(SOURCE_DIR)
vpath %.c $(SOURCE_DIR)/AFE
vpath %.c $(SOURCE_DIR)/AS3910
vpath %.c $(SOURCE_DIR)/AS3911
vpath %.c $(SOURCE_DIR)/EMV
vpath %.s $(SOURCE_DIR)
vpath %.c $(MCU_DIR)/src

C_FILES+=\
	main.c	\
    led.c \
	as3911_io.c \
	as3911_interrupt.c \
	as3911_com.c \
	as3911_modulation_adjustment.c \
	as3911_gain_adjustment.c \
	emv_digital.c \
	emv_display.c \
	emv_hal.c \
	emv_layer4.c \
	emv_main.c \
	emv_picc.c \
	emv_poll.c \
	emv_prelayer4.c \
	emv_prevalidation.c \
	emv_response_buffer.c \
	emv_typeA.c \
	emv_typeB.c \
	emv_gui.c \
	sleep.c \
	usb_descriptors.c \
	usb_device.c \
	usb_hal_pic24.c \
	usb_function_hid.c \
	usb_hid_stream_driver.c \
	usb_hid_stream_appl_handler.c \
	timer_driver.c \
	spi_driver.c \
	logger.c \
	uart.c

ASM_FILES=\




###############################################################################
# Object file *.o are all c-source and asm-source files. First we remove
# everything but the name of the file, than we substitute the suffix to *.o
# and finally we add the build directory as a prefix to *.o.
###############################################################################
OBJECTS:=\
	$(addprefix $(BUILD_DIR)/,$(patsubst %.c,%.o,$(C_FILES)))   \
	$(addprefix $(BUILD_DIR)/,$(patsubst %.s,%.o,$(ASM_FILES))) \

###############################################################################
# Rules
###############################################################################

all: $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex


# build the executabe by linking all o-files
$(BUILD_DIR)/$(TARGET).cof: $(OBJECTS)
	$(CC) -o $@ $(LDFLAGS) $(OBJECTS) $(LIBRARIES)

# build the hex file
%.hex: %.cof
	$(HEXER) $<

# build the binary file from the hex file
%.bin: %.hex
	$(HEX2BIN) $< $@

.PHONY: doc
doc: 
	@echo "Creating doxygen configuration file"; \
	echo "@INCLUDE               = \"$(DOC_DIR)/Doxyfile\"" > $(DOC_DIR)/Doxyfile.make ; \
#    echo "PROJECT_NAME           = \"$(TARGET) Firmware\"" >> $(DOC_DIR)/Doxyfile.make ; \
	echo "INPUT                  = \"$(SOURCE_DIR)\"" >> $(DOC_DIR)/Doxyfile.make ; \
	echo "OUTPUT_DIRECTORY       = \"$(DOC_OUTPUT_DIR)\"" >> $(DOC_DIR)/Doxyfile.make ; \
	echo "Generate doxygen documentation"; \
	$(DOXYGEN) $(DOC_DIR)/Doxyfile.make

.PHONY: modules
modules: $(OBJECTS)

$(BUILD_DIR)/%.o: %.c $(BUILD_DIR)/.dir
	$(CC) -c $< $(CFLAGS) -o $@

$(BUILD_DIR)/%.o: %.s $(BUILD_DIR)/.dir
	$(AS) $< $(ASFLAGS) -o $@

clean:
	rm -rf $(BUILD_DIR) $(DOC_OUTPUT_DIR) $(DOC_DIR)/Doxyfile.make

# Mark .dir files as not being intermediate, otherwise it will be deleted and 
# subsequently recreated forcing rebuild of all objects
.PRECIOUS: %/.dir
%/.dir: 
	mkdir -p $*
	touch $@

program:
	$(PICPROGRAMMER) -P24FJ64GB002 -E -M -F./$(BUILD_DIR)/$(TARGET).hex -L

.PHONY: help
help:
	@echo "make clean"
	@echo "make all"
	@echo "make program"
	@echo "make doc"

