MODULE:= rtmouse
TARGET:= $(MODULE).ko

all: $(TARGET)

$(TARGET): $(MODULE).c
	make -C /usr/src/linux M=$(PWD) modules

obj-m:= $(MODULE).o

clean:
	rm -f *.o *.order *.mod.c *.symvers *.ko
