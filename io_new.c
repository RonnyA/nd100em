#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h> // For read() system call
#include <poll.h>   // For pollfd struct and POLLIN
#include "nd100.h"
#include "io_new.h"
#include "iox/deviceTerminal.h"

Device *terminal;

void IO_Init(void)
{
    DeviceManager_Init(LOG_WARNING);
    //DeviceManager_Init(LOG_DEBUG);
    DeviceManager_AddAllDevices();

    // Get a reference to the terminal device
    terminal = DeviceManager_GetDeviceByAddress(0300);
    if (!terminal)
    {
        printf("Failed to get terminal device\n");
        exit(1);
    }
}

void IO_Destroy(void)
{
    DeviceManager_Destroy();
}

uint16_t IO_Read(uint32_t address)
{
    return DeviceManager_Read(address);
}

void IO_Write(uint32_t address, uint16_t value)
{
    DeviceManager_Write(address, value);
}

uint16_t IO_Ident(uint16_t level)
{
    return DeviceManager_Ident(level);
}

void IO_Tick(void)
{
    // Check for input from console and queue it to terminal device
    char ch;
    static int tick_count = 0;

    // Check for input from console every 5000 ticks
    if (tick_count++ >= 5000)
    {

        tick_count = 0;

        if (terminal)
        {
            char recv_data[1024];
            int numbytes = 10;
            int numread;

            // Try to read from stdin
            struct pollfd fds;
            fds.fd = 0; // stdin
            fds.events = POLLIN;
            if (poll(&fds, 1, 0) > 0 && (fds.revents & POLLIN)) {
                numread = read(0, recv_data, numbytes);                
            } else {
                numread = 0;
            }
            
            for (int i = 0; i < numread; i++)
            {
                ch = (char)recv_data[i];

                // ND doesnt like \n
                if (ch == '\n')
                {
                    ch = '\r';
                }
                Terminal_QueueKeyCode(terminal, ch);
            }
        }
    }
    // Tick all devices, and check for interrupts
    uint16_t interruptBits = DeviceManager_Tick();
    device_interrupt(interruptBits);
}

void mopc_out(char ch)
{
    IO_Write(0305, ch); // write to console device (0305)
}

int mopc_in(char *chptr)
{
    *chptr = IO_Read(0300); // read from console device
    return 1;
}

void io_op(ushort ioadd)
{
    // Even addresses are read operations, odd addresses are write operations
    if (ioadd & 1)
    {
        // Odd address - write operation
        IO_Write(ioadd, gA);
    }
    else
    {
        // Even address - read operation
        gA = IO_Read(ioadd);
    }
}
